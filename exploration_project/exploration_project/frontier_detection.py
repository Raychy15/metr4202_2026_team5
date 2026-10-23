import rclpy
import math
from collections import deque
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray



def main(args=None):
    rclpy.init(args=args)

    #create border/frontier detection node
    frontier_detector = frontierDetector()

    # Execute the node
    rclpy.spin(frontier_detector)


class frontierDetector(Node):

    def __init__(self):
        super().__init__('frontier_detector')

        # Create a subscriber to the map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)

        # Publish detected frontier candidates
        self.frontier_publisher = self.create_publisher(
            Float32MultiArray,
            'frontier_candidates',
            10
            )
        self.subscription

        # Goal search for each frontier piece: try the strict rule first,
        # then look further away and allow the goal closer to UNKNOWN cells
        # (never closer to known walls). Each entry is
        # (search radius around frontier cells, clearance from unknown) in m.
        # Before, a piece with no goal under the first rule was thrown away,
        # which left the robot with nothing to go to near it
        self.obstacle_clearance = 0.30
        self.goal_tiers = [(0.35, 0.20), (0.60, 0.10), (1.00, 0.0)]

        # Free cells this close to a known wall are not used as frontier
        # cells. Unknown cells right next to a wall are the inside of the
        # wall or a gap in its mapped surface, not unexplored space, and they
        # kept pulling the robot back into corridors it had already explored
        self.wall_frontier_gap = 0.15
        self._offsets = {}


    def map_callback(self, msg: OccupancyGrid):

        #all map info needed
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.get_logger().info(
            'Received map: %d x %d' % (width, height)
        )

        # slam_toolbox leaves small unknown holes inside explored floor
        # (beams that graze a wall). They are not real frontiers and every
        # cell near them failed the 'clear of unknown' check, so fill them in
        grid = self.fill_unknown_holes(msg.data, width, height)

        # distance from every cell to the nearest wall / unknown cell, worked
        # out once per map instead of searching around every candidate cell
        self.obstacle_dist = self.distance_grid(
            grid, width, height, lambda v: v > 50,
            int(math.ceil(self.obstacle_clearance / resolution)) + 1)
        self.unknown_dist = self.distance_grid(
            grid, width, height, lambda v: v == -1,
            int(math.ceil(max(t[1] for t in self.goal_tiers) / resolution)) + 1)

        # store known free cell locations that are next to unknown cells
        border_cells_locations = []
        wall_gap_cells = self.wall_frontier_gap / resolution

        for y in range(height):
            for x in range(width):
                index = y * width + x
                if grid[index] == 0 and self.obstacle_dist[index] > wall_gap_cells:  # free cell, not hugging a wall
                    if self.is_border_cell(x, y, width, height, grid):
                        border_cells_locations.append((x, y)) #keep track of desired locations


        self.get_logger().info(
            'Number of border cells: %d' % len(border_cells_locations)
        )

        #group connected frontier cells with breath first
        clusters = self.cluster_frontiers(border_cells_locations)

        self.get_logger().info(
        'Number of frontier clusters: %d' % len(clusters)
        )   

        # FIX: split long frontiers into pieces of at most ~1.5 m. A long
        # frontier used to become ONE candidate; if the robot was already
        # next to it at start-up, there was nothing else to go to
        max_cells = max(10, int(1.5 / resolution))
        segments = []
        for cluster in clusters:
            segments.extend(self.split_cluster(cluster, max_cells))

        self.get_logger().info(
            'Number of frontier segments: %d' % len(segments)
        )

        # Simplify to one location for each cluster to track and also size
        frontier_locations = []
        rejected = 0
        relaxed = 0

        for cluster in segments:

            # number of frontier cells in this cluster
            cluster_size = len(cluster)

            # Find centre of cluster in grid coordinates
            centre_x, centre_y = self.frontier_centre(
                cluster
            )

            # Find actual frontier cell closest to that centre
            safe_cell = self.cell_closest_to_centre(
                cluster,
                centre_x,
                centre_y,
                width,
                height,
                grid,
                resolution
            )

            if safe_cell is None:
                rejected += 1
                continue

            (grid_x, grid_y), tier = safe_cell
            if tier > 0:
                relaxed += 1

            # Convert grid coordinates to real map coordinates
            world_x, world_y = self.convert_grid_to_worldmap(
                grid_x,
                grid_y,
                resolution,
                origin_x,
                origin_y
            )


            frontier_locations.append(
                (world_x, world_y, cluster_size)
            )

        # one summary line instead of one line per rejected piece
        self.get_logger().info(
            'Frontier goals: %d published (%d with a relaxed goal), '
            '%d pieces with no collision-safe goal'
            % (len(frontier_locations), relaxed, rejected)
        )

        # create message to send frontier candidates
        frontier_message = Float32MultiArray()

        frontier_message.data = []

        for frontier in frontier_locations:

            x, y, size = frontier

            frontier_message.data.append(float(x))
            frontier_message.data.append(float(y))
            frontier_message.data.append(float(size))

        # publish frontier candidates
        self.frontier_publisher.publish(frontier_message)


    def is_border_cell(self, x, y, width, height, grid):

        #need to check surrounding cells (8 cells around a cell)
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0: #ignore current grid cell
                    continue
                nx = x + dx
                ny = y + dy
                if nx < 0 or nx >= width or ny < 0 or ny >= height: #outside of map range
                    continue
                neighbor_index = ny * width + nx
                if grid[neighbor_index] == -1:  # unknown cell -> free cell is a frontier
                    return True
        return False


    def cluster_frontiers(self, frontier_cells):
        #for each frontier, if visitied already skip
        # if not, run graph search BFS from that location
        # Collect all connected unknown cells and store

        frontier_set = set(frontier_cells)
        #store cells already checked by bfs
        visited = set()
        #store frontiers as groups of clustsers
        clusters = []

        for cell in frontier_cells:
            #check if cell already visited
            if cell in visited:
                continue #its already in another cluster

            #do BFS from this cell
            queue = [cell]
            visited.add(cell)

            cluster = []
            #go until there are no more connected frontier cells left to check
            while queue:

                current = queue.pop(0)
                cluster.append(current)

                x, y = current

                #check surrounding cells
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:

                        if dx == 0 and dy == 0:
                            continue

                        neighbour = (x + dx, y + dy)
                        #add connected cells to queue
                        if neighbour in frontier_set and neighbour not in visited:

                            visited.add(neighbour)
                            queue.append(neighbour)
                        
            if len(cluster) < 5: #current decision to ignore clusters that are too small
                continue
        
            clusters.append(cluster)

        return clusters


    def split_cluster(self, cluster, max_cells):
        # FIX: split a cluster into pieces along its longest direction
        n = len(cluster)
        if n <= max_cells:
            return [cluster]

        mean_x = sum(c[0] for c in cluster) / n
        mean_y = sum(c[1] for c in cluster) / n
        sxx = sum((c[0] - mean_x) ** 2 for c in cluster)
        syy = sum((c[1] - mean_y) ** 2 for c in cluster)
        sxy = sum((c[0] - mean_x) * (c[1] - mean_y) for c in cluster)

        # direction of largest spread (principal axis of the cells)
        angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        ux, uy = math.cos(angle), math.sin(angle)

        ordered = sorted(
            cluster,
            key=lambda c: (c[0] - mean_x) * ux + (c[1] - mean_y) * uy
        )

        pieces = int(math.ceil(n / float(max_cells)))
        size = int(math.ceil(n / float(pieces)))
        return [ordered[i:i + size] for i in range(0, n, size)]


    #currently only finds average of group of cells
    #might be inaccurate depending on shape of cluster
    #if this is has faults I need to add a section that chooses the closest cell
    # in cluster to the centre found
    def frontier_centre(self, cluster):
        #find one centre point of cluster to pass on
        x_total = 0
        y_total = 0
        
        for cell in cluster:
            x,y = cell

            #find average of x and y location
            x_total += x
            y_total += y

        centre_x = x_total/len(cluster)
        centre_y = y_total/len(cluster)

        return centre_x, centre_y


    def fill_unknown_holes(self, data, width, height):
        # copy of the map with small unknown holes inside free space set to
        # free: an unknown cell with at least 6 of its 8 neighbours free.
        # The edge of a real unknown area has at most 5 free neighbours, so
        # real frontiers are kept. Two passes clear holes up to 2 cells
        grid = list(data)
        for _ in range(2):
            changed = 0
            for y in range(1, height - 1):
                row = y * width
                for x in range(1, width - 1):
                    i = row + x
                    if grid[i] != -1:
                        continue
                    free = 0
                    for j in (i - width - 1, i - width, i - width + 1, i - 1,
                              i + 1, i + width - 1, i + width, i + width + 1):
                        if 0 <= grid[j] <= 50:
                            free += 1
                    if free >= 6:
                        grid[i] = 0
                        changed += 1
            if changed == 0:
                break
        return grid


    def distance_grid(self, grid, width, height, is_source, max_cells):
        # distance (in cells) from every cell to the nearest source cell,
        # only worked out up to max_cells (further cells get max_cells + 1).
        # Spreads outwards from the sources keeping track of the nearest one
        n = width * height
        far = float(max_cells + 1)
        dist = [far] * n
        source = [-1] * n
        queue = deque()
        for i in range(n):
            if is_source(grid[i]):
                dist[i] = 0.0
                source[i] = i
        # only sources next to a non-source cell can spread anywhere
        for i in range(n):
            if dist[i] == 0.0:
                x, y = i % width, i // width
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height and dist[ny * width + nx] != 0.0:
                            queue.append(i)
                            break
                    else:
                        continue
                    break
        while queue:
            i = queue.popleft()
            x, y = i % width, i // width
            s = source[i]
            sx, sy = s % width, s // width
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if not (0 <= nx < width and 0 <= ny < height):
                        continue
                    j = ny * width + nx
                    d = math.hypot(nx - sx, ny - sy)
                    if d < dist[j] and d <= max_cells:
                        dist[j] = d
                        source[j] = s
                        queue.append(j)
        return dist


    def is_safe_cell(self, x, y, width, height, grid, resolution,
                 obstacle_clearance=0.30, unknown_clearance=0.20):
        # FIX: was 0.25 / 0.12. The waffle's rear corners are 0.24 m from
        # base_link, so 0.25 m from a wall cell centre is touching distance
        # when it turns; and 0.12 m from unknown could be 0.12 m from a wall
        # that has not been mapped yet

        if grid[y * width + x] != 0:
            return False

        # too close to the edge of the map
        radius = int(math.ceil(obstacle_clearance / resolution))
        if x < radius or y < radius or x >= width - radius or y >= height - radius:
            return False

        index = y * width + x
        if self.obstacle_dist[index] * resolution <= obstacle_clearance:
            return False
        if unknown_clearance > 0.0 and self.unknown_dist[index] * resolution <= unknown_clearance:
            return False
        return True



    def search_offsets(self, search_radius, resolution):
        # cell offsets within search_radius, nearest first (cached)
        key = (search_radius, resolution)
        if key not in self._offsets:
            radius = max(1, int(math.ceil(search_radius / resolution)))
            self._offsets[key] = sorted(
                (
                    (dx, dy)
                    for dy in range(-radius, radius + 1)
                    for dx in range(-radius, radius + 1)
                    if math.hypot(dx, dy) * resolution <= search_radius
                ),
                key=lambda d: d[0] ** 2 + d[1] ** 2
            )
        return self._offsets[key]


    def cell_closest_to_centre(self, cluster, centre_x, centre_y, width, height, grid, resolution):
        # returns ((x, y), tier) for the safe cell closest to the cluster
        # centre, using the strictest goal tier that finds one, or None

        # Start with frontier cells closest to the cluster centre
        ordered = sorted(
            cluster,
            key=lambda c: (
                (c[0] - centre_x) ** 2
                + (c[1] - centre_y) ** 2
            )
        )

        samples = ordered[:15]

        # Also sample across the frontier edge
        if len(ordered) > 15:
            step = max(1, len(ordered) // 20)
            samples += ordered[::step][:20]

        for tier, (search_radius, unknown_clearance) in enumerate(self.goal_tiers):
            offsets = self.search_offsets(search_radius, resolution)
            best_cell = None
            best_cost = float('inf')
            tested = set()

            for fx, fy in samples:
                for dx, dy in offsets:

                    x = fx + dx
                    y = fy + dy

                    if (x, y) in tested:
                        continue

                    tested.add((x, y))

                    if not (0 <= x < width and 0 <= y < height):
                        continue

                    if not self.is_safe_cell(
                        x, y, width, height, grid, resolution,
                        self.obstacle_clearance, unknown_clearance
                    ):
                        continue

                    cost = (
                        (x - centre_x) ** 2
                        + (y - centre_y) ** 2
                    )

                    if cost < best_cost:
                        best_cost = cost
                        best_cell = (x, y)

            if best_cell is not None:
                return best_cell, tier

        return None



    def convert_grid_to_worldmap(self, grid_x, grid_y, resolution, origin_x,
                                 origin_y):
        #convert grid cell coords to map coords in metres??
        #projewct brief says map resol is 0.05m --> each grid cell is 
        # 5cm wide
        world_x = (
        origin_x
        + (grid_x + 0.5) * resolution # +0.5 makes sure location is tracking centre of cell
         )

        world_y = (
        origin_y
        + (grid_y + 0.5) * resolution
        )

        return world_x, world_y


    
if __name__ == '__main__':
    main()
