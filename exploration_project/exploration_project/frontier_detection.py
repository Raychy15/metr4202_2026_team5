import rclpy
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


    def map_callback(self, msg: OccupancyGrid):

        #all map info needed
        width = msg.info.width
        height = msg.info.height
        grid = msg.data
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.get_logger().info(
            'Received map: %d x %d' % (width, height)
        )

        # store known free cell locations that are next to unknown cells
        border_cells_locations = []

        for y in range(height):
            for x in range(width):
                index = y * width + x
                if grid[index] == 0:  # free cell cell
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

        # Simplify to one location for each cluster to track and also size
        frontier_locations = []

        for cluster in clusters:

            # number of frontier cells in this cluster
            cluster_size = len(cluster)

            # Find centre of cluster in grid coordinates
            centre_x, centre_y = self.frontier_centre(
                cluster
            )

            # Find actual frontier cell closest to that centre
            grid_x, grid_y = self.cell_closest_to_centre(
                cluster,
                centre_x,
                centre_y
            )

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

        # Print frontier locations
        #for i in range(len(frontier_locations)):
        #
        #    x, y = frontier_locations[i]
        #
        #    self.get_logger().info(
        #        'Frontier %d location: x = %.2f, y = %.2f'
        #        % (i + 1, x, y)
        #    )


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


    def cell_closest_to_centre(self, cluster, centre_x, centre_y):

        # assuming the first cell is the closest to have a starting comparison
        closest_cell = cluster[0]

        x, y = closest_cell

        #pythagorus for distance calc
        closest_distance = (
            (x - centre_x) ** 2
            + (y - centre_y) ** 2
        ) ** 0.5

        # compare every frontier cell to the calculated centre
        for cell in cluster:

            x, y = cell

            distance = (
                (x - centre_x) ** 2
                + (y - centre_y) ** 2
            ) ** 0.5

            if distance < closest_distance:
                #update closest cell with new one
                closest_distance = distance
                closest_cell = cell

        return closest_cell



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