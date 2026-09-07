import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid


def main(args=None):
    rclpy.init(args=args)

    border_detector = BorderDetector()

    rclpy.spin(border_detector)


class BorderDetector(Node):

    def __init__(self):
        super().__init__('border_detector')

        # Create a subscriber to the map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)

        self.subscription


    def map_callback(self, msg: OccupancyGrid):

        width = msg.info.width
        height = msg.info.height
        grid = msg.data

        self.get_logger().info(
            'Received map: %d x %d' % (width, height)
        )

        # count number of known free cells that are next to unknown cells
        border_number = 0
        border_cells_locations = []

        for y in range(height):
            for x in range(width):
                index = y * width + x
                if grid[index] == 0:  # free cell cell
                    if self.is_border_cell(x, y, width, height, grid):
                        border_number += 1
                        border_cells_locations.append((x, y)) #keep track of desired locations


        self.get_logger().info(
            'Number of border cells: %d' % border_number
        )   

    def is_border_cell(self, x, y, width, height, grid):

        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0: #grid cell known
                    continue
                nx = x + dx
                ny = y + dy
                if nx < 0 or nx >= width or ny < 0 or ny >= height: #outside of map range
                    continue
                neighbor_index = ny * width + nx
                if grid[neighbor_index] == -1:  # unknown cell
                    return True
        return False

    def cluster_frontiers(self, frontier_cells):

        frontier_set = set(frontier_cells)
        visited = set()
        clusters = []

        for cell in frontier_cells:

            if cell in visited:
                continue

        queue = [cell]
        visited.add(cell)

        cluster = []

        while queue:

            current = queue.pop(0)
            cluster.append(current)

            x, y = current

            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:

                    if dx == 0 and dy == 0:
                        continue

                    neighbour = (x + dx, y + dy)

                    if neighbour in frontier_set and neighbour not in visited:

                        visited.add(neighbour)
                        queue.append(neighbour)

        clusters.append(cluster)

        return clusters
    


if __name__ == '__main__':
    main()