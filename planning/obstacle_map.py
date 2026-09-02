class ObstacleMap:
    """
    Grid-based obstacle map for path planning.

    M1 receives obstacle information from the perception module (M2)
    and converts it into a grid that A* / Dijkstra can use.
    """

    def __init__(self, width=20, height=20):
        self.width = width
        self.height = height

        # 0 = free, 1 = obstacle
        self.grid = [
            [0 for _ in range(width)]
            for _ in range(height)
        ]

    def clear(self):
        """Clear all obstacles from the grid."""
        self.grid = [
            [0 for _ in range(self.width)]
            for _ in range(self.height)
        ]

    def add_obstacle(self, x, y):
        """Mark a grid cell as occupied."""
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 1

    def remove_obstacle(self, x, y):
        """Mark a grid cell as free."""
        if 0 <= x < self.width and 0 <= y < self.height:            
            self.grid[y][x] = 0

    def is_occupied(self, x, y):
        """Return True if the cell contains an obstacle."""
        if not (0 <= x < self.width and 0 <= y < self.height):
            return True

        return self.grid[y][x] == 1

    def update_from_objects(self, objects):
        """
        Convert perception objects into occupied grid cells.

        Each object should contain:
            {
                "bbox": [x1, y1, x2, y2]
            }
        """

        self.clear()

        for obj in objects:
            bbox = obj.get("bbox")

            if not bbox or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = bbox

            # Use the center of the bounding box as
            # the approximate obstacle position.
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Keep the prototype mapping simple.
            grid_x = int(center_x / 10)
            grid_y = int(center_y / 10)
            
            self.add_obstacle(grid_x, grid_y)

    def get_grid(self):
        """Return the current occupancy grid."""
        return self.grid