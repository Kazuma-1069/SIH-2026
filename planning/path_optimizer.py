class PathOptimizer:
    """
    Optimizes a grid-based path by removing unnecessary
    intermediate waypoints.

    The optimizer does not change the path's start or goal.
    """

    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map

    def optimize(self, path):
        """
        Simplify a path while keeping it collision-free.

        Args:
            path: List of (x, y) grid coordinates.

        Returns:
            Optimized list of (x, y) coordinates.
        """

        if not path:
            return []

        if len(path) <= 2:
            return path.copy()

        optimized = [path[0]]

        current_index = 0

        while current_index < len(path) - 1:
            farthest_index = current_index + 1

            # Try to connect the current point directly
            # to the farthest possible later point.
            for candidate_index in range(
                current_index + 1,
                len(path)
            ):
                if self._line_is_clear(
                    path[current_index],
                    path[candidate_index]
                ):
                    farthest_index = candidate_index

            optimized.append(path[farthest_index])
            current_index = farthest_index

        return optimized

    def _line_is_clear(self, start, end):
        """
        Check whether a straight grid connection between
        two points is free of obstacles.

        Uses linear interpolation between the points.
        """

        x1, y1 = start
        x2, y2 = end

        dx = x2 - x1
        dy = y2 - y1

        steps = max(abs(dx), abs(dy))

        if steps == 0:
            return not self.obstacle_map.is_occupied(
                x1,
                y1
            )

        for i in range(steps + 1):
            x = round(x1 + dx * i / steps)
            y = round(y1 + dy * i / steps)

            if self.obstacle_map.is_occupied(x, y):
                return False

        return True