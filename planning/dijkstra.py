import heapq


class DijkstraPlanner:
    """
    Dijkstra path planner operating on a 2D occupancy grid.

    Grid convention:
        0 = free
        1 = obstacle
    """

    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map

    def get_neighbors(self, node, safety_distance=0.0, start=None, goal=None):
        """Return valid 4-directional neighboring cells."""
        x, y = node

        candidates = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]

        valid = []
        for cell in candidates:
            if not (0 <= cell[0] < self.obstacle_map.width and 0 <= cell[1] < self.obstacle_map.height):
                continue
            if self.obstacle_map.is_occupied(*cell):
                continue
            if safety_distance > 0:
                if cell != goal and cell != start:
                    if self.obstacle_map.distance_to_nearest_obstacle(cell) <= safety_distance:
                        continue
            valid.append(cell)

        return valid

    def find_path(self, start, goal, safety_distance=0.0):
        """
        Find the shortest path from start to goal.

        Returns:
            List of (x, y) grid coordinates.
            Returns [] if no path exists.
        """

        if self.obstacle_map.is_occupied(*start):
            return []

        if self.obstacle_map.is_occupied(*goal):
            return []

        distances = {start: 0}
        came_from = {}

        queue = [(0, start)]

        while queue:
            current_distance, current = heapq.heappop(queue)

            if current_distance > distances.get(
                current, float("inf")
            ):
                continue

            if current == goal:
                return self._reconstruct_path(
                    came_from,
                    current
                )

            for neighbor in self.get_neighbors(current, safety_distance=safety_distance, start=start, goal=goal):
                new_distance = current_distance + 1

                if new_distance < distances.get(
                    neighbor, float("inf")
                ):
                    distances[neighbor] = new_distance
                    came_from[neighbor] = current

                    heapq.heappush(
                        queue,
                        (new_distance, neighbor)
                    )

        return []

    @staticmethod
    def _reconstruct_path(came_from, current):
        """Reconstruct path from goal back to start."""
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        return path