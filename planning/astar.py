# astar.py

# TODO: implement A* algorithm
import heapq


class AStarPlanner:
    """
    A* path planner operating on a 2D occupancy grid.

    Grid convention:
        0 = free
        1 = obstacle
    """

    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map

    @staticmethod
    def heuristic(a, b):
        """Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node, safety_distance=0.0, start=None, goal=None, corridor=None):
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
            if corridor is not None and cell not in corridor:
                continue
            if self.obstacle_map.is_occupied(*cell):
                continue
            if safety_distance > 0:
                if cell != goal and cell != start:
                    if self.obstacle_map.distance_to_nearest_obstacle(cell) <= safety_distance:
                        continue
            valid.append(cell)

        return valid

    def find_path(self, start, goal, safety_distance=0.0, corridor=None):
        """
        Find a path from start to goal.

        Returns:
            List of (x, y) grid coordinates.
            Returns [] if no path exists.
        """

        if self.obstacle_map.is_occupied(*start):
            return []

        if self.obstacle_map.is_occupied(*goal):
            return []

        open_set = []
        heapq.heappush(
            open_set,
            (0, start)
        )

        came_from = {}

        g_score = {
            start: 0
        }

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(
                    came_from,
                    current
                )

            for neighbor in self.get_neighbors(current, safety_distance=safety_distance, start=start, goal=goal, corridor=corridor):

                tentative_g = (
                    g_score[current] + 1
                )

                if tentative_g < g_score.get(
                    neighbor,
                    float("inf")
                ):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = (
                        tentative_g
                        + self.heuristic(neighbor, goal)
                    )

                    heapq.heappush(
                        open_set,
                        (f_score, neighbor)
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