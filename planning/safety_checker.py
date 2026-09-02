class SafetyChecker:
    """
    Checks whether a planned path is safe with respect to
    the current obstacle map.
    """

    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map

    def is_path_safe(self, path):
        """
        Check whether every point in the path is valid and free.

        Args:
            path: List of (x, y) grid coordinates.

        Returns:
            True if the complete path is safe, otherwise False.
        """

        if not path:
            return False

        for x, y in path:
            if self.obstacle_map.is_occupied(x, y):
                return False

        return True

    def validate_path(self, path):
        """
        Validate a path and return a structured safety result.
        """

        if not path:
            return {
                "safe": False,
                "reason": "EMPTY_PATH"
            }

        for x, y in path:
            if not (
                0 <= x < self.obstacle_map.width
                and 0 <= y < self.obstacle_map.height
            ):
                return {
                    "safe": False,
                    "reason": "PATH_OUT_OF_BOUNDS"
                }

            if self.obstacle_map.is_occupied(x, y):
                return {
                    "safe": False,
                    "reason": "OBSTACLE_ON_PATH"
                }

        return {
            "safe": True,
            "reason": "PATH_CLEAR"
        }