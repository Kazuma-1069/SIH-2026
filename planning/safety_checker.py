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

class BubbleShield:
    """
    Safety bubble around the ego vehicle.

    Any obstacle inside the bubble is considered an
    immediate safety threat.
    """

    def __init__(
        self,
        obstacle_map,
        radius=2.0,
        emergency_radius=1.0,
    ):
        if radius <= 0:
            raise ValueError("radius must be greater than zero")

        if emergency_radius <= 0:
            raise ValueError(
                "emergency_radius must be greater than zero"
            )

        if emergency_radius > radius:
            raise ValueError(
                "emergency_radius cannot exceed radius"
            )

        self.obstacle_map = obstacle_map
        self.radius = float(radius)
        self.emergency_radius = float(emergency_radius)

    def check(self, ego_position):
        """
        Evaluate obstacles around the ego vehicle.

        Returns:
            {
                "safe": bool,
                "emergency": bool,
                "distance": float,
                "reason": str
            }
        """

        if (
            not isinstance(ego_position, (list, tuple))
            or len(ego_position) != 2
        ):
            raise ValueError(
                "ego_position must contain [x, y]"
            )

        ego_x, ego_y = ego_position

        nearest_distance = (
            self.obstacle_map.distance_to_nearest_obstacle(
                (ego_x, ego_y)
            )
        )

        if nearest_distance <= self.emergency_radius:
            return {
                "safe": False,
                "emergency": True,
                "distance": nearest_distance,
                "reason": "EMERGENCY_BUBBLE_VIOLATION",
            }

        if nearest_distance <= self.radius:
            return {
                "safe": False,
                "emergency": False,
                "distance": nearest_distance,
                "reason": "BUBBLE_VIOLATION",
            }

        return {
            "safe": True,
            "emergency": False,
            "distance": nearest_distance,
            "reason": "BUBBLE_CLEAR",
        }

    def check_path(self, path):
        """
        Check whether any path point enters the Bubble Shield.
        """

        if not path:
            return {
                "safe": False,
                "path_safe": False,
                "reason": "EMPTY_PATH",
                "minimum_distance": float("inf"),
            }

        minimum_distance = float("inf")

        for point in path:
            if (
                not isinstance(point, (list, tuple))
                or len(point) != 2
            ):
                return {
                    "safe": False,
                    "path_safe": False,
                    "reason": "INVALID_PATH_POINT",
                    "minimum_distance": minimum_distance,
                }

            distance = (
                self.obstacle_map
                .distance_to_nearest_obstacle(point)
            )

            minimum_distance = min(
                minimum_distance,
                distance
            )

            if distance <= self.emergency_radius:
                return {
                    "safe": False,
                    "path_safe": False,
                    "reason": "EMERGENCY_BUBBLE_VIOLATION",
                    "minimum_distance": minimum_distance,
                }

            if distance <= self.radius:
                return {
                    "safe": False,
                    "path_safe": False,
                    "reason": "BUBBLE_VIOLATION",
                    "minimum_distance": minimum_distance,
                }

        return {
            "safe": True,
            "path_safe": True,
            "reason": "BUBBLE_CLEAR",
            "minimum_distance": minimum_distance,
        }

    def validate_path(self, path):
        """
        Check whether any path point enters the Bubble Shield.
        """

        if not path:
            return {
                "safe": False,
                "emergency": False,
                "reason": "EMPTY_PATH",
            }

        for point in path:
            if len(point) != 2:
                return {
                    "safe": False,
                    "emergency": False,
                    "reason": "INVALID_PATH_POINT",
                }

            distance = (
                self.obstacle_map
                .distance_to_nearest_obstacle(point)
            )

            if distance <= self.emergency_radius:
                return {
                    "safe": False,
                    "emergency": True,
                    "reason": "EMERGENCY_BUBBLE_VIOLATION",
                }

            if distance <= self.radius:
                return {
                    "safe": False,
                    "emergency": False,
                    "reason": "BUBBLE_VIOLATION",
                }

        return {
            "safe": True,
            "emergency": False,
            "reason": "BUBBLE_CLEAR",
        }
        