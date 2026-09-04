import numpy as np


class LidarProcessor:
    """
    Basic CARLA LiDAR point-cloud processor.

    Converts raw LiDAR points into a simple obstacle representation
    that can later be used by M1 planning.
    """

    def __init__(
        self,
        obstacle_distance=20.0,
        min_height=-2.0,
        max_height=2.0,
    ):
        self.obstacle_distance = obstacle_distance
        self.min_height = min_height
        self.max_height = max_height

    def process(self, points):
        """
        Process a LiDAR point cloud.

        Args:
            points: numpy array of shape (N, 3) or (N, 4).
                    Columns are x, y, z, and optionally intensity.

        Returns:
            list of obstacle points.
        """

        if points is None:
            return []

        points = np.asarray(points)

        if points.size == 0:
            return []

        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError(
                "LiDAR points must have shape (N, 3) or (N, 4)."
            )

        xyz = points[:, :3]

        distance = np.linalg.norm(
            xyz[:, :2],
            axis=1,
        )

        valid = (
            (distance <= self.obstacle_distance)
            & (xyz[:, 2] >= self.min_height)
            & (xyz[:, 2] <= self.max_height)
        )

        filtered = xyz[valid]

        return [
            {
                "x": float(point[0]),
                "y": float(point[1]),
                "z": float(point[2]),
                "distance": float(
                    np.linalg.norm(point[:2])
                ),
            }
            for point in filtered
        ]