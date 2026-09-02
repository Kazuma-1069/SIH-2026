class DepthEstimator:
    """
    Lightweight monocular distance estimator.

    This is an approximation based on known object dimensions
    and bounding-box height.

    For CARLA, this can later be replaced or supplemented
    with true depth-camera data.
    """

    KNOWN_HEIGHTS = {
        "person": 1.7,
        "pedestrian": 1.7,
        "car": 1.5,
        "truck": 3.0,
        "bus": 3.0,
        "motorcycle": 1.4,
        "bicycle": 1.2,
        "traffic light": 2.5,
        "stop sign": 2.0,
    }

    def __init__(self, focal_length=700.0):
        self.focal_length = focal_length

    def estimate_distance(self, bbox, class_name):
        """
        Estimate distance in meters.

        Args:
            bbox: [x1, y1, x2, y2]
            class_name: detected class

        Returns:
            float or None
        """

        x1, y1, x2, y2 = bbox

        pixel_height = abs(y2 - y1)

        if pixel_height <= 0:
            return None

        known_height = self.KNOWN_HEIGHTS.get(
            class_name.lower()
        )

        if known_height is None:
            return None

        distance = (
            known_height * self.focal_length
        ) / pixel_height

        return round(distance, 2)

    def add_distance(self, tracked_objects):
        """
        Add estimated distance to every tracked object.
        """

        output = []

        for obj in tracked_objects:

            item = dict(obj)

            distance = self.estimate_distance(
                item["bbox"],
                item["class_name"],
            )

            item["distance"] = distance

            output.append(item)

        return output