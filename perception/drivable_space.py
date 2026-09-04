import numpy as np


class DrivableSpaceDetector:
    """
    Basic drivable-space and road-edge estimator.

    Produces a binary mask from the camera image.
    White = estimated drivable area.
    Black = non-drivable area.
    """

    def detect(self, frame):
        if frame is None:
            raise ValueError(
                "Received empty camera frame."
            )

        if not isinstance(frame, np.ndarray):
            raise TypeError(
                "frame must be a numpy.ndarray"
            )

        if frame.size == 0:
            return None

        height, width = frame.shape[:2]

        mask = np.zeros(
            (height, width),
            dtype=np.uint8,
        )

        # Initial baseline: lower portion of the
        # camera image is treated as drivable.
        mask[int(height * 0.55):, :] = 255

        return mask

    def get_road_edges(self, mask):
        if mask is None:
            return []

        height, width = mask.shape[:2]

        edges = []

        for y in range(int(height * 0.55), height):
            row = mask[y]

            drivable_pixels = np.where(
                row > 0
            )[0]

            if len(drivable_pixels) == 0:
                continue

            edges.append(
                {
                    "y": int(y),
                    "left": int(drivable_pixels[0]),
                    "right": int(drivable_pixels[-1]),
                }
            )

        return edges