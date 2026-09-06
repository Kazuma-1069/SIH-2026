import math


class TrajectoryPredictor:
    """
    Basic object trajectory predictor.

    Estimates future positions using
    previous object centers.
    """

    def __init__(self, history_size=5):
        self.history_size = history_size
        self.history = {}

    def update(self, objects):
        predictions = []

        for obj in objects:
            track_id = (
                obj.get("track_id", -1)
                if isinstance(obj, dict)
                else obj.track_id
            )
            center = (
                obj.get("centroid")
                if isinstance(obj, dict)
                else obj.center
            )
            if center is None and isinstance(obj, dict):
                bbox = obj.get("bbox", [0, 0, 0, 0])
                center = [
                    (bbox[0] + bbox[2]) / 2.0,
                    (bbox[1] + bbox[3]) / 2.0,
                ]

            if track_id not in self.history:
                self.history[track_id] = []

            self.history[track_id].append(center)

            if len(self.history[track_id]) > self.history_size:
                self.history[track_id].pop(0)

            future = self.predict(
                self.history[track_id]
            )

            predictions.append(
                {
                    "track_id": track_id,
                    "future_position": future,
                    "velocity": [
                        future[0] - center[0],
                        future[1] - center[1],
                    ],
                    "horizon_frames": 5,
                }
            )

        return predictions


    def predict(self, positions):

        if len(positions) < 2:
            return positions[-1]

        x1, y1 = positions[-2]
        x2, y2 = positions[-1]

        vx = x2 - x1
        vy = y2 - y1

        return [
            x2 + vx * 5,
            y2 + vy * 5,
        ]