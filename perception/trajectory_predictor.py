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

            track_id = obj.track_id
            center = obj.center

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