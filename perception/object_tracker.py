import math


class ObjectTracker:
    """
    Lightweight centroid-based multi-object tracker.

    Designed to keep dependencies minimal and work with
    YOLO detections.
    """

    def __init__(
        self,
        max_distance=80.0,
        max_missing_frames=10,
    ):
        self.max_distance = max_distance
        self.max_missing_frames = max_missing_frames

        self.next_track_id = 1
        self.tracks = {}

    @staticmethod
    def _centroid(bbox):
        x1, y1, x2, y2 = bbox

        return (
            (x1 + x2) / 2.0,
            (y1 + y2) / 2.0,
        )

    @staticmethod
    def _distance(point_a, point_b):
        return math.sqrt(
            (point_a[0] - point_b[0]) ** 2
            + (point_a[1] - point_b[1]) ** 2
        )

    def _create_track(self, detection):

        bbox = detection["bbox"]

        track_id = self.next_track_id
        self.next_track_id += 1

        self.tracks[track_id] = {
            "track_id": track_id,
            "class_id": detection["class_id"],
            "class_name": detection["class_name"],
            "confidence": detection["confidence"],
            "bbox": bbox,
            "centroid": self._centroid(bbox),
            "missing_frames": 0,
        }

    def update(self, detections):
        """
        Update tracker using current frame detections.

        Args:
            detections: list returned by YOLODetector.detect()

        Returns:
            list of tracked objects.
        """

        if not detections:
            for track in self.tracks.values():
                track["missing_frames"] += 1

            self._remove_old_tracks()

            return list(self.tracks.values())

        detection_centroids = [
            self._centroid(detection["bbox"])
            for detection in detections
        ]

        matched_tracks = set()
        matched_detections = set()

        candidate_matches = []

        for track_id, track in self.tracks.items():

            for detection_index, centroid in enumerate(
                detection_centroids
            ):

                if detection_index in matched_detections:
                    continue

                if (
                    track["class_id"]
                    != detections[detection_index]["class_id"]
                ):
                    continue

                distance = self._distance(
                    track["centroid"],
                    centroid,
                )

                if distance <= self.max_distance:
                    candidate_matches.append(
                        (
                            distance,
                            track_id,
                            detection_index,
                        )
                    )

        candidate_matches.sort(
            key=lambda item: item[0]
        )

        for (
            distance,
            track_id,
            detection_index,
        ) in candidate_matches:

            if track_id in matched_tracks:
                continue

            if detection_index in matched_detections:
                continue

            detection = detections[detection_index]

            track = self.tracks[track_id]

            track["bbox"] = detection["bbox"]
            track["centroid"] = self._centroid(
                detection["bbox"]
            )
            track["confidence"] = detection["confidence"]
            track["class_id"] = detection["class_id"]
            track["class_name"] = detection["class_name"]
            track["missing_frames"] = 0

            matched_tracks.add(track_id)
            matched_detections.add(detection_index)

        for detection_index, detection in enumerate(detections):

            if detection_index not in matched_detections:
                self._create_track(detection)

        for track_id, track in self.tracks.items():

            if track_id not in matched_tracks:
                if track["missing_frames"] == 0:
                    continue

                track["missing_frames"] += 1

        self._remove_old_tracks()

        return list(self.tracks.values())

    def _remove_old_tracks(self):

        expired_ids = [
            track_id
            for track_id, track in self.tracks.items()
            if track["missing_frames"]
            > self.max_missing_frames
        ]

        for track_id in expired_ids:
            del self.tracks[track_id]

    def reset(self):
        """Remove all active tracks."""

        self.tracks.clear()
        self.next_track_id = 1