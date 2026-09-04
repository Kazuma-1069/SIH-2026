import time
from perception.hazard_detector import HazardDetector

from interfaces.perception_output import (
    PerceptionObject,
    PerceptionOutput,
)


class PerceptionPipeline:
    """
    Complete M2 perception pipeline.

    CARLA camera frame
        ↓
    YOLO detection
        ↓
    Object tracking
        ↓
    Distance estimation
        ↓
    PerceptionOutput
    """

    def __init__(
        self,
        detector,
        tracker,
        depth_estimator=None,
    ):
        self.detector = detector
        self.tracker = tracker
        self.depth_estimator = depth_estimator
        self.hazard_detector = HazardDetector()
        self.frame_id = 0

    def process_frame(self, frame):

        if frame is None:
            raise ValueError(
                "Received empty camera frame."
            )

        self.frame_id += 1

        height, width = frame.shape[:2]

        # 1. YOLO detection
        detections = self.detector.detect(frame)

        # 2. Tracking
        tracked_objects = self.tracker.update(
            detections
        )
        hazards = self.hazard_detector.detect(
            detections
        )

        # 3. Distance estimation
        if self.depth_estimator is not None:
            tracked_objects = (
                self.depth_estimator.add_distance(
                    tracked_objects
                )
            )
        else:
            for obj in tracked_objects:
                obj["distance"] = None

        # 4. Convert to standard interface
        perception_objects = []

        for obj in tracked_objects:

            perception_objects.append(
                PerceptionObject(
                    track_id=int(
                        obj["track_id"]
                    ),
                    class_id=int(
                        obj["class_id"]
                    ),
                    class_name=str(
                        obj["class_name"]
                    ),
                    confidence=float(
                        obj["confidence"]
                    ),
                    bbox=list(
                        map(int, obj["bbox"])
                    ),
                    distance=obj.get(
                        "distance"
                    ),
                )
            )

        return PerceptionOutput(
    timestamp=time.time(),
    frame_id=self.frame_id,
    image_width=width,
    image_height=height,
    objects=perception_objects,
    source="CARLA_RGB_CAMERA",
    hazards=hazards
)