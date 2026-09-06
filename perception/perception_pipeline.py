import time
from perception.hazard_detector import HazardDetector
from perception.drivable_space import DrivableSpaceDetector
from perception.lidar_processor import LidarProcessor
from perception.sensor_fusion import SensorFusion
from perception.risk_assessment import RiskAssessor
from perception.trajectory_predictor import TrajectoryPredictor

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
        lidar_processor=None,
    ):
        self.detector = detector
        self.tracker = tracker
        self.depth_estimator = depth_estimator
        self.lidar_processor = lidar_processor
        self.hazard_detector = HazardDetector()
        self.drivable_space_detector = DrivableSpaceDetector()
        self.lidar_processor = lidar_processor or LidarProcessor()
        self.sensor_fusion = SensorFusion()
        self.trajectory_predictor = TrajectoryPredictor()
        self.risk_assessor = RiskAssessor()
        self.frame_id = 0

    def process_frame(self, frame, lidar_points=None):

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
        drivable_mask = self.drivable_space_detector.detect(
            frame
        )
        road_edges = self.drivable_space_detector.get_road_edges(
            drivable_mask
        )
        lidar_obstacles = self.lidar_processor.process(
            lidar_points
        )
        environment = self.sensor_fusion.fuse(
            objects=tracked_objects,
            hazards=hazards,
            lidar_obstacles=lidar_obstacles,
            road_edges=road_edges,
            drivable_mask=drivable_mask
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

        predictions = self.trajectory_predictor.update(
            tracked_objects
        )
        prediction_by_track = {
            prediction["track_id"]: prediction
            for prediction in predictions
        }
        for obj in tracked_objects:
            prediction = prediction_by_track.get(
                obj["track_id"]
            )
            if prediction is not None:
                obj["predicted_position"] = prediction[
                    "future_position"
                ]

        risk_assessments = self.risk_assessor.assess(
            tracked_objects,
            image_width=width,
        )

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
                    position=obj.get(
                        "position"
                    ),
                    velocity=obj.get("velocity"),
                    predicted_position=obj.get("predicted_position"),
                    age=int(obj.get("age", 0)),
                )
            )

        return PerceptionOutput(
    timestamp=time.time(),
    frame_id=self.frame_id,
    image_width=width,
    image_height=height,
    objects=perception_objects,
    source="CARLA_RGB_CAMERA",
    hazards=hazards,
    drivable_mask=drivable_mask,
    road_edges=road_edges,
    lidar_obstacles=lidar_obstacles,
    predictions=predictions,
    risk_assessments=risk_assessments,
    environment=environment,
)