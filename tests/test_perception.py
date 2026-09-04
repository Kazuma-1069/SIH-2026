import numpy as np

from perception.yolo_detector import YOLODetector
from perception.perception_pipeline import PerceptionPipeline
from perception.carla_perception import CarlaPerception


def test_yolo_detector_requires_ultralytics():
    assert YOLODetector is not None


def test_frame_is_valid_numpy_image():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    assert isinstance(frame, np.ndarray)
    assert frame.shape == (720, 1280, 3)
    assert frame.dtype == np.uint8


def test_frame_dimensions():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    height, width = frame.shape[:2]

    assert height == 720
    assert width == 1280


class FakeDetector:
    def detect(self, frame):
        return [
            {
                "track_id": 1,
                "class_id": 2,
                "class_name": "car",
                "confidence": 0.95,
                "bbox": [100, 100, 300, 250],
            }
        ]


class FakeTracker:
    def update(self, detections):
        return detections


def test_perception_pipeline():
    frame = np.zeros(
        (720, 1280, 3),
        dtype=np.uint8,
    )

    pipeline = PerceptionPipeline(
        detector=FakeDetector(),
        tracker=FakeTracker(),
    )

    output = pipeline.process_frame(frame)

    assert output.frame_id == 1
    assert output.image_width == 1280
    assert output.image_height == 720
    assert len(output.objects) == 1

    obj = output.objects[0]

    assert obj.class_name == "car"
    assert obj.class_id == 2
    assert obj.confidence == 0.95
    assert obj.bbox == [100, 100, 300, 250]


class FakeSensorManager:
    def get_latest_data(self, sensor_name):
        return {
            "frame": 1,
            "timestamp": 123.0,
            "data": np.zeros(
                (720, 1280, 3),
                dtype=np.uint8,
            ),
        }


class FakePipeline:
    def process_frame(self, frame):
        return "perception_output"


def test_carla_perception():
    sensor_manager = FakeSensorManager()
    pipeline = FakePipeline()

    perception = CarlaPerception(
        sensor_manager,
        pipeline,
    )

    output = perception.process_latest_frame()

    assert output == "perception_output"

from perception.hazard_detector import HazardDetector


def test_hazard_detector():
    detector = HazardDetector()

    detections = [
        {
            "class_name": "pothole",
            "confidence": 0.91,
            "bbox": [100, 200, 300, 350],
        },
        {
            "class_name": "car",
            "confidence": 0.95,
            "bbox": [400, 200, 600, 400],
        },
    ]

    hazards = detector.detect(detections)

    assert len(hazards) == 1
    assert hazards[0].hazard_type == "pothole"
    assert hazards[0].confidence == 0.91
    assert hazards[0].bbox == [100, 200, 300, 350]

from perception.drivable_space import DrivableSpaceDetector


def test_drivable_space_detector():
    detector = DrivableSpaceDetector()

    frame = np.zeros(
        (720, 1280, 3),
        dtype=np.uint8,
    )

    mask = detector.detect(frame)

    assert mask is not None
    assert mask.shape == (720, 1280)
    assert mask.dtype == np.uint8
    assert np.all(mask[:396] == 0)
    assert np.all(mask[396:] == 255)


def test_road_edges():
    detector = DrivableSpaceDetector()

    frame = np.zeros(
        (720, 1280, 3),
        dtype=np.uint8,
    )

    mask = detector.detect(frame)
    edges = detector.get_road_edges(mask)

    assert len(edges) > 0
    assert edges[0]["left"] == 0
    assert edges[0]["right"] == 1279

from perception.sensor_fusion import SensorFusion


def test_sensor_fusion():

    fusion = SensorFusion()

    output = fusion.fuse(
        objects=["car"],
        hazards=["pothole"],
        lidar_obstacles=["obstacle"],
        road_edges=["edge"],
        drivable_mask="mask",
    )

    assert output["objects"] == ["car"]
    assert output["hazards"] == ["pothole"]
    assert output["lidar_obstacles"] == ["obstacle"]
    assert output["road_edges"] == ["edge"]
    assert output["drivable_mask"] == "mask"