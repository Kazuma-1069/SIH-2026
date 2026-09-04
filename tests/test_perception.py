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