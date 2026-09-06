# -*- coding: utf-8 -*-
"""
Tests for unknown-object handling in perception and tracking.

Requirements:
- Detection threshold: 0.40
- Classification threshold: 0.60
- Detections with confidence >= 0.60 keep their class
- Detections with confidence < 0.60 become class_name="unknown", class_id=-1
- Unknown detection is not discarded throughout perception -> tracking -> adapter -> planning
- Unknown track (class_id == -1) can associate across frames using centroid distance
- Known-class tracking behavior is strictly preserved (no cross-matching between distinct known classes)
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest
from perception.object_tracker import ObjectTracker
from perception.perception_pipeline import PerceptionPipeline
from perception.yolo_detector import YOLODetector
from integration.data_adapter import perception_to_planning_input
from planning.obstacle_map import ObstacleMap


# ============================================================================
# 1. YOLODetector classification threshold tests
# ============================================================================

def _make_mock_box(xyxy, conf, cls_id):
    box = MagicMock()
    # xyxy tensor mock
    t_xyxy = MagicMock()
    t_xyxy.cpu().numpy.return_value = np.array(xyxy, dtype=float)
    box.xyxy = [t_xyxy]

    # conf tensor mock
    t_conf = MagicMock()
    t_conf.cpu().item.return_value = float(conf)
    box.conf = [t_conf]

    # cls tensor mock
    t_cls = MagicMock()
    t_cls.cpu().item.return_value = float(cls_id)
    box.cls = [t_cls]

    return box


def _make_mock_yolo_detector(conf_thresh=0.40, class_thresh=0.60):
    detector = object.__new__(YOLODetector)
    detector.confidence_threshold = conf_thresh
    detector.classification_threshold = class_thresh
    detector.device = None
    detector.class_names = {0: "person", 2: "car", 3: "motorcycle"}
    detector.model = MagicMock()
    return detector


def test_low_confidence_detection_becomes_unknown():
    detector = _make_mock_yolo_detector(conf_thresh=0.40, class_thresh=0.60)

    # Box with confidence 0.52 (>= 0.40 detection threshold, but < 0.60 classification threshold)
    mock_box = _make_mock_box([100, 100, 200, 200], conf=0.52, cls_id=2)
    mock_result = MagicMock()
    mock_result.boxes = [mock_box]
    detector.model.predict.return_value = [mock_result]

    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    detections = detector.detect(frame)

    assert len(detections) == 1
    det = detections[0]
    assert det["class_id"] == -1
    assert det["class_name"] == "unknown"
    assert det["confidence"] == 0.52
    assert det["bbox"] == [100, 100, 200, 200]


def test_high_confidence_detection_keeps_its_class():
    detector = _make_mock_yolo_detector(conf_thresh=0.40, class_thresh=0.60)

    # Box with confidence 0.85 (>= 0.60 classification threshold)
    mock_box = _make_mock_box([50, 50, 150, 150], conf=0.85, cls_id=2)
    mock_result = MagicMock()
    mock_result.boxes = [mock_box]
    detector.model.predict.return_value = [mock_result]

    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    detections = detector.detect(frame)

    assert len(detections) == 1
    det = detections[0]
    assert det["class_id"] == 2
    assert det["class_name"] == "car"
    assert det["confidence"] == 0.85


def test_exact_threshold_detection_keeps_its_class():
    detector = _make_mock_yolo_detector(conf_thresh=0.40, class_thresh=0.60)

    # Box with exact confidence 0.60
    mock_box = _make_mock_box([50, 50, 150, 150], conf=0.60, cls_id=0)
    mock_result = MagicMock()
    mock_result.boxes = [mock_box]
    detector.model.predict.return_value = [mock_result]

    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    detections = detector.detect(frame)

    assert len(detections) == 1
    det = detections[0]
    assert det["class_id"] == 0
    assert det["class_name"] == "person"
    assert det["confidence"] == 0.60


# ============================================================================
# 2. Tracking tests: unknown track association & known class isolation
# ============================================================================

def test_unknown_track_continues_across_frames():
    tracker = ObjectTracker(max_distance=50.0)

    # Frame 1: unknown detection
    frame1_detections = [
        {
            "class_id": -1,
            "class_name": "unknown",
            "confidence": 0.50,
            "bbox": [100, 100, 200, 200],
        }
    ]
    tracks1 = tracker.update(frame1_detections)
    assert len(tracks1) == 1
    track_id = tracks1[0]["track_id"]
    assert tracks1[0]["class_id"] == -1
    assert tracks1[0]["class_name"] == "unknown"
    assert tracks1[0]["age"] == 1

    # Frame 2: unknown detection slightly moved
    frame2_detections = [
        {
            "class_id": -1,
            "class_name": "unknown",
            "confidence": 0.53,
            "bbox": [105, 105, 205, 205],
        }
    ]
    tracks2 = tracker.update(frame2_detections)
    assert len(tracks2) == 1
    assert tracks2[0]["track_id"] == track_id  # Same track!
    assert tracks2[0]["class_id"] == -1
    assert tracks2[0]["class_name"] == "unknown"
    assert tracks2[0]["age"] == 2


def test_unknown_track_associates_with_nearby_detection():
    tracker = ObjectTracker(max_distance=50.0)

    # Frame 1: track established as unknown
    frame1 = [
        {"class_id": -1, "class_name": "unknown", "confidence": 0.48, "bbox": [100, 100, 150, 150]}
    ]
    tracks1 = tracker.update(frame1)
    orig_id = tracks1[0]["track_id"]

    # Frame 2: detection at centroid (126, 126) - nearby
    frame2 = [
        {"class_id": -1, "class_name": "unknown", "confidence": 0.55, "bbox": [102, 102, 152, 152]}
    ]
    tracks2 = tracker.update(frame2)
    assert len(tracks2) == 1
    assert tracks2[0]["track_id"] == orig_id
    assert tracks2[0]["age"] == 2


def test_existing_known_class_tracking_still_works():
    tracker = ObjectTracker(max_distance=50.0)

    # Frame 1: two distinct known objects
    frame1 = [
        {"class_id": 2, "class_name": "car", "confidence": 0.90, "bbox": [100, 100, 200, 200]},
        {"class_id": 0, "class_name": "person", "confidence": 0.85, "bbox": [300, 100, 350, 200]},
    ]
    tracks1 = tracker.update(frame1)
    assert len(tracks1) == 2
    car_id = [t["track_id"] for t in tracks1 if t["class_name"] == "car"][0]
    person_id = [t["track_id"] for t in tracks1 if t["class_name"] == "person"][0]

    # Frame 2: both move slightly, keeping same classes
    frame2 = [
        {"class_id": 2, "class_name": "car", "confidence": 0.92, "bbox": [105, 100, 205, 200]},
        {"class_id": 0, "class_name": "person", "confidence": 0.87, "bbox": [302, 100, 352, 200]},
    ]
    tracks2 = tracker.update(frame2)
    assert len(tracks2) == 2
    car_track = [t for t in tracks2 if t["track_id"] == car_id][0]
    person_track = [t for t in tracks2 if t["track_id"] == person_id][0]
    assert car_track["class_name"] == "car"
    assert person_track["class_name"] == "person"

    # Verify that different known classes nearby DO NOT cross-match:
    # A pedestrian detection placed right at the car position should NOT match the car track
    tracker_cross = ObjectTracker(max_distance=50.0)
    tracker_cross.update([
        {"class_id": 2, "class_name": "car", "confidence": 0.90, "bbox": [100, 100, 200, 200]}
    ])
    # Next frame has a pedestrian at the exact same location
    cross_tracks = tracker_cross.update([
        {"class_id": 0, "class_name": "person", "confidence": 0.85, "bbox": [100, 100, 200, 200]}
    ])
    # Must spawn a new track for person, NOT reuse the car track
    assert len(cross_tracks) == 2
    person_only = [t for t in cross_tracks if t["class_name"] == "person"][0]
    assert person_only["track_id"] != 1


# ============================================================================
# 3. Pipeline & downstream flow: unknown detection is not discarded
# ============================================================================

def test_unknown_detection_not_discarded_in_pipeline():
    class DummyUnknownDetector:
        def detect(self, frame):
            return [
                {
                    "class_id": -1,
                    "class_name": "unknown",
                    "confidence": 0.55,
                    "bbox": [500, 200, 700, 400],
                }
            ]

    pipeline = PerceptionPipeline(
        detector=DummyUnknownDetector(),
        tracker=ObjectTracker(),
    )

    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    output = pipeline.process_frame(frame)

    assert len(output.objects) == 1
    obj = output.objects[0]
    assert obj.class_id == -1
    assert obj.class_name == "unknown"
    assert obj.confidence == 0.55
    assert obj.bbox == [500, 200, 700, 400]

    # Downstream data_adapter verification
    planning_input = perception_to_planning_input(output)
    assert len(planning_input["primary_objects"]) == 1
    p_obj = planning_input["primary_objects"][0]
    assert p_obj["class_name"] == "unknown"
    assert p_obj["class_id"] == -1
    assert p_obj["vehicle_relative"] is True
    assert p_obj["position"] is not None

    # Downstream ObstacleMap verification: obstacle is successfully added
    obs_map = ObstacleMap(width=20, height=20)
    obs_map.update_from_objects(planning_input["primary_objects"])
    assert len(obs_map.obstacles) >= 1
    assert obs_map.obstacles[0]["type"] == "unknown"
