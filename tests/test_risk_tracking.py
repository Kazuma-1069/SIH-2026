import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration.data_adapter import perception_to_planning_input
from interfaces.perception_output import PerceptionOutput
from perception.object_tracker import ObjectTracker
from perception.perception_pipeline import PerceptionPipeline
from planning.planner import Planner


class SequenceDetector:
    def __init__(self):
        self.frames = [
            [
                {
                    "track_id": 99,
                    "class_id": 2,
                    "class_name": "car",
                    "confidence": 0.95,
                    "bbox": [600, 200, 680, 400],
                }
            ],
            [
                {
                    "track_id": 99,
                    "class_id": 2,
                    "class_name": "car",
                    "confidence": 0.94,
                    "bbox": [600, 210, 680, 410],
                }
            ],
        ]
        self.index = 0

    def detect(self, _frame):
        detections = self.frames[min(self.index, len(self.frames) - 1)]
        self.index += 1
        return detections


class FixedDepth:
    def add_distance(self, objects):
        output = []
        for obj in objects:
            item = dict(obj)
            item["distance"] = 10.0
            output.append(item)
        return output


def test_tracker_preserves_identity_and_estimates_velocity():
    tracker = ObjectTracker(max_distance=100.0)
    first = tracker.update(
        [
            {
                "class_id": 2,
                "class_name": "car",
                "confidence": 0.9,
                "bbox": [100, 100, 180, 200],
            }
        ]
    )
    second = tracker.update(
        [
            {
                "class_id": 2,
                "class_name": "car",
                "confidence": 0.9,
                "bbox": [110, 105, 190, 205],
            }
        ]
    )

    assert first[0]["track_id"] == second[0]["track_id"]
    assert second[0]["velocity"] == [10.0, 5.0]
    assert second[0]["age"] == 2


def test_perception_emits_predictions_and_risk_assessments():
    pipeline = PerceptionPipeline(
        detector=SequenceDetector(),
        tracker=ObjectTracker(),
        depth_estimator=FixedDepth(),
    )
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    pipeline.process_frame(frame)
    output = pipeline.process_frame(frame)

    assert output.predictions[0]["track_id"] == 1
    assert output.predictions[0]["horizon_frames"] == 5
    assert output.risk_assessments[0]["risk_level"] == "high"
    assert output.risk_assessments[0]["path_conflict"] is True

    planning_input = perception_to_planning_input(output)
    assert planning_input["risk_assessments"][0]["track_id"] == 1
    assert planning_input["primary_objects"][0]["velocity"] == [0.0, 10.0]


def test_planner_stops_on_predicted_high_risk_path_conflict():
    output = Planner(width=20, height=20).plan(
        {
            "primary_objects": [],
            "fallback_anomalies": [],
            "risk_assessments": [
                {
                    "track_id": 1,
                    "risk_level": "critical",
                    "path_conflict": True,
                }
            ],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 0],
            "goal": [10, 0],
            "current_path": [[0, 0], [1, 0], [2, 0]],
        }
    )

    assert output["action"] == "STOP"
    assert output["path_safe"] is False
    assert output["safety_reason"] == "PREDICTED_PATH_CONFLICT"


@pytest.mark.parametrize(
    "hazard_type",
    ["pedestrian", "bike_ahead", "sudden_stopping_car", "pothole"],
)
def test_m1_stops_for_high_risk_conflicts(hazard_type):
    output = Planner(width=20, height=20).plan(
        {
            "primary_objects": [],
            "fallback_anomalies": [
                {
                    "class_name": hazard_type,
                    "position": [4.0, 0.0],
                    "radius": 1.0,
                }
            ],
            "risk_assessments": [
                {
                    "hazard_type": hazard_type,
                    "risk_level": "critical",
                    "path_conflict": True,
                }
            ],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 0],
            "goal": [10, 0],
            "current_path": [[0, 0], [1, 0], [2, 0]],
        }
    )

    assert output["action"] == "STOP"
    assert output["path_safe"] is False
