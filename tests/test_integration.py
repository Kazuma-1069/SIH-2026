from dataclasses import dataclass

from integration.data_adapter import perception_to_planning_input
from integration.pipeline import IntegrationPipeline
from interfaces.perception_output import (
    PerceptionOutput,
    PerceptionObject,
)


def test_perception_to_planning_input():
    perception = PerceptionOutput(
        frame_id=1,
        image_width=1280,
        image_height=720,
        objects=[
            PerceptionObject(
                track_id=1,
                class_id=2,
                class_name="car",
                confidence=0.95,
                bbox=[100, 100, 200, 200],
                distance=10.0,
            )
        ],
    )

    planning_input = perception_to_planning_input(perception)

    assert "primary_objects" in planning_input
    assert "fallback_anomalies" in planning_input
    assert "drivable_space" in planning_input
    assert "confidence_uncertainty" in planning_input
    assert "start" in planning_input
    assert "goal" in planning_input

    assert len(planning_input["primary_objects"]) == 1

    obj = planning_input["primary_objects"][0]

    assert obj["track_id"] == 1
    assert obj["class_name"] == "car"
    assert obj["bbox"] == [100, 100, 200, 200]
    assert obj["distance"] == 10.0

    assert planning_input["start"] == [0, 0]
    assert planning_input["goal"] == [10, 10]


class FakePerceptionPipeline:
    """Minimal M2 substitute for integration testing."""

    def process_frame(self, frame):
        return PerceptionOutput(
            frame_id=1,
            image_width=1280,
            image_height=720,
            objects=[],
        )


class FakePlanner:
    """Minimal M1 substitute for integration testing."""

    def plan(self, planning_input):
        assert planning_input["start"] == [0, 0]
        assert planning_input["goal"] == [10, 10]

        return {
            "action": "PROCEED_FORWARD",
            "target_speed_mps": 5.0,
            "algorithm": "A_STAR",
            "hazard_count": 0,
            "waypoints": [[0, 0], [10, 10]],
            "path_safe": True,
            "safety_reason": "PATH_CLEAR",
            "confidence_uncertainty": {},
        }


def test_integration_pipeline_m2_to_m1():
    pipeline = IntegrationPipeline(
        perception_pipeline=FakePerceptionPipeline(),
        planner=FakePlanner(),
    )

    frame = object()

    perception_output, planning_output = pipeline.process_frame(
        frame
    )

    assert isinstance(perception_output, PerceptionOutput)

    assert planning_output["action"] == "PROCEED_FORWARD"
    assert planning_output["algorithm"] == "A_STAR"
    assert planning_output["path_safe"] is True
    assert planning_output["waypoints"] == [
        [0, 0],
        [10, 10],
    ]