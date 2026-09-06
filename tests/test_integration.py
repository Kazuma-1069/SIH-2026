import sys
import os

sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            ".."
        )
    )
)
from simulation.controller import VehicleController
from integration.data_adapter import (
    perception_to_planning_input,
    pixel_to_vehicle_coords,
)
from interfaces.perception_output import (
    PerceptionOutput,
    PerceptionObject,
    RoadHazard,
)
from planning.obstacle_map import ObstacleMap
from planning.safety_checker import BubbleShield
from integration.pipeline import IntegrationPipeline


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
    controller=VehicleController(),
)

    frame = object()

    perception_output, planning_output, control_command = pipeline.process_frame(
    frame
)
    assert isinstance(perception_output, PerceptionOutput)
    assert control_command is not None
    assert control_command["throttle"] > 0
    assert control_command["brake"] == 0.0
    assert planning_output["action"] == "PROCEED_FORWARD"
    assert planning_output["algorithm"] == "A_STAR"
    assert planning_output["path_safe"] is True
    assert planning_output["waypoints"] == [
        [0, 0],
        [10, 10],
    ]


def test_pixel_to_vehicle_coords_centered():
    bbox = [540, 200, 740, 400]
    coords = pixel_to_vehicle_coords(
        bbox=bbox,
        distance=10.0,
        image_width=1280,
        image_height=720,
    )
    assert coords[0] == 10.0
    assert coords[1] == 0.0


def test_pixel_to_vehicle_coords_lateral_offsets():
    # Right-side object: u_center = 960 (640 + 320)
    bbox_right = [860, 200, 1060, 400]
    coords_right = pixel_to_vehicle_coords(
        bbox=bbox_right,
        distance=10.0,
        image_width=1280,
        image_height=720,
    )
    assert coords_right[0] == 10.0
    assert coords_right[1] == 5.0

    # Left-side object: u_center = 320 (640 - 320)
    bbox_left = [220, 200, 420, 400]
    coords_left = pixel_to_vehicle_coords(
        bbox=bbox_left,
        distance=10.0,
        image_width=1280,
        image_height=720,
    )
    assert coords_left[0] == 10.0
    assert coords_left[1] == -5.0


def test_perception_to_planning_input_converts_coordinates():
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
                bbox=[540, 200, 740, 400],
                distance=12.0,
            )
        ],
        hazards=[
            RoadHazard(
                hazard_type="pothole",
                confidence=0.88,
                bbox=[220, 500, 420, 600],
                distance=6.0,
            )
        ],
    )

    planning_input = perception_to_planning_input(perception)

    assert len(planning_input["primary_objects"]) == 1
    car_obj = planning_input["primary_objects"][0]
    assert car_obj["vehicle_relative"] is True
    assert car_obj["position"] == [12.0, 0.0]
    assert car_obj["radius"] > 0

    assert len(planning_input["fallback_anomalies"]) == 1
    hazard_obj = planning_input["fallback_anomalies"][0]
    assert hazard_obj["vehicle_relative"] is True
    assert hazard_obj["position"][0] == 6.0
    assert hazard_obj["position"][1] < 0.0


def test_bubble_shield_with_vehicle_relative_obstacles():
    obstacle_map = ObstacleMap(width=20, height=20)
    bubble_shield = BubbleShield(
        obstacle_map,
        radius=2.0,
        emergency_radius=1.0,
    )

    # Distant obstacle (15m ahead) -> should be safe
    obstacle_map.update_from_objects(
        [
            {
                "position": [15.0, 0.0],
                "radius": 1.0,
                "vehicle_relative": True,
            }
        ]
    )
    result_far = bubble_shield.check([0.0, 0.0])
    assert result_far["safe"] is True
    assert result_far["emergency"] is False

    # Close obstacle (1.5m ahead, radius 1.0 -> boundary at 0.5m) -> emergency
    obstacle_map.update_from_objects(
        [
            {
                "position": [1.5, 0.0],
                "radius": 1.0,
                "vehicle_relative": True,
            }
        ]
    )
    result_near = bubble_shield.check([0.0, 0.0])
    assert result_near["safe"] is False
    assert result_near["emergency"] is True
    assert result_near["reason"] == "EMERGENCY_BUBBLE_VIOLATION"