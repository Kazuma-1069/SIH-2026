import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
from planning.planner import Planner
from planning.obstacle_map import ObstacleMap
from interfaces.perception_output import PerceptionOutput, PerceptionObject, RoadHazard
from integration.data_adapter import perception_to_planning_input


def test_multiple_hazards_coexist_in_representation():
    planner = Planner(width=30, height=30)

    # 5 different hazard types coexisting
    primary = [
        {"position": [5.0, 5.0], "radius": 1.5, "class_name": "car", "class_id": 2},
        {"position": [8.0, 12.0], "radius": 0.8, "class_name": "pedestrian", "class_id": 0},
        {"position": [12.0, 5.0], "radius": 0.8, "class_name": "bicycle", "class_id": 1},
        {"position": [15.0, 15.0], "radius": 1.0, "class_name": "unknown", "class_id": -1},
    ]
    fallback = [
        {"position": [10.0, 10.0], "radius": 0.5, "class_name": "pothole"},
    ]

    output = planner.plan(
        {
            "primary_objects": primary,
            "fallback_anomalies": fallback,
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 10],
            "goal": [25, 10],
            "current_path": [[x, 10] for x in range(26)],
            "allow_local_detour": True,
        }
    )

    assert output["hazard_count"] == 5
    assert len(output["hazards"]) == 5

    types = {h.get("class_name") or h.get("type") for h in output["hazards"]}
    assert types == {"car", "pedestrian", "bicycle", "unknown", "pothole"}

    # Verify obstacle_map records all 5
    obs_types = {obs["type"] for obs in planner.obstacle_map.obstacles}
    assert obs_types == {"car", "pedestrian", "bicycle", "unknown", "pothole"}


def test_planning_navigates_around_multiple_coexisting_hazards():
    planner = Planner(width=25, height=25)
    start = [0, 10]
    goal = [20, 10]
    straight_path = [[x, 10] for x in range(21)]

    # Scattered hazards: car, pedestrian, bike, unknown, pothole
    hazards = [
        {"position": [5.0, 10.0], "radius": 0.5, "class_name": "car"},
        {"position": [10.0, 12.0], "radius": 0.5, "class_name": "pedestrian"},
        {"position": [10.0, 8.0], "radius": 0.5, "class_name": "bicycle"},
        {"position": [15.0, 10.0], "radius": 0.5, "class_name": "unknown"},
        {"position": [15.0, 7.0], "radius": 0.5, "class_name": "pothole"},
    ]

    output = planner.plan(
        {
            "primary_objects": hazards[:4],
            "fallback_anomalies": hazards[4:],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": start,
            "goal": goal,
            "current_path": straight_path,
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "REROUTE"
    assert output["replanned"] is True
    assert output["path_safe"] is True
    assert output["destination"] == goal
    assert len(output["waypoints"]) > 0
    assert list(output["waypoints"][-1]) == goal

    # Verify waypoints avoid all hazard positions
    hazard_coords = {(int(h["position"][0]), int(h["position"][1])) for h in hazards}
    for wp in output["waypoints"]:
        assert (int(wp[0]), int(wp[1])) not in hazard_coords


def test_planning_considers_all_hazards_not_only_first():
    planner = Planner(width=20, height=20)
    start = [0, 5]
    goal = [15, 5]

    # Hazard 1 blocks direct path at (5, 5).
    # Hazard 2 blocks northern detour at (5, 6), (5, 7), (5, 8).
    # Planner MUST detour south (y < 5) to avoid both hazards.
    hazards = [
        {"position": [5.0, 5.0], "radius": 0.5, "class_name": "car"},
        {"position": [5.0, 6.0], "radius": 0.5, "class_name": "pedestrian"},
        {"position": [5.0, 7.0], "radius": 0.5, "class_name": "bicycle"},
        {"position": [5.0, 8.0], "radius": 0.5, "class_name": "pothole"},
    ]

    output = planner.plan(
        {
            "primary_objects": hazards,
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": start,
            "goal": goal,
            "current_path": [[x, 5] for x in range(16)],
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "REROUTE"
    assert output["path_safe"] is True
    assert len(output["waypoints"]) > 0

    # Every waypoint at x=5 must detour south (y < 5) because northern route is blocked
    for wp in output["waypoints"]:
        if int(wp[0]) == 5:
            assert int(wp[1]) < 5


def test_multiple_hazards_completely_blocking_corridor_triggers_stop():
    planner = Planner(width=10, height=6)
    start = [0, 3]
    goal = [9, 3]

    # Full barricade across x=5 from y=0 to y=5 using mixed hazard classes
    blockade = [
        {"position": [5.0, 0.0], "radius": 0.0, "class_name": "car"},
        {"position": [5.0, 1.0], "radius": 0.0, "class_name": "pedestrian"},
        {"position": [5.0, 2.0], "radius": 0.0, "class_name": "bicycle"},
        {"position": [5.0, 3.0], "radius": 0.0, "class_name": "unknown"},
        {"position": [5.0, 4.0], "radius": 0.0, "class_name": "pothole"},
        {"position": [5.0, 5.0], "radius": 0.0, "class_name": "road_hazard"},
    ]

    output = planner.plan(
        {
            "primary_objects": blockade[:4],
            "fallback_anomalies": blockade[4:],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": start,
            "goal": goal,
            "current_path": [[x, 3] for x in range(10)],
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["waypoints"] == []
    assert output["path_safe"] is False
    assert output["safety_reason"] == "NO_PATH_FOUND"


def test_multiple_hazards_tight_gap_violates_bubble_shield_triggers_stop():
    # Width 10, Height 5.
    # Obstacles at y=0,1 and y=3,4 with a 1-cell gap at y=2.
    # An obstacle is also at [5, 2] with radius 0.
    # Adjacent obstacles have radius 1.5, which overlap the gap with bubble radius 2.0.
    planner = Planner(width=10, height=5)
    start = [0, 2]
    goal = [9, 2]

    hazards = [
        {"position": [5.0, 2.0], "radius": 0.0, "class_name": "pothole"},
        {"position": [5.0, 0.0], "radius": 1.0, "class_name": "car"},
        {"position": [5.0, 4.0], "radius": 1.0, "class_name": "truck"},
    ]

    output = planner.plan(
        {
            "primary_objects": hazards,
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": start,
            "goal": goal,
            "current_path": [[x, 2] for x in range(10)],
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["waypoints"] == []
    assert output["path_safe"] is False
    assert output["safety_reason"] in {"BUBBLE_SHIELD_PATH_BLOCKED", "NO_PATH_FOUND"}


def test_bubble_shield_emergency_priority_with_multiple_hazards():
    planner = Planner(width=20, height=20)

    # 4 hazards in scene; hazard 1 is at ego's location (emergency distance <= 1.0)
    hazards = [
        {"position": [0.5, 0.0], "radius": 0.0, "class_name": "pedestrian", "vehicle_relative": True},
        {"position": [5.0, 5.0], "radius": 1.0, "class_name": "car"},
        {"position": [8.0, 3.0], "radius": 0.5, "class_name": "bicycle"},
        {"position": [10.0, 0.0], "radius": 0.5, "class_name": "pothole"},
    ]

    output = planner.plan(
        {
            "primary_objects": hazards,
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 0],
            "goal": [15, 0],
            "current_path": [[x, 0] for x in range(16)],
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["safety_reason"] == "BUBBLE_SHIELD_EMERGENCY"
    assert output["bubble_emergency"] is True


def test_traffic_light_red_priority_with_multiple_hazards():
    planner = Planner(width=20, height=20)

    hazards = [
        {"position": [5.0, 5.0], "radius": 0.5, "class_name": "car"},
        {"position": [8.0, 5.0], "radius": 0.5, "class_name": "unknown"},
        {"position": [10.0, 5.0], "radius": 0.5, "class_name": "pothole"},
    ]

    output = planner.plan(
        {
            "primary_objects": hazards,
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 5],
            "goal": [15, 5],
            "current_path": [[x, 5] for x in range(16)],
            "traffic_light_state": "RED",
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["safety_reason"] == "TRAFFIC_LIGHT_RED"
    assert output["hazard_count"] == 3


def test_dynamic_trajectory_risk_priority_with_multiple_hazards():
    planner = Planner(width=20, height=20)

    hazards = [
        {"position": [12.0, 8.0], "radius": 0.5, "class_name": "pothole"},
        {"position": [15.0, 2.0], "radius": 0.5, "class_name": "unknown"},
    ]

    # Dynamic risk assessment on oncoming car with critical conflict
    risk = [
        {
            "track_id": 1,
            "class_name": "car",
            "path_conflict": True,
            "risk_level": "critical",
            "distance_m": 4.0,
        }
    ]

    output = planner.plan(
        {
            "primary_objects": hazards,
            "fallback_anomalies": [],
            "risk_assessments": risk,
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 5],
            "goal": [15, 5],
            "current_path": [[x, 5] for x in range(16)],
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["safety_reason"] == "PREDICTED_PATH_CONFLICT"
    assert output["algorithm"] == "RISK_ASSESSMENT"


def test_data_adapter_combines_objects_and_hazards_seamlessly():
    perception_out = PerceptionOutput(
        frame_id=1,
        image_width=1280,
        image_height=720,
        objects=[
            PerceptionObject(track_id=1, class_id=2, class_name="car", confidence=0.92, bbox=[100, 200, 300, 400], distance=15.0),
            PerceptionObject(track_id=2, class_id=0, class_name="pedestrian", confidence=0.85, bbox=[400, 200, 480, 400], distance=10.0),
            PerceptionObject(track_id=3, class_id=1, class_name="bicycle", confidence=0.78, bbox=[600, 250, 680, 380], distance=8.0),
            PerceptionObject(track_id=4, class_id=-1, class_name="unknown", confidence=0.52, bbox=[800, 200, 900, 350], distance=20.0),
        ],
        hazards=[
            RoadHazard(hazard_type="pothole", confidence=0.90, bbox=[500, 500, 600, 600], distance=6.0, radius=0.6),
        ],
    )

    planning_in = perception_to_planning_input(perception_out)

    assert len(planning_in["primary_objects"]) == 4
    assert len(planning_in["fallback_anomalies"]) == 1

    primary_classes = {o["class_name"] for o in planning_in["primary_objects"]}
    assert primary_classes == {"car", "pedestrian", "bicycle", "unknown"}

    assert planning_in["fallback_anomalies"][0]["class_name"] == "pothole"

    planner = Planner(width=20, height=20)
    output = planner.plan(planning_in)

    assert output["hazard_count"] == 5
    assert len(output["hazards"]) == 5
