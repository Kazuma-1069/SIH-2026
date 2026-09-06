# -*- coding: utf-8 -*-
"""
SIH-2026 Traffic Light Decision Logic Tests

Tests offline traffic-light state decision logic:
- RED -> STOP, target_speed_mps = 0.0, M5 brake = 1.0
- YELLOW -> SLOW using planner's reduced speed, M5 brake = 0.0
- GREEN -> continue through existing safety checks
- Missing, unknown, or invalid state -> safe STOP, target_speed_mps = 0.0
- A GREEN light must NOT override Bubble Shield, blocked path, or risk assessment STOPs.
- Preserves existing planning behavior when no traffic light is present.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
from planning.planner import Planner
from simulation.controller import VehicleController
from integration.data_adapter import perception_to_planning_input
from interfaces.perception_output import PerceptionOutput, PerceptionObject


def _base_planning_input(ego=(0, 0), goal=(5, 5)):
    return {
        "primary_objects": [],
        "fallback_anomalies": [],
        "drivable_space": {},
        "confidence_uncertainty": {},
        "ego_position": list(ego),
        "goal": list(goal),
        "current_path": [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5]],
    }


# ============================================================================
# 1. RED Traffic Light -> STOP
# ============================================================================

def test_traffic_light_red_stops_vehicle():
    planner = Planner(width=10, height=10)
    data = _base_planning_input()
    data["traffic_light"] = "RED"

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["path_safe"] is False
    assert output["safety_reason"] == "TRAFFIC_LIGHT_RED"
    assert output["traffic_light_state"] == "RED"

    # M5 Controller integration
    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["throttle"] == 0.0
    assert control["brake"] == 1.0


def test_traffic_light_red_dict_format():
    planner = Planner(width=10, height=10)
    data = _base_planning_input()
    data["traffic_light"] = {"state": "red"}

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["traffic_light_state"] == "RED"


# ============================================================================
# 2. YELLOW Traffic Light -> SLOW (using reduced_speed_mps)
# ============================================================================

def test_traffic_light_yellow_slows_vehicle():
    planner = Planner(width=10, height=10)
    data = _base_planning_input()
    data["traffic_light"] = "YELLOW"

    output = planner.plan(data)

    assert output["action"] == "SLOW"
    assert output["target_speed_mps"] == planner.reduced_speed_mps
    assert output["path_safe"] is True
    assert output["traffic_light_state"] == "YELLOW"

    # M5 Controller integration
    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=45.0)
    assert control["brake"] == 0.0
    assert control["throttle"] > 0.0


def test_traffic_light_amber_alias():
    planner = Planner(width=10, height=10)
    data = _base_planning_input()
    data["traffic_light_state"] = "amber"

    output = planner.plan(data)

    assert output["action"] == "SLOW"
    assert output["target_speed_mps"] == planner.reduced_speed_mps
    assert output["traffic_light_state"] == "YELLOW"


# ============================================================================
# 3. GREEN Traffic Light -> GO (PROCEED_FORWARD if clear)
# ============================================================================

def test_traffic_light_green_proceeds_when_safe():
    planner = Planner(width=10, height=10)
    data = _base_planning_input()
    data["traffic_light"] = "GREEN"

    output = planner.plan(data)

    assert output["action"] == "PROCEED_FORWARD"
    assert output["target_speed_mps"] == planner.default_speed_mps
    assert output["path_safe"] is True
    assert output["traffic_light_state"] == "GREEN"

    # M5 Controller integration
    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=45.0)
    assert control["brake"] == 0.0
    assert control["throttle"] > 0.0


# ============================================================================
# 4. GREEN Light must NOT override existing safety STOP decisions
# ============================================================================

def test_green_light_does_not_override_bubble_shield_emergency():
    planner = Planner(width=10, height=10)
    data = _base_planning_input(ego=(0, 0))
    # Hazard right on ego vehicle -> distance = 0 <= emergency_radius (1.0)
    data["primary_objects"] = [{"position": [0.0, 0.0], "radius": 0.5, "vehicle_relative": True}]
    data["traffic_light"] = "GREEN"

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["algorithm"] == "BUBBLE_SHIELD"
    assert output["safety_reason"] == "BUBBLE_SHIELD_EMERGENCY"

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0


def test_green_light_does_not_override_blocked_path_stop():
    planner = Planner(width=10, height=10)
    data = _base_planning_input(ego=(0, 0))
    # Obstacle blocking active route at [2, 2]
    data["primary_objects"] = [{"position": [2.0, 2.0], "radius": 1.0, "vehicle_relative": False}]
    data["traffic_light"] = "GREEN"

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["current_path_blocked"] is True

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0


def test_green_light_does_not_override_predicted_path_conflict():
    planner = Planner(width=10, height=10)
    data = _base_planning_input(ego=(0, 0))
    data["risk_assessments"] = [{"path_conflict": True, "risk_level": "critical"}]
    data["traffic_light"] = "GREEN"

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["safety_reason"] == "PREDICTED_PATH_CONFLICT"

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0


# ============================================================================
# 5. Unknown, Invalid, or Missing Traffic-Light State -> safe STOP
# ============================================================================

def test_traffic_light_unknown_state_stops():
    planner = Planner(width=10, height=10)
    data = _base_planning_input()
    data["traffic_light"] = "UNKNOWN"

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["path_safe"] is False
    assert output["safety_reason"] == "TRAFFIC_LIGHT_UNKNOWN"

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0


def test_traffic_light_invalid_state_stops():
    planner = Planner(width=10, height=10)
    data = _base_planning_input()
    data["traffic_light"] = "BLUE"

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["path_safe"] is False
    assert output["safety_reason"] == "TRAFFIC_LIGHT_INVALID"

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0


def test_traffic_light_missing_state_stops():
    planner = Planner(width=10, height=10)

    # Case A: traffic_light key is None
    data_a = _base_planning_input()
    data_a["traffic_light"] = None
    output_a = planner.plan(data_a)
    assert output_a["action"] == "STOP"
    assert output_a["target_speed_mps"] == 0.0
    assert output_a["safety_reason"] == "TRAFFIC_LIGHT_MISSING"

    # Case B: traffic_light dict has no state
    data_b = _base_planning_input()
    data_b["traffic_light"] = {}
    output_b = planner.plan(data_b)
    assert output_b["action"] == "STOP"
    assert output_b["target_speed_mps"] == 0.0
    assert output_b["safety_reason"] == "TRAFFIC_LIGHT_MISSING"

    # Case C: traffic_light_state is empty string
    data_c = _base_planning_input()
    data_c["traffic_light_state"] = ""
    output_c = planner.plan(data_c)
    assert output_c["action"] == "STOP"
    assert output_c["target_speed_mps"] == 0.0
    assert output_c["safety_reason"] == "TRAFFIC_LIGHT_MISSING"


# ============================================================================
# 6. Data Adapter Traffic-Light Forwarding & Baseline Preservation
# ============================================================================

def test_data_adapter_forwards_environment_traffic_light():
    perception = PerceptionOutput(
        frame_id=1,
        image_width=1280,
        image_height=720,
        environment={"traffic_light": "RED"},
    )

    planning_input = perception_to_planning_input(perception)

    assert "traffic_light" in planning_input
    assert planning_input["traffic_light"] == "RED"


def test_data_adapter_forwards_traffic_light_object():
    obj = PerceptionObject(
        track_id=1,
        class_id=9,
        class_name="traffic light",
        confidence=0.92,
        bbox=[600, 100, 680, 250],
    )
    obj.state = "YELLOW"

    perception = PerceptionOutput(
        frame_id=1,
        image_width=1280,
        image_height=720,
        objects=[obj],
    )

    planning_input = perception_to_planning_input(perception)

    assert "traffic_light" in planning_input
    assert planning_input["traffic_light"] == "YELLOW"


def test_baseline_preserved_when_no_traffic_light_present():
    perception = PerceptionOutput(
        frame_id=1,
        image_width=1280,
        image_height=720,
        objects=[],
    )

    planning_input = perception_to_planning_input(perception)
    assert "traffic_light" not in planning_input

    planner = Planner(width=10, height=10)
    planning_input.update(_base_planning_input())
    output = planner.plan(planning_input)

    assert output["action"] == "PROCEED_FORWARD"
    assert output["target_speed_mps"] == planner.default_speed_mps
