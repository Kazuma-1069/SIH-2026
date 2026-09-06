# -*- coding: utf-8 -*-
"""
Tests for hardened Bubble Shield safety behavior.

Verifies:
1. If an obstacle is inside the configured safety bubble around the ego vehicle, the planner treats the path as unsafe (path_safe=False, action=STOP, target_speed_mps=0.0).
2. If an obstacle is inside the configured safety bubble along a current/planned path, the planner treats the path as unsafe.
3. Critical/emergency Bubble Shield situations (distance <= emergency_radius) produce immediate STOP with zero target speed.
4. A clear Bubble Shield does not interfere with normal planning (path_safe=True, action=PROCEED_FORWARD, default speed).
5. Bubble Shield STOP is NOT overridden by traffic-light GREEN logic.
6. M5 VehicleController applies full braking (brake=1.0, throttle=0.0) when Bubble Shield is violated.
7. Preserves existing blocked-path, trajectory-risk, and emergency stop behavior.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
from planning.planner import Planner
from simulation.controller import VehicleController
from planning.safety_checker import BubbleShield
from planning.obstacle_map import ObstacleMap


def _base_planning_input(ego=(0, 0), goal=(8, 0)):
    return {
        "primary_objects": [],
        "fallback_anomalies": [],
        "drivable_space": {},
        "confidence_uncertainty": {},
        "ego_position": list(ego),
        "goal": list(goal),
        "current_path": [[x, 0] for x in range(9)],
    }


def test_clear_bubble_shield_does_not_interfere_with_normal_planning():
    planner = Planner(width=20, height=20)

    # Case A: completely clear scene -> PROCEED_FORWARD
    data_clear = _base_planning_input(ego=(0, 0), goal=(8, 0))
    output_clear = planner.plan(data_clear)
    assert output_clear["path_safe"] is True
    assert output_clear["bubble_safe"] is True
    assert output_clear["bubble_emergency"] is False
    assert output_clear["bubble_path_safe"] is True
    assert output_clear["action"] == "PROCEED_FORWARD"
    assert output_clear["target_speed_mps"] == planner.default_speed_mps

    # Case B: distant obstacle far outside the 2.0m bubble (e.g. at x=15, y=15)
    data_distant = _base_planning_input(ego=(0, 0), goal=(8, 0))
    data_distant["primary_objects"] = [
        {"position": [15.0, 15.0], "radius": 0.5, "vehicle_relative": False}
    ]
    output_distant = planner.plan(data_distant)
    assert output_distant["path_safe"] is True
    assert output_distant["bubble_safe"] is True
    assert output_distant["bubble_emergency"] is False
    assert output_distant["bubble_path_safe"] is True
    # Bubble Shield does NOT force a stop; normal hazard handling proceeds
    assert output_distant["action"] in {"PROCEED_FORWARD", "SLOW_AND_REROUTE"}
    assert output_distant["target_speed_mps"] > 0.0


def test_obstacle_inside_configured_safety_bubble_treats_path_as_unsafe():
    planner = Planner(width=20, height=20)
    data = _base_planning_input(ego=(0, 0), goal=(8, 0))
    # Obstacle at lateral offset 1.5m (inside 2.0m radius, but > 1.0m emergency radius)
    data["primary_objects"] = [
        {"position": [0.0, 1.5], "radius": 0.0, "vehicle_relative": True}
    ]

    output = planner.plan(data)

    # Path must be treated as unsafe
    assert output["path_safe"] is False
    assert output["bubble_safe"] is False
    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["safety_reason"] == "BUBBLE_VIOLATION"

    # M5 controller verifies emergency brake
    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0
    assert control["throttle"] == 0.0


def test_emergency_bubble_shield_produces_stop_with_zero_speed():
    planner = Planner(width=20, height=20)
    data = _base_planning_input(ego=(0, 0), goal=(8, 0))
    # Obstacle right at 0.5m ahead (<= 1.0m emergency radius)
    data["primary_objects"] = [
        {"position": [0.5, 0.0], "radius": 0.0, "vehicle_relative": True}
    ]

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["algorithm"] == "BUBBLE_SHIELD"
    assert output["path_safe"] is False
    assert output["bubble_emergency"] is True
    assert output["safety_reason"] == "BUBBLE_SHIELD_EMERGENCY"
    assert output["waypoints"] == []

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0
    assert control["throttle"] == 0.0


def test_bubble_shield_stop_not_overridden_by_traffic_light_green():
    planner = Planner(width=20, height=20)
    data = _base_planning_input(ego=(0, 0), goal=(8, 0))
    # Obstacle inside emergency radius
    data["primary_objects"] = [
        {"position": [0.5, 0.0], "radius": 0.0, "vehicle_relative": True}
    ]
    # Explicit GREEN traffic light signal
    data["traffic_light"] = "GREEN"

    output = planner.plan(data)

    # Must still produce Bubble Shield emergency STOP
    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["algorithm"] == "BUBBLE_SHIELD"
    assert output["path_safe"] is False
    assert output["safety_reason"] == "BUBBLE_SHIELD_EMERGENCY"

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0
    assert control["throttle"] == 0.0


def test_path_bubble_shield_violation_on_current_path_treats_path_as_unsafe():
    planner = Planner(width=20, height=20)
    data = _base_planning_input(ego=(0, 0), goal=(8, 0))
    # Obstacle alongside path at x=4, y=1.2 (not directly on path line y=0, but inside 2.0m bubble radius)
    data["primary_objects"] = [
        {"position": [4.0, 1.2], "radius": 0.0, "vehicle_relative": False}
    ]

    output = planner.plan(data)

    assert output["path_safe"] is False
    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["bubble_path_safe"] is False

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0
    assert control["throttle"] == 0.0


def test_green_light_with_bubble_violation_stops_cleanly():
    planner = Planner(width=20, height=20)
    data = _base_planning_input(ego=(0, 0), goal=(8, 0))
    # Obstacle in safety bubble (1.4m away, > 1.0m emergency) with GREEN light
    data["primary_objects"] = [
        {"position": [0.0, 1.4], "radius": 0.0, "vehicle_relative": True}
    ]
    data["traffic_light"] = "GREEN"

    output = planner.plan(data)

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["path_safe"] is False
    assert output["bubble_safe"] is False

    controller = VehicleController()
    control = controller.compute_control(output, vehicle_location=[0.0, 0.0], vehicle_heading=0.0)
    assert control["brake"] == 1.0
    assert control["throttle"] == 0.0
