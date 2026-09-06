import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
from planning.planner import Planner
from simulation.controller import VehicleController
from integration.pipeline import IntegrationPipeline
from interfaces.perception_output import PerceptionOutput


class FakeLocation:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeVehicle:
    def __init__(self, location=(0.0, 0.0), destination=(10.0, 0.0), route=None):
        self._location = FakeLocation(*location)
        self._transform = type("T", (), {"rotation": type("R", (), {"yaw": 0.0})()})()
        self.destination = FakeLocation(*destination)
        self._route = route or []
        self.last_control = None

    def get_location(self):
        return self._location

    def get_transform(self):
        return self._transform

    def generate_route(self, *_, **__):
        return self._route

    def has_reached_destination(self):
        return False

    def apply_control(self, throttle, steer, brake):
        self.last_control = {"throttle": throttle, "steer": steer, "brake": brake}


def test_dynamic_replan_prefers_safe_local_detour_when_path_blocked():
    planner = Planner(width=20, height=20)
    straight_path = [[x, 5] for x in range(11)]
    goal = [10, 5]

    output = planner.plan(
        {
            "primary_objects": [
                {"position": [5, 5], "radius": 0.5, "vehicle_relative": True}
            ],
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 5],
            "goal": goal,
            "current_path": straight_path,
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "REROUTE"
    assert output["replanned"] is True
    assert output["current_path_blocked"] is True
    assert output["path_safe"] is True
    assert output["target_speed_mps"] == planner.reduced_speed_mps
    assert output["destination"] == goal
    assert len(output["waypoints"]) > 0
    # Detour must reach the goal
    assert output["waypoints"][-1] == goal
    # Detour must avoid the blocked point (5, 5)
    assert [5, 5] not in output["waypoints"]


def test_dynamic_replan_triggered_when_obstacle_makes_path_unsafe_via_bubble():
    planner = Planner(width=20, height=20)
    straight_path = [[x, 5] for x in range(11)]
    goal = [10, 5]

    # Obstacle is at (5, 6.5) - not directly on path y=5, but distance is 1.5 <= bubble radius 2.0
    output = planner.plan(
        {
            "primary_objects": [
                {"position": [5.0, 6.5], "radius": 0.0}
            ],
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 5],
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
    assert output["waypoints"][-1] == goal


def test_dynamic_replan_stops_with_zero_speed_when_no_detour_exists():
    planner = Planner(width=10, height=10)
    goal = [9, 5]
    straight_path = [[x, 5] for x in range(10)]

    # Complete wall at x=5 from y=0 to y=9
    wall = [{"position": [5, y], "radius": 0} for y in range(10)]

    output = planner.plan(
        {
            "primary_objects": wall,
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 5],
            "goal": goal,
            "current_path": straight_path,
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["waypoints"] == []
    assert output["path_safe"] is False
    assert output["safety_reason"] == "NO_PATH_FOUND"
    assert output["destination"] == goal


def test_dynamic_replan_stops_when_candidate_detour_violates_bubble_shield():
    planner = Planner(width=10, height=5)
    goal = [9, 2]
    straight_path = [[x, 2] for x in range(10)]

    # Block path at (5, 2), and place obstacles at (5, 0), (5, 1), (5, 3), (5, 4) with bubble overlap
    obstacles = [
        {"position": [5, 2], "radius": 0},
        {"position": [5, 0], "radius": 0.8},
        {"position": [5, 4], "radius": 0.8},
    ]

    output = planner.plan(
        {
            "primary_objects": obstacles,
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 2],
            "goal": goal,
            "current_path": straight_path,
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["waypoints"] == []
    assert output["path_safe"] is False
    assert output["safety_reason"] in {"BUBBLE_SHIELD_PATH_BLOCKED", "NO_PATH_FOUND"}
    assert output["destination"] == goal


def test_dynamic_replan_respects_drivable_corridor():
    planner = Planner(width=20, height=20)
    straight_path = [[x, 5] for x in range(11)]
    goal = [10, 5]

    # Corridor allows y in [4, 5, 6] only. If obstacle at (5, 5) forces detour outside corridor, it stops.
    # Restrict corridor to ONLY the straight path y=5
    restricted_corridor = [[x, 5] for x in range(11)]

    output = planner.plan(
        {
            "primary_objects": [
                {"position": [5, 5], "radius": 0.5}
            ],
            "fallback_anomalies": [],
            "drivable_space": {
                "allow_detour": True,
                "drivable_corridor": restricted_corridor,
            },
            "confidence_uncertainty": {},
            "ego_position": [0, 5],
            "goal": goal,
            "current_path": straight_path,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["waypoints"] == []
    assert output["safety_reason"] == "NO_SAFE_ROAD_DETOUR"


def test_pipeline_preserves_replanned_waypoints_against_global_road_waypoints():
    planner = Planner(width=20, height=20)
    straight_path = [[x, 0] for x in range(10)]
    global_route = [
        FakeLocation(0.0, 0.0),
        FakeLocation(50.0, 0.0),
        FakeLocation(100.0, 0.0),
    ]
    vehicle = FakeVehicle(
        location=(0.0, 0.0),
        destination=(100.0, 0.0),
        route=global_route,
    )

    dummy_perception = type(
        "DummyPerception",
        (),
        {
            "process_frame": lambda self, frame: PerceptionOutput(
                frame_id=1,
                image_width=1280,
                image_height=720,
                objects=[
                    type(
                        "Obj",
                        (),
                        {
                            "track_id": 1,
                            "class_id": 2,
                            "class_name": "car",
                            "confidence": 0.9,
                            "bbox": [100, 100, 200, 200],
                            "distance": 5.0,
                            "position": [5.0, 0.0],
                            "radius": 0.5,
                            "velocity": None,
                            "predicted_position": None,
                        },
                    )()
                ],
                hazards=[],
            )
        },
    )()

    pipeline = IntegrationPipeline(
        perception_pipeline=dummy_perception,
        planner=planner,
        controller=VehicleController(),
        vehicle=vehicle,
        destination=vehicle.destination,
    )
    pipeline.road_waypoints = [[0.0, 0.0], [50.0, 0.0], [100.0, 0.0]]
    planner.current_path = straight_path

    _, planning_output, control_command = pipeline.process_frame(object())

    assert planning_output["action"] in {"STOP", "REROUTE"}
    if planning_output["action"] == "REROUTE":
        assert planning_output["waypoints"] != pipeline.road_waypoints


def test_bubble_shield_emergency_overrides_detour_replanning():
    planner = Planner(width=20, height=20)
    straight_path = [[x, 0] for x in range(10)]

    # Obstacle at distance 0.5 <= emergency radius 1.0
    output = planner.plan(
        {
            "primary_objects": [
                {"position": [0.5, 0.0], "radius": 0.0, "vehicle_relative": True}
            ],
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 0],
            "goal": [9, 0],
            "current_path": straight_path,
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["safety_reason"] == "BUBBLE_SHIELD_EMERGENCY"
    assert output["bubble_emergency"] is True


def test_traffic_light_red_overrides_detour_replanning():
    planner = Planner(width=20, height=20)
    straight_path = [[x, 0] for x in range(10)]

    # Obstacle on path AND traffic light is RED
    output = planner.plan(
        {
            "primary_objects": [
                {"position": [5, 0], "radius": 0.5, "vehicle_relative": True}
            ],
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 0],
            "goal": [9, 0],
            "current_path": straight_path,
            "traffic_light_state": "RED",
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "STOP"
    assert output["target_speed_mps"] == 0.0
    assert output["safety_reason"] == "TRAFFIC_LIGHT_RED"


def test_traffic_light_yellow_with_detour_slows_vehicle():
    planner = Planner(width=20, height=20)
    straight_path = [[x, 5] for x in range(11)]
    goal = [10, 5]

    output = planner.plan(
        {
            "primary_objects": [
                {"position": [5, 5], "radius": 0.5, "vehicle_relative": True}
            ],
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 5],
            "goal": goal,
            "current_path": straight_path,
            "traffic_light_state": "YELLOW",
            "allow_local_detour": True,
        }
    )

    assert output["action"] == "SLOW"
    assert output["target_speed_mps"] == planner.reduced_speed_mps
    assert output["replanned"] is True
    assert output["destination"] == goal
    assert len(output["waypoints"]) > 0
