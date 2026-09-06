import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration.pipeline import IntegrationPipeline
from planning.planner import Planner
from simulation.controller import VehicleController
from interfaces.perception_output import PerceptionOutput


class FakeLocation:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeRotation:
    def __init__(self, yaw=0.0):
        self.yaw = yaw


class FakeTransform:
    def __init__(self, yaw=0.0):
        self.rotation = FakeRotation(yaw)


class FakeVehicle:
    def __init__(
        self,
        location=(10.0, 10.0),
        yaw=0.0,
        destination=(50.0, 10.0),
        route=None,
    ):
        self._location = FakeLocation(*location)
        self._transform = FakeTransform(yaw)
        self.destination = FakeLocation(*destination)
        self._route = route
        self.last_control = None

    def get_location(self):
        return self._location

    def get_transform(self):
        return self._transform

    def has_reached_destination(self):
        return False

    def generate_route(self, location, destination):
        if self._route is not None:
            return self._route
        return [
            FakeLocation(location.x, location.y),
            FakeLocation(25.0, 10.0),
            FakeLocation(50.0, 10.0),
        ]

    def apply_control(self, throttle, steer, brake):
        self.last_control = {
            "throttle": throttle,
            "steer": steer,
            "brake": brake,
        }


class FakePerceptionPipeline:
    def __init__(self, objects=None, hazards=None):
        self.objects = objects or []
        self.hazards = hazards or []

    def process_frame(self, frame):
        return PerceptionOutput(
            frame_id=1,
            image_width=1280,
            image_height=720,
            objects=self.objects,
            hazards=self.hazards,
        )


class RecordingPlanner:
    def __init__(self, waypoints=None):
        self.last_planning_input = None
        self.waypoints = waypoints or [[2, 2], [4, 2], [6, 2]]

    def plan(self, planning_input):
        self.last_planning_input = planning_input
        return {
            "action": "PROCEED_FORWARD",
            "target_speed_mps": 5.0,
            "algorithm": "A_STAR",
            "hazard_count": 0,
            "waypoints": [list(point) for point in self.waypoints],
            "path_safe": True,
            "safety_reason": "PATH_CLEAR",
            "confidence_uncertainty": {},
            "bubble_safe": True,
            "bubble_emergency": False,
            "bubble_path_safe": True,
        }


def _make_pipeline(
    planner,
    vehicle=None,
    perception=None,
):
    return IntegrationPipeline(
        perception_pipeline=perception or FakePerceptionPipeline(),
        planner=planner,
        controller=VehicleController(),
        vehicle=vehicle,
        destination=getattr(vehicle, "destination", None),
    )


def test_safe_global_path_used_without_blocking_obstacle():
    vehicle = FakeVehicle(
        location=(10.0, 10.0),
        destination=(50.0, 10.0),
    )
    pipeline = _make_pipeline(
        Planner(width=20, height=20),
        vehicle=vehicle,
    )

    _, planning_output, control_command = pipeline.process_frame(object())

    assert planning_output["path_safe"] is True
    assert planning_output["action"] in {
        "PROCEED_FORWARD",
        "SLOW_AND_REROUTE",
    }
    assert planning_output["waypoints"]
    assert pipeline.road_waypoints is not None

    first_world = planning_output["waypoints"][0]
    first_global = pipeline.road_waypoints[0]
    assert first_world == first_global

    assert control_command is not None
    assert control_command["brake"] == 0.0
    assert control_command["throttle"] > 0.0


def test_blocking_obstacle_triggers_local_replanning():
    planner = Planner(width=20, height=20)
    straight_path = [[index, 0] for index in range(7)]

    clear_output = planner.plan(
        {
            "primary_objects": [],
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 0],
            "goal": [6, 0],
            "current_path": straight_path,
        }
    )

    blocked_output = planner.plan(
        {
            "primary_objects": [
                {
                    "position": [3, 0],
                    "radius": 1,
                }
            ],
            "fallback_anomalies": [],
            "drivable_space": {"obstacle_occupied_cells": 1},
            "confidence_uncertainty": {},
            "ego_position": [0, 0],
            "goal": [6, 0],
            "current_path": straight_path,
        }
    )

    assert blocked_output["replanned"] is True
    assert blocked_output["current_path_blocked"] is True
    assert blocked_output["waypoints"] != clear_output["waypoints"]
    assert blocked_output["action"] in {
        "REROUTE",
        "SLOW_AND_REROUTE",
        "STOP",
    }


def test_planner_output_reaches_controller_without_road_overwrite():
    distinctive_waypoints = [[2, 2], [5, 2], [8, 2]]
    global_road_waypoints = [[100.0, 100.0], [200.0, 200.0], [300.0, 300.0]]

    vehicle = FakeVehicle(
        location=(10.0, 10.0),
        destination=(50.0, 10.0),
        route=[
            FakeLocation(point[0], point[1])
            for point in global_road_waypoints
        ],
    )
    planner = RecordingPlanner(
        waypoints=distinctive_waypoints
    )
    pipeline = _make_pipeline(
        planner,
        vehicle=vehicle,
    )

    _, planning_output, control_command = pipeline.process_frame(object())

    expected_world = [
        pipeline.coordinate_adapter.grid_to_world(point)
        for point in distinctive_waypoints
    ]

    assert planning_output["waypoints"] == expected_world
    assert planning_output["waypoints"] != global_road_waypoints
    assert control_command is not None
    assert control_command["brake"] == 0.0


def test_no_safe_path_commands_emergency_stop():
    planner = Planner(width=5, height=5)
    wall_obstacles = [
        {"position": [2, row], "radius": 0}
        for row in range(5)
    ]

    output = planner.plan(
        {
            "primary_objects": wall_obstacles,
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {},
            "ego_position": [0, 0],
            "goal": [4, 4],
            "current_path": [[0, 0], [1, 0], [2, 0], [3, 0], [4, 4]],
        }
    )

    assert output["action"] == "STOP"
    assert output["path_safe"] is False
    assert output["safety_reason"] == "NO_PATH_FOUND"
    assert output["waypoints"] == []

    controller = VehicleController()
    control = controller.compute_control(
        output,
        vehicle_location=[0.0, 0.0],
        vehicle_heading=0.0,
    )

    assert control["throttle"] == 0.0
    assert control["steer"] == 0.0
    assert control["brake"] == 1.0
