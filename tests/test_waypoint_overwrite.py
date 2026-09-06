# -*- coding: utf-8 -*-
"""Regression test for waypoint‑overwriting bug.

Ensures that when the planner produces a replanned safe detour (replanned=True)
the IntegrationPipeline does **not** replace the planner‑provided waypoints
with the stored global road waypoints.
"""

import sys
from pathlib import Path

# Ensure repository root is on PYTHONPATH for imports
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration.pipeline import IntegrationPipeline
from simulation.controller import VehicleController
from interfaces.perception_output import PerceptionOutput


class FakeLocation:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeVehicle:
    def __init__(self, location=(0.0, 0.0), yaw=0.0, destination=(10.0, 0.0), route=None):
        self._location = FakeLocation(*location)
        self._transform = type("T", (), {"rotation": type("R", (), {"yaw": yaw})()})()
        self.destination = FakeLocation(*destination)
        self._route = route
        self.last_control = None

    def get_location(self):
        return self._location

    def get_transform(self):
        return self._transform

    def apply_control(self, throttle, steer, brake):
        self.last_control = {"throttle": throttle, "steer": steer, "brake": brake}

    def generate_route(self, *_, **__):
        return self._route or []

    def has_reached_destination(self):
        return False


class ReplannedPlanner:
    """Minimal planner that always signals a replanned safe detour."""

    def __init__(self, waypoints):
        self.waypoints = waypoints

    def plan(self, *_):
        return {
            "action": "REROUTE",
            "target_speed_mps": 5.0,
            "algorithm": "A_STAR",
            "hazard_count": 0,
            "waypoints": [list(p) for p in self.waypoints],
            "path_safe": True,
            "safety_reason": "PATH_CLEAR",
            "confidence_uncertainty": {},
            "bubble_safe": True,
            "bubble_emergency": False,
            "bubble_path_safe": True,
            "replanned": True,
            "current_path_blocked": True,
            "replan_count": 1,
        }


def _make_pipeline(planner, vehicle=None):
    dummy_perception = type(
        "DummyPerception",
        (),
        {"process_frame": lambda self, frame: PerceptionOutput(
            frame_id=1,
            image_width=1280,
            image_height=720,
            objects=[],
            hazards=[],
        )},
    )()
    return IntegrationPipeline(
        perception_pipeline=dummy_perception,
        planner=planner,
        controller=VehicleController(),
        vehicle=vehicle,
        destination=getattr(vehicle, "destination", None),
    )


def test_replanned_path_not_overwritten_by_global_route():
    replanned = [[2, 2], [5, 2], [8, 2]]
    global_route = [[100.0, 100.0], [200.0, 200.0], [300.0, 300.0]]

    vehicle = FakeVehicle(
        location=(10.0, 10.0),
        destination=(50.0, 10.0),
        route=[FakeLocation(*pt) for pt in global_route],
    )

    pipeline = _make_pipeline(ReplannedPlanner(replanned), vehicle=vehicle)

    _, planning_output, control = pipeline.process_frame(object())

    expected_world = [pipeline.coordinate_adapter.grid_to_world(pt) for pt in replanned]

    assert planning_output["replanned"] is True
    assert planning_output["waypoints"] == expected_world
    assert planning_output["waypoints"] != global_route
    assert control is not None
    assert control["brake"] == 0.0
