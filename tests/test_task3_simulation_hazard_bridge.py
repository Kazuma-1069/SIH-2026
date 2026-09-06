import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration.data_adapter import perception_to_planning_input
from integration.pipeline import IntegrationPipeline
from integration.simulation_hazard_bridge import (
    bridge_scenario_hazards,
    is_hazard_in_perception_range,
    scenario_hazard_to_road_hazard,
    world_to_vehicle_relative,
)
from interfaces.perception_output import (
    PerceptionOutput,
    PerceptionObject,
    RoadHazard,
)
from perception.perception_pipeline import PerceptionPipeline
from planning.obstacle_map import ObstacleMap
from planning.planner import Planner
from simulation.controller import VehicleController
from simulation.scenario_manager import ScenarioManager


def _scenario_hazard(
    hazard_type,
    world_x,
    world_y,
    distance_ahead=None,
    lateral_offset=None,
):
    hazard = {
        "id": 1,
        "type": hazard_type,
        "location": {
            "x": world_x,
            "y": world_y,
            "z": 0.0,
        },
        "severity": "high",
        "dynamic": False,
        "active": True,
    }

    if distance_ahead is not None:
        hazard["distance_ahead"] = distance_ahead

    if lateral_offset is not None:
        hazard["lateral_offset"] = lateral_offset

    return hazard


def test_simulation_hazard_converts_to_perception_format():
    scenario_hazards = [
        _scenario_hazard(
            "pothole",
            world_x=34.0,
            world_y=-0.9,
        ),
        _scenario_hazard(
            "construction_barricade",
            world_x=46.0,
            world_y=0.0,
        ),
    ]

    bridged = bridge_scenario_hazards(
        scenario_hazards,
        ego_x=10.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
    )

    assert len(bridged) == 2
    assert isinstance(bridged[0], RoadHazard)
    assert bridged[0].hazard_type == "pothole"
    assert bridged[1].hazard_type == "construction_barricade"
    assert bridged[0].position == [24.0, -0.9]
    assert bridged[0].distance == 24.0
    assert bridged[0].confidence > 0.9


def test_pothole_becomes_valid_planning_obstacle():
    road_hazard = scenario_hazard_to_road_hazard(
        _scenario_hazard("pothole", 30.0, 1.0),
        ego_x=10.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
    )

    perception = PerceptionOutput(
        frame_id=1,
        image_width=1280,
        image_height=720,
        hazards=[road_hazard],
    )

    planning_input = perception_to_planning_input(
        perception
    )

    assert len(planning_input["fallback_anomalies"]) == 1
    obstacle = planning_input["fallback_anomalies"][0]
    assert obstacle["class_name"] == "pothole"
    assert obstacle["vehicle_relative"] is True
    assert obstacle["position"] == [20.0, 1.0]

    obstacle_map = ObstacleMap(width=20, height=20)
    obstacle_map.update_from_objects(
        planning_input["fallback_anomalies"]
    )
    assert len(obstacle_map.get_obstacles()) == 1


def test_barricade_becomes_valid_planning_obstacle():
    road_hazard = scenario_hazard_to_road_hazard(
        _scenario_hazard(
            "construction_barricade",
            46.0,
            -2.0,
        ),
        ego_x=10.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
    )

    planning_input = perception_to_planning_input(
        PerceptionOutput(
            frame_id=1,
            image_width=1280,
            image_height=720,
            hazards=[road_hazard],
        )
    )

    assert len(planning_input["fallback_anomalies"]) == 1
    obstacle = planning_input["fallback_anomalies"][0]
    assert obstacle["class_name"] == "construction_barricade"
    assert obstacle["position"] == [36.0, -2.0]
    assert obstacle["radius"] > 0.0


def test_hazard_coordinates_use_vehicle_relative_metric_convention():
    position = world_to_vehicle_relative(
        ego_x=10.0,
        ego_y=5.0,
        ego_yaw_deg=90.0,
        world_x=10.0,
        world_y=29.0,
    )

    assert position[0] == 24.0
    assert position[1] == 0.0

    road_hazard = scenario_hazard_to_road_hazard(
        _scenario_hazard(
            "pedestrian",
            10.0,
            29.0,
        ),
        ego_x=10.0,
        ego_y=5.0,
        ego_yaw_deg=90.0,
    )

    assert road_hazard.position == [24.0, 0.0]
    assert road_hazard.distance == 24.0


def test_hazards_outside_perception_range_are_ignored():
    near_hazard = scenario_hazard_to_road_hazard(
        _scenario_hazard("pothole", 30.0, 0.0),
        ego_x=10.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
        max_range_m=60.0,
    )
    far_hazard = scenario_hazard_to_road_hazard(
        _scenario_hazard("pothole", 100.0, 0.0),
        ego_x=10.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
        max_range_m=60.0,
    )
    behind_hazard = scenario_hazard_to_road_hazard(
        _scenario_hazard("pothole", 5.0, 0.0),
        ego_x=10.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
        max_range_m=60.0,
    )

    assert near_hazard is not None
    assert far_hazard is None
    assert behind_hazard is None
    assert is_hazard_in_perception_range([20.0, 0.0]) is True
    assert is_hazard_in_perception_range([80.0, 0.0]) is False
    assert is_hazard_in_perception_range([-2.0, 0.0]) is False


class FakeDetector:
    def detect(self, frame):
        return [
            {
                "track_id": 1,
                "class_id": 2,
                "class_name": "car",
                "confidence": 0.95,
                "bbox": [100, 100, 300, 250],
            },
            {
                "track_id": 2,
                "class_id": 0,
                "class_name": "person",
                "confidence": 0.88,
                "bbox": [500, 100, 650, 400],
            },
        ]


class FakeTracker:
    def update(self, detections):
        return detections


def test_yolo_vehicle_and_pedestrian_detections_continue_to_work():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    pipeline = PerceptionPipeline(
        detector=FakeDetector(),
        tracker=FakeTracker(),
    )

    output = pipeline.process_frame(frame)

    assert len(output.objects) == 2
    class_names = {
        obj.class_name
        for obj in output.objects
    }
    assert class_names == {"car", "person"}


class FakeScenarioManager:
    def __init__(self, hazards):
        self._hazards = hazards

    def get_hazards(self):
        return self._hazards


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
    def __init__(self, location=(10.0, 0.0), yaw=0.0):
        self._location = FakeLocation(*location)
        self._transform = FakeTransform(yaw)
        self.last_control = None

    def get_location(self):
        return self._location

    def get_transform(self):
        return self._transform

    def has_reached_destination(self):
        return False

    def apply_control(self, throttle, steer, brake):
        self.last_control = {
            "throttle": throttle,
            "steer": steer,
            "brake": brake,
        }


class FakePerceptionPipeline:
    def process_frame(self, frame):
        return PerceptionOutput(
            frame_id=1,
            image_width=1280,
            image_height=720,
            objects=[
                PerceptionObject(
                    track_id=1,
                    class_id=2,
                    class_name="car",
                    confidence=0.95,
                    bbox=[100, 100, 300, 250],
                    distance=12.0,
                )
            ],
        )


class RecordingPlanner:
    def __init__(self):
        self.last_planning_input = None

    def plan(self, planning_input):
        self.last_planning_input = planning_input
        return {
            "action": "PROCEED_FORWARD",
            "target_speed_mps": 5.0,
            "algorithm": "A_STAR",
            "hazard_count": len(
                planning_input.get(
                    "primary_objects",
                    [],
                )
            )
            + len(
                planning_input.get(
                    "fallback_anomalies",
                    [],
                )
            ),
            "waypoints": [[0, 0], [10, 10]],
            "path_safe": True,
            "safety_reason": "PATH_CLEAR",
            "confidence_uncertainty": {},
            "bubble_safe": True,
            "bubble_emergency": False,
            "bubble_path_safe": True,
        }


def test_pipeline_supplements_yolo_with_simulation_hazards():
    scenario_manager = FakeScenarioManager(
        [
            _scenario_hazard("pothole", 34.0, -0.9),
            _scenario_hazard(
                "construction_barricade",
                120.0,
                0.0,
            ),
        ]
    )

    planner = RecordingPlanner()
    pipeline = IntegrationPipeline(
        perception_pipeline=FakePerceptionPipeline(),
        planner=planner,
        controller=VehicleController(),
        vehicle=FakeVehicle(),
        scenario_manager=scenario_manager,
    )

    _, planning_output, control_command = pipeline.process_frame(
        object()
    )

    planning_input = planner.last_planning_input

    assert len(planning_input["primary_objects"]) == 1
    assert planning_input["primary_objects"][0]["class_name"] == "car"
    assert len(planning_input["fallback_anomalies"]) == 1
    assert (
        planning_input["fallback_anomalies"][0]["class_name"]
        == "pothole"
    )
    assert planning_output["hazard_count"] == 2
    assert control_command is not None


def test_pipeline_preserves_bridged_metric_position_and_stops_on_blocked_route():
    scenario_manager = FakeScenarioManager(
        [_scenario_hazard("parked_vehicle", 35.0, 0.0)]
    )
    vehicle = FakeVehicle(location=(10.0, 0.0), yaw=0.0)
    planner = Planner(width=20, height=20)
    pipeline = IntegrationPipeline(
        perception_pipeline=FakePerceptionPipeline(),
        planner=planner,
        controller=VehicleController(),
        vehicle=vehicle,
        scenario_manager=scenario_manager,
    )
    pipeline.road_waypoints = [
        [10.0, 0.0],
        [15.0, 0.0],
        [20.0, 0.0],
        [25.0, 0.0],
        [30.0, 0.0],
        [35.0, 0.0],
    ]

    perception, planning_output, control_command = pipeline.process_frame(
        object()
    )

    assert perception.hazards[0].position == [25.0, 0.0]
    assert planning_output["action"] == "STOP"
    assert planning_output["safety_reason"] in {
        "BUBBLE_SHIELD_EMERGENCY",
        "NO_SAFE_ROAD_DETOUR",
    }
    assert control_command["throttle"] == 0.0
    assert control_command["brake"] == 1.0


def test_pothole_hazard_stays_active_without_visual_actor():
    manager = ScenarioManager.__new__(ScenarioManager)
    manager.actors = []
    manager.hazards = []

    transform = type(
        "Transform",
        (),
        {"location": FakeLocation(34.0, 0.0)},
    )()
    manager._record_hazard(
        "pothole",
        actor=None,
        transform=transform,
        distance_ahead=24.0,
        lateral_offset=0.0,
        severity="medium",
    )

    assert manager.hazards[0]["active"] is True
    bridged = bridge_scenario_hazards(
        manager.hazards,
        ego_x=10.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
    )
    assert len(bridged) == 1
    assert bridged[0].hazard_type == "pothole"


def test_simulation_hazard_replanning_reaches_planner():
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

    pothole_hazard = scenario_hazard_to_road_hazard(
        _scenario_hazard("pothole", 13.0, 0.0),
        ego_x=10.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
    )
    planning_input = perception_to_planning_input(
        PerceptionOutput(
            frame_id=1,
            image_width=1280,
            image_height=720,
            hazards=[pothole_hazard],
        )
    )
    planning_input["ego_position"] = [0, 0]
    planning_input["goal"] = [6, 0]
    planning_input["current_path"] = straight_path

    blocked_output = planner.plan(planning_input)

    assert len(planning_input["fallback_anomalies"]) == 1
    assert blocked_output["waypoints"] != clear_output["waypoints"] or (
        blocked_output["replanned"]
        and blocked_output["current_path_blocked"]
    )
