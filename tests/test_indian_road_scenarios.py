import sys
from pathlib import Path

import carla
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration.data_adapter import perception_to_planning_input
from integration.simulation_hazard_bridge import bridge_scenario_hazards
from interfaces.perception_output import PerceptionOutput
from planning.planner import Planner
from simulation.scenario_manager import ScenarioManager


class FakeBlueprint:
    def __init__(self, blueprint_id="vehicle.fake"):
        self.id = blueprint_id

    def has_attribute(self, _name):
        return False

    def set_attribute(self, _name, _value):
        return None


class FakeBlueprintLibrary:
    def __init__(self):
        self.blueprint = FakeBlueprint()

    def filter(self, _pattern):
        return [self.blueprint]

    def find(self, _blueprint_id):
        return self.blueprint


class FakeActor:
    next_id = 1

    def __init__(self, transform):
        self.id = FakeActor.next_id
        FakeActor.next_id += 1
        self._transform = transform
        self.is_alive = True

    def get_transform(self):
        return self._transform

    def set_transform(self, transform):
        self._transform = transform

    def destroy(self):
        self.is_alive = False


class FakeMap:
    def get_waypoint(self, location, **_kwargs):
        return type(
            "Waypoint",
            (),
            {
                "transform": carla.Transform(
                    carla.Location(location.x, location.y, 0.0),
                    carla.Rotation(yaw=0.0),
                )
            },
        )()


class FakeWorld:
    def __init__(self):
        self.map = FakeMap()
        self.library = FakeBlueprintLibrary()
        self.actors = {}

    def get_map(self):
        return self.map

    def get_blueprint_library(self):
        return self.library

    def try_spawn_actor(self, _blueprint, transform):
        actor = FakeActor(transform)
        self.actors[actor.id] = actor
        return actor

    def get_actor(self, actor_id):
        return self.actors.get(actor_id)


class FakeVehicle:
    def __init__(self):
        self._transform = carla.Transform(
            carla.Location(0.0, 0.0, 0.0),
            carla.Rotation(yaw=0.0),
        )

    def get_transform(self):
        return self._transform


def _make_manager():
    return ScenarioManager(FakeWorld(), FakeVehicle())


def test_all_indian_road_scenarios_create_expected_hazards_and_bridge():
    expected_types = {
        "human_crossing": {"human_crossing"},
        "bike_ahead": {"bike_ahead"},
        "sudden_stopping_car": {"sudden_stopping_car"},
        "uneven_road": {"uneven_road"},
        "potholes": {"pothole"},
        "combined_indian_road": {
            "pothole",
            "human_crossing",
            "bike_ahead",
            "sudden_stopping_car",
            "uneven_road",
        },
        "no_road": {"no_road"},
    }

    for scenario_name, required_types in expected_types.items():
        manager = _make_manager()
        assert manager.set_scenario(scenario_name) is True

        hazards = manager.get_hazards()
        hazard_types = {hazard["type"] for hazard in hazards}
        assert required_types <= hazard_types
        assert hazards
        assert all(hazard["active"] for hazard in hazards)

        bridged = bridge_scenario_hazards(
            hazards,
            ego_x=0.0,
            ego_y=0.0,
            ego_yaw_deg=0.0,
        )
        assert {hazard.hazard_type for hazard in bridged} >= required_types

        planning_input = perception_to_planning_input(
            PerceptionOutput(
                frame_id=1,
                image_width=1280,
                image_height=720,
                hazards=bridged,
            )
        )
        planning_types = {
            obstacle["class_name"]
            for obstacle in planning_input["fallback_anomalies"]
        }
        assert required_types <= planning_types


def test_crossing_and_bike_move_toward_ego_lane():
    for scenario_name in ("human_crossing", "bike_ahead"):
        manager = _make_manager()
        manager.set_scenario(scenario_name)
        hazard = manager.hazards[0]
        initial_lateral = hazard["lateral_offset"]

        manager.update_dynamic_obstacles(delta_seconds=1.0)

        assert hazard["lateral_offset"] != initial_lateral
        assert hazard["active"] is True


def test_human_crossing_uses_fixed_crossing_axis():
    manager = _make_manager()
    manager.set_scenario("human_crossing")
    hazard = manager.hazards[0]

    assert hazard["lateral_offset"] == -4.0
    assert hazard["target_lateral_offset"] == 0.0
    assert hazard["crossing_axis_vector"] == [0.0, 1.0]
    initial_location = dict(hazard["location"])

    manager.update_dynamic_obstacles(delta_seconds=1.0)

    assert hazard["lateral_offset"] == -2.5
    assert hazard["location"]["x"] == 20.0
    assert hazard["location"]["y"] - initial_location["y"] == 1.5


def test_uneven_road_obstacles_are_inside_configured_section():
    manager = _make_manager()
    manager.set_scenario("uneven_road")
    section_start = manager.SCENARIO_CONFIG["uneven_road"][
        "section_start_distance"
    ]
    section_end = manager.SCENARIO_CONFIG["uneven_road"][
        "section_end_distance"
    ]

    obstacles = [
        hazard
        for hazard in manager.hazards
        if hazard["type"] != "uneven_road"
    ]
    assert {hazard["type"] for hazard in obstacles} == {
        "pothole",
        "construction_barricade",
    }
    assert all(
        section_start <= hazard["distance_ahead"] <= section_end
        and hazard["on_uneven_road"] is True
        for hazard in obstacles
    )

    bridged = bridge_scenario_hazards(
        obstacles,
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
    )
    planning_input = perception_to_planning_input(
        PerceptionOutput(
            frame_id=1,
            image_width=1280,
            image_height=720,
            hazards=bridged,
        )
    )
    assert {
        obstacle["class_name"]
        for obstacle in planning_input["fallback_anomalies"]
    } == {"pothole", "construction_barricade"}
    assert all(
        obstacle["metadata"]["on_uneven_road"] is True
        for obstacle in planning_input["fallback_anomalies"]
    )


def test_combined_hazards_share_the_uneven_road_section():
    manager = _make_manager()
    manager.set_scenario("combined_indian_road")
    section_start = 18.0
    section_end = 32.0

    section_hazards = [
        hazard
        for hazard in manager.hazards
        if hazard.get("on_uneven_road") is True
    ]
    assert {hazard["type"] for hazard in section_hazards} >= {
        "uneven_road",
        "pothole",
        "construction_barricade",
        "human_crossing",
        "bike_ahead",
        "sudden_stopping_car",
    }
    assert all(
        section_start <= hazard["distance_ahead"] <= section_end
        for hazard in section_hazards
    )

    potholes = [
        hazard
        for hazard in section_hazards
        if hazard["type"] == "pothole"
    ]
    assert [hazard["distance_ahead"] for hazard in potholes] == [
        20.0,
        27.0,
    ]
    assert [hazard["lateral_offset"] for hazard in potholes] == [
        -1.0,
        1.0,
    ]


def test_embedded_obstacle_on_current_path_returns_stop():
    manager = _make_manager()
    manager.set_scenario("uneven_road")
    bridged = bridge_scenario_hazards(
        manager.get_hazards(),
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw_deg=0.0,
    )
    planning_input = perception_to_planning_input(
        PerceptionOutput(
            frame_id=1,
            image_width=1280,
            image_height=720,
            hazards=bridged,
        )
    )
    pothole = next(
        obstacle
        for obstacle in planning_input["fallback_anomalies"]
        if obstacle["class_name"] == "pothole"
    )
    pothole["grid_position"] = [4, 0]

    output = Planner(width=20, height=20).plan(
        {
            **planning_input,
            "ego_position": [0, 0],
            "goal": [6, 0],
            "current_path": [[index, 0] for index in range(7)],
        }
    )

    assert output["action"] == "STOP"
    assert output["path_safe"] is False
    assert output["waypoints"] == []


def test_sudden_stopping_car_transitions_from_motion_to_stop():
    manager = _make_manager()
    manager.set_scenario("sudden_stopping_car")
    hazard = manager.hazards[0]
    actor = manager.world.get_actor(hazard["id"])
    initial_x = actor.get_transform().location.x

    manager.update_dynamic_obstacles(delta_seconds=1.0)
    moving_x = actor.get_transform().location.x
    assert moving_x != initial_x
    assert hazard["stopped"] is False

    manager.update_dynamic_obstacles(delta_seconds=3.0)
    stopped_x = actor.get_transform().location.x
    assert hazard["stopped"] is True
    manager.update_dynamic_obstacles(delta_seconds=1.0)
    assert actor.get_transform().location.x == stopped_x


@pytest.mark.parametrize(
    ("scenario_name", "overrides", "expected_type"),
    [
        (
            "human_crossing",
            {"distance_ahead": 12.0, "speed_mps": 0.8},
            "human_crossing",
        ),
        (
            "bike_ahead",
            {"distance_ahead": 18.0, "speed_mps": 2.0},
            "bike_ahead",
        ),
        (
            "sudden_stopping_car",
            {"distance_ahead": 16.0, "stop_after_seconds": 1.0},
            "sudden_stopping_car",
        ),
        (
            "uneven_road",
            {
                "positions": ((10.0, 0.0), (14.0, 0.5)),
                "severity": "high",
            },
            "uneven_road",
        ),
        (
            "potholes",
            {"pothole_positions": ((8.0, 0.0), (20.0, -0.5))},
            "pothole",
        ),
        (
            "no_road",
            {"distance_ahead": 12.0, "radius": 4.0},
            "no_road",
        ),
    ],
)
def test_scenario_overrides_are_deterministic(
    scenario_name,
    overrides,
    expected_type,
):
    manager = _make_manager()
    manager.set_scenario(scenario_name, overrides)

    hazards = manager.get_hazards()
    matching = [
        hazard
        for hazard in hazards
        if hazard["type"] == expected_type
    ]
    assert matching
    assert all(hazard["active"] for hazard in matching)

    if scenario_name == "potholes":
        assert len(matching) == 2
    if scenario_name == "uneven_road":
        assert len(matching) == 2
        assert all(hazard["severity"] == "high" for hazard in matching)
