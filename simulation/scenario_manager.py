"""
CARLA scenario management for SIH 2026.

Supported scenarios:
    - normal
    - normal_driving
    - static_obstacle
    - dynamic_obstacle
    - pothole
    - parked_vehicle
    - pedestrian
    - construction_barricade
    - indian_road_hazards
    - human_crossing
    - bike_ahead
    - sudden_stopping_car
    - uneven_road
    - potholes
    - combined_indian_road
    - no_road
    - combined

CARLA version:
    0.9.16
"""

from __future__ import annotations

import math
import random
from typing import Any, Dict, List, Optional

import carla


class ScenarioManager:
    """
    Creates and manages repeatable M4 simulation scenarios.

    The manager is intentionally independent from M1/M2.
    It exposes standardized hazard information for integration.
    """

    SUPPORTED_SCENARIOS = [
        "normal",
        "normal_driving",
        "static_obstacle",
        "dynamic_obstacle",
        "pothole",
        "parked_vehicle",
        "pedestrian",
        "construction_barricade",
        "indian_road_hazards",
        "human_crossing",
        "bike_ahead",
        "sudden_stopping_car",
        "uneven_road",
        "potholes",
        "combined_indian_road",
        "no_road",
        "combined",
    ]

    SCENARIO_HAZARDS = {
        "normal": (),
        "normal_driving": (),
        "pothole": ("pothole",),
        "parked_vehicle": ("parked_vehicle",),
        "pedestrian": ("pedestrian",),
        "construction_barricade": ("construction_barricade",),
        "indian_road_hazards": (
            "pothole",
            "parked_vehicle",
            "pedestrian",
            "construction_barricade",
        ),
    }

    SCENARIO_CONFIG = {
        "human_crossing": {
            "distance_ahead": 20.0,
            "lateral_offset": -4.0,
            "target_lateral_offset": 0.0,
            "speed_mps": 1.5,
            "crossing_axis": "road_right",
        },
        "bike_ahead": {
            "distance_ahead": 30.0,
            "lateral_offset": 2.5,
            "target_lateral_offset": 0.0,
            "speed_mps": 3.0,
        },
        "sudden_stopping_car": {
            "distance_ahead": 28.0,
            "lateral_offset": 0.0,
            "speed_mps": 4.0,
            "stop_after_seconds": 3.0,
        },
        "uneven_road": {
            "section_start_distance": 18.0,
            "section_end_distance": 32.0,
            "positions": (
                (18.0, 0.0),
                (22.0, 0.0),
                (26.0, 0.0),
            ),
            "severity": "medium",
            "obstacles": (
                {
                    "type": "pothole",
                    "distance_ahead": 20.0,
                    "lateral_offset": -1.0,
                    "radius": 0.6,
                },
                {
                    "type": "construction_barricade",
                    "distance_ahead": 28.0,
                    "lateral_offset": 0.5,
                    "radius": 1.2,
                },
            ),
        },
        "combined_indian_road": {
            "uneven_road": {
                "positions": (
                    (18.0, 0.0),
                    (22.0, 0.0),
                    (26.0, 0.0),
                ),
                "obstacles": (
                    {
                        "type": "pothole",
                        "distance_ahead": 20.0,
                        "lateral_offset": -1.0,
                        "radius": 0.6,
                    },
                    {
                        "type": "pothole",
                        "distance_ahead": 27.0,
                        "lateral_offset": 1.0,
                        "radius": 0.6,
                    },
                    {
                        "type": "construction_barricade",
                        "distance_ahead": 30.0,
                        "lateral_offset": 0.5,
                        "radius": 1.2,
                    },
                ),
            },
            "human_crossing": {
                "distance_ahead": 22.0,
                "lateral_offset": -3.5,
            },
            "bike_ahead": {
                "distance_ahead": 26.0,
                "lateral_offset": 2.0,
            },
            "sudden_stopping_car": {
                "distance_ahead": 30.0,
                "stop_after_seconds": 3.0,
            },
        },
        "no_road": {
            "distance_ahead": 34.0,
            "lateral_offset": 0.0,
            "radius": 3.5,
        },
    }

    POTHOLE_POSITIONS = (
        (20.0, -1.2),
        (38.0, 1.2),
        (56.0, -1.0),
    )

    INDIAN_ROAD_POTHOLE_POSITIONS = (
        (20.0, -1.2),
        (38.0, 1.2),
    )

    def __init__(
        self,
        world: carla.World,
        vehicle: Optional[carla.Vehicle] = None,
        seed: int = 42,
    ):
        self.world = world
        self.vehicle = vehicle
        self.random = random.Random(seed)

        self.active_scenario: str = "normal"
        self.destination: Optional[carla.Location] = None

        self.hazards: List[Dict[str, Any]] = []
        self.actors: List[carla.Actor] = []
        self.elapsed_seconds = 0.0
        self.scenario_overrides: Dict[str, Any] = {}

        print("[M4] Scenario manager initialized")

    # ============================================================
    # SCENARIO API
    # ============================================================

    @classmethod
    def list_scenarios(cls) -> List[str]:
        """
        Return all supported scenario names.
        """
        return list(cls.SUPPORTED_SCENARIOS)

    def set_scenario(
        self,
        scenario_name: str,
        config_overrides: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """
        Activate a scenario.

        Example:
            manager.set_scenario("static_obstacle")
        """

        scenario_name = str(scenario_name).strip().lower()

        if scenario_name not in self.SUPPORTED_SCENARIOS:
            raise ValueError(
                f"Unsupported scenario '{scenario_name}'. "
                f"Supported scenarios: {self.SUPPORTED_SCENARIOS}"
            )

        self.clear_scenario()
        self.scenario_overrides = dict(config_overrides or {})

        self.active_scenario = scenario_name

        if scenario_name in ("normal", "normal_driving"):
            self._setup_normal()

        elif scenario_name == "static_obstacle":
            self._setup_static_obstacle()

        elif scenario_name == "dynamic_obstacle":
            self._setup_dynamic_obstacle()

        elif scenario_name == "indian_road_hazards":
            self._setup_indian_road_hazards()

        elif scenario_name == "human_crossing":
            self._setup_human_crossing()

        elif scenario_name == "bike_ahead":
            self._setup_bike_ahead()

        elif scenario_name == "sudden_stopping_car":
            self._setup_sudden_stopping_car()

        elif scenario_name == "uneven_road":
            self._setup_uneven_road()

        elif scenario_name == "potholes":
            self._spawn_potholes(
                self._configured_pothole_positions(
                    self.POTHOLE_POSITIONS
                )
            )

        elif scenario_name == "combined_indian_road":
            self._setup_combined_indian_road()

        elif scenario_name == "no_road":
            self._setup_no_road()

        elif scenario_name in self.SCENARIO_HAZARDS:
            self._setup_configured_hazards(
                self.SCENARIO_HAZARDS[scenario_name]
            )

        elif scenario_name == "combined":
            self._setup_combined()

        print(f"[M4] Scenario active: {self.active_scenario}")
        print(f"[M4] Hazards: {len(self.hazards)}")

        return True

    def get_active_scenario(self) -> str:
        """
        Return currently active scenario.
        """
        return self.active_scenario

    def get_scenario_state(self) -> Dict[str, Any]:
        """
        Standardized scenario state for M0/M2/M1/M3/M6.
        """
        return {
            "scenario": self.active_scenario,
            "destination": self._location_to_dict(self.destination),
            "hazards": self.get_hazards(),
            "hazard_count": len(self.hazards),
        }

    def get_hazards(self) -> List[Dict[str, Any]]:
        """
        Return standardized hazard descriptions.
        """
        return [dict(hazard) for hazard in self.hazards]

    def get_hazard_count(self) -> int:
        return len(self.hazards)

    def _scenario_config(self, scenario_name: str) -> Dict[str, Any]:
        config = dict(self.SCENARIO_CONFIG.get(scenario_name, {}))
        active_config = self.SCENARIO_CONFIG.get(
            self.active_scenario,
            {},
        )
        config.update(active_config.get(scenario_name, {}))
        config.update(self.scenario_overrides)
        return config

    def _configured_pothole_positions(self, default):
        positions = self.scenario_overrides.get(
            "pothole_positions",
            self.SCENARIO_CONFIG.get(
                self.active_scenario,
                {},
            ).get("pothole_positions", default),
        )
        return tuple(
            (float(distance), float(lateral))
            for distance, lateral in positions
        )

    def get_destination(self) -> Optional[carla.Location]:
        return self.destination

    def set_destination(self, destination: carla.Location) -> None:
        """
        Set the final destination.
        """
        self.destination = destination

        print(
            "Destination set: "
            f"({destination.x:.2f}, "
            f"{destination.y:.2f}, "
            f"{destination.z:.2f})"
        )

    # ============================================================
    # NORMAL
    # ============================================================

    def _setup_normal(self) -> None:
        """
        Empty road scenario.
        """
        self.hazards = []

    # ============================================================
    # STATIC OBSTACLE
    # ============================================================

    def _setup_static_obstacle(self) -> None:
        """
        Spawn a static vehicle/obstacle ahead of the ego vehicle.
        """

        if self.vehicle is None:
            print("[M4] Warning: no ego vehicle supplied.")
            return

        ego_transform = self.vehicle.get_transform()

        forward = ego_transform.get_forward_vector()

        obstacle_location = ego_transform.location + carla.Location(
            x=forward.x * 25.0,
            y=forward.y * 25.0,
            z=0.5,
        )

        # Find a nearby road waypoint so the obstacle is placed
        # approximately on the drivable road surface.
        try:
            map_obj = self.world.get_map()
            waypoint = map_obj.get_waypoint(
                obstacle_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )

            if waypoint is not None:
                obstacle_location = waypoint.transform.location
                obstacle_location.z += 0.5

        except Exception as exc:
            print(f"[M4] Waypoint lookup warning: {exc}")

        blueprint_library = self.world.get_blueprint_library()

        vehicle_blueprints = blueprint_library.filter("vehicle.*")

        if not vehicle_blueprints:
            print("[M4] No vehicle blueprint available.")
            return

        blueprint = vehicle_blueprints[0]

        try:
            if blueprint.has_attribute("color"):
                blueprint.set_attribute("color", "255,0,0")
        except Exception:
            pass

        obstacle_transform = carla.Transform(
            obstacle_location,
            ego_transform.rotation,
        )

        obstacle = self.world.try_spawn_actor(
            blueprint,
            obstacle_transform,
        )

        if obstacle is None:
            print("[M4] Static obstacle spawn failed.")
            return

        self.actors.append(obstacle)

        self.hazards.append(
            {
                "id": obstacle.id,
                "type": "static_obstacle",
                "location": {
                    "x": obstacle_location.x,
                    "y": obstacle_location.y,
                    "z": obstacle_location.z,
                },
                "distance_ahead": 25.0,
                "lateral_offset": 0.0,
                "severity": "high",
                "dynamic": False,
                "active": True,
            }
        )

        print("[M4] Static obstacle spawned.")
        print(f"[M4] Obstacle ID: {obstacle.id}")
        print("[M4] Distance ahead: 25.0 m")
        print("[M4] Lateral offset: 0.0 m")

    # ============================================================
    # DYNAMIC OBSTACLE
    # ============================================================

    def _setup_dynamic_obstacle(self) -> None:
        """
        Spawn a vehicle ahead of the ego vehicle.

        The obstacle is marked dynamic and can be updated every
        simulation iteration using update_dynamic_obstacles().
        """

        if self.vehicle is None:
            print("[M4] Warning: no ego vehicle supplied.")
            return

        ego_transform = self.vehicle.get_transform()
        forward = ego_transform.get_forward_vector()

        obstacle_location = ego_transform.location + carla.Location(
            x=forward.x * 30.0,
            y=forward.y * 30.0,
            z=0.5,
        )

        try:
            waypoint = self.world.get_map().get_waypoint(
                obstacle_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )

            if waypoint is not None:
                obstacle_location = waypoint.transform.location
                obstacle_location.z += 0.5

        except Exception as exc:
            print(f"[M4] Waypoint lookup warning: {exc}")

        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter("vehicle.*")

        if not vehicle_blueprints:
            print("[M4] No vehicle blueprint available.")
            return

        blueprint = vehicle_blueprints[0]

        obstacle = self.world.try_spawn_actor(
            blueprint,
            carla.Transform(
                obstacle_location,
                ego_transform.rotation,
            ),
        )

        if obstacle is None:
            print("[M4] Dynamic obstacle spawn failed.")
            return

        self.actors.append(obstacle)

        self.hazards.append(
            {
                "id": obstacle.id,
                "type": "dynamic_obstacle",
                "location": {
                    "x": obstacle_location.x,
                    "y": obstacle_location.y,
                    "z": obstacle_location.z,
                },
                "distance_ahead": 30.0,
                "lateral_offset": 0.0,
                "severity": "high",
                "dynamic": True,
                "speed": 4.0,
                "active": True,
            }
        )

        print("[M4] Dynamic obstacle spawned.")
        print(f"[M4] Obstacle ID: {obstacle.id}")
        print("[M4] Distance ahead: 30.0 m")

    def update_dynamic_obstacles(self, delta_seconds: float = 0.05) -> None:
        """
        Move dynamic hazards.

        Called by the simulation loop.
        """

        self.elapsed_seconds += max(0.0, float(delta_seconds))

        for hazard in self.hazards:
            hazard_type = hazard.get("type")
            if hazard_type not in {
                "dynamic_obstacle",
                "human_crossing",
                "bike_ahead",
                "sudden_stopping_car",
            }:
                continue

            actor_id = hazard.get("id")
            actor = self._find_actor(actor_id)

            if actor is None:
                if hazard_type == "dynamic_obstacle":
                    hazard["active"] = False
                else:
                    hazard["active"] = True
                    hazard["motion_unavailable"] = True
                continue

            if (
                hazard_type == "sudden_stopping_car"
                and self.elapsed_seconds >= float(
                    hazard.get("stop_after_seconds", 3.0)
                )
            ):
                hazard["speed"] = 0.0
                hazard["stopped"] = True
                continue

            speed = float(hazard.get("speed", 4.0))

            transform = actor.get_transform()
            if hazard_type in {"human_crossing", "bike_ahead"}:
                current_lateral = float(
                    hazard.get("lateral_offset", 0.0)
                )
                target_lateral = float(
                    hazard.get("target_lateral_offset", 0.0)
                )
                direction = 1.0 if target_lateral > current_lateral else -1.0
                lateral_step = min(
                    abs(target_lateral - current_lateral),
                    speed * delta_seconds,
                )
                crossing_axis = hazard.get(
                    "crossing_axis_vector",
                    [
                        transform.get_right_vector().x,
                        transform.get_right_vector().y,
                    ],
                )
                new_location = transform.location + carla.Location(
                    x=crossing_axis[0] * direction * lateral_step,
                    y=crossing_axis[1] * direction * lateral_step,
                    z=0.0,
                )
                hazard["lateral_offset"] = (
                    current_lateral + direction * lateral_step
                )
            else:
                forward = transform.get_forward_vector()
                new_location = transform.location + carla.Location(
                    x=forward.x * speed * delta_seconds,
                    y=forward.y * speed * delta_seconds,
                    z=0.0,
                )

            actor.set_transform(
                carla.Transform(
                    new_location,
                    transform.rotation,
                )
            )

            hazard["location"] = {
                "x": new_location.x,
                "y": new_location.y,
                "z": new_location.z,
            }

    def _spawn_actor_hazard(
        self,
        hazard_type: str,
        blueprint_filters,
        config,
        severity: str,
        dynamic: bool = True,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        transform = self._hazard_transform(
            config["distance_ahead"],
            config.get("lateral_offset", 0.0),
        )
        if transform is None:
            return

        blueprints = []
        for blueprint_filter in blueprint_filters:
            blueprints.extend(
                self.world.get_blueprint_library().filter(
                    blueprint_filter
                )
            )
        blueprints = sorted(
            blueprints,
            key=lambda blueprint_item: blueprint_item.id,
        )
        actor = (
            self.world.try_spawn_actor(blueprints[0], transform)
            if blueprints
            else None
        )
        if actor is not None and hasattr(actor, "set_simulate_physics"):
            actor.set_simulate_physics(False)
        if hazard_type in {"human_crossing", "bike_ahead"}:
            right = transform.get_right_vector()
            metadata = {
                **(metadata or {}),
                "crossing_axis_vector": [right.x, right.y],
            }
        self._record_hazard(
            hazard_type,
            actor,
            transform,
            config["distance_ahead"],
            config.get("lateral_offset", 0.0),
            severity,
            dynamic,
            metadata=metadata,
        )

    def _setup_human_crossing(self) -> None:
        config = self._scenario_config("human_crossing")
        self._spawn_actor_hazard(
            "human_crossing",
            ("walker.pedestrian.*",),
            config,
            "high",
            metadata={
                "speed": config["speed_mps"],
                "target_lateral_offset": config[
                    "target_lateral_offset"
                ],
                "crossing_axis": config["crossing_axis"],
                "radius": 1.0,
            },
        )

    def _setup_bike_ahead(self) -> None:
        config = self._scenario_config("bike_ahead")
        self._spawn_actor_hazard(
            "bike_ahead",
            (
                "vehicle.*crossbike*",
                "vehicle.*omafiets*",
                "vehicle.*century*",
                "vehicle.*ninja*",
                "vehicle.*vespa*",
                "vehicle.*yzf*",
                "vehicle.*low_rider*",
            ),
            config,
            "high",
            metadata={
                "speed": config["speed_mps"],
                "target_lateral_offset": config[
                    "target_lateral_offset"
                ],
                "radius": 1.5,
            },
        )

    def _setup_sudden_stopping_car(self) -> None:
        config = self._scenario_config("sudden_stopping_car")
        self._spawn_actor_hazard(
            "sudden_stopping_car",
            ("vehicle.*",),
            config,
            "high",
            metadata={
                "speed": config["speed_mps"],
                "stop_after_seconds": config[
                    "stop_after_seconds"
                ],
                "stopped": False,
                "radius": 2.0,
            },
        )

    def _record_virtual_hazard(
        self,
        hazard_type: str,
        distance_ahead: float,
        lateral_offset: float,
        severity: str,
        radius: float,
        transform: Optional[carla.Transform] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        if transform is None:
            transform = self._hazard_transform(
                distance_ahead,
                lateral_offset,
            )
        if transform is None:
            return

        self._record_hazard(
            hazard_type,
            actor=None,
            transform=transform,
            distance_ahead=distance_ahead,
            lateral_offset=lateral_offset,
            severity=severity,
            dynamic=False,
            metadata={
                "radius": radius,
                **(metadata or {}),
            },
        )

    def _setup_uneven_road(self) -> None:
        config = self._scenario_config("uneven_road")
        blueprint = None
        try:
            blueprint = self.world.get_blueprint_library().find(
                "static.prop.dirtdebris01"
            )
        except Exception:
            pass

        for distance_ahead, lateral_offset in config["positions"]:
            transform = self._hazard_transform(
                distance_ahead,
                lateral_offset,
            )
            if transform is None:
                continue
            actor = (
                self.world.try_spawn_actor(blueprint, transform)
                if blueprint is not None
                else None
            )
            self._record_hazard(
                "uneven_road",
                actor,
                transform,
                distance_ahead,
                lateral_offset,
                config.get("severity", "medium"),
                metadata={
                    "radius": 1.0,
                    "height_m": 0.12,
                    "on_uneven_road": True,
                    "uneven_section_start": config[
                        "section_start_distance"
                    ],
                    "uneven_section_end": config[
                        "section_end_distance"
                    ],
                },
            )

        for obstacle in config.get("obstacles", ()):
            self._spawn_uneven_section_obstacle(obstacle, config)

    def _spawn_uneven_section_obstacle(
        self,
        obstacle: Dict[str, Any],
        section_config: Dict[str, Any],
    ) -> None:
        obstacle_type = obstacle["type"]
        distance_ahead = float(obstacle["distance_ahead"])
        lateral_offset = float(obstacle.get("lateral_offset", 0.0))
        metadata = {
            "radius": float(obstacle.get("radius", 1.0)),
            "on_uneven_road": True,
            "uneven_section_start": section_config[
                "section_start_distance"
            ],
            "uneven_section_end": section_config[
                "section_end_distance"
            ],
        }

        if obstacle_type == "pothole":
            self._record_virtual_hazard(
                "pothole",
                distance_ahead,
                lateral_offset,
                "medium",
                metadata["radius"],
                metadata=metadata,
            )
            return

        blueprint_filters = {
            "construction_barricade": (
                "static.prop.streetbarrier",
            ),
            "parked_vehicle": ("vehicle.*",),
        }.get(obstacle_type)
        if blueprint_filters is None:
            raise ValueError(
                f"Unsupported uneven-road obstacle: {obstacle_type}"
            )

        self._spawn_actor_hazard(
            obstacle_type,
            blueprint_filters,
            {
                "distance_ahead": distance_ahead,
                "lateral_offset": lateral_offset,
            },
            "high",
            dynamic=False,
            metadata=metadata,
        )

    def _setup_no_road(self) -> None:
        config = self._scenario_config("no_road")
        self._record_virtual_hazard(
            "no_road",
            config["distance_ahead"],
            config["lateral_offset"],
            "high",
            config["radius"],
            metadata={"road_continuation": False},
        )

    def _setup_combined_indian_road(self) -> None:
        self._setup_uneven_road()
        self._setup_human_crossing()
        self._setup_bike_ahead()
        self._setup_sudden_stopping_car()

        for hazard in self.hazards:
            hazard.setdefault("on_uneven_road", True)
            hazard.setdefault("uneven_section_start", 18.0)
            hazard.setdefault("uneven_section_end", 32.0)

    # ============================================================
    # POTHOLE
    # ============================================================

    def _setup_configured_hazards(
        self,
        hazard_types,
    ) -> None:
        """Spawn deterministic hazards enabled by the active scenario."""

        for hazard_type in hazard_types:
            spawn_method = getattr(
                self,
                f"_spawn_{hazard_type}",
            )
            spawn_method()

    def _hazard_transform(
        self,
        distance_ahead: float,
        lateral_offset: float = 0.0,
    ) -> Optional[carla.Transform]:
        """Return a deterministic transform projected onto a driving lane."""

        if self.vehicle is None:
            print("[M4] Warning: no ego vehicle supplied.")
            return None

        ego_transform = self.vehicle.get_transform()
        forward = ego_transform.get_forward_vector()
        right = ego_transform.get_right_vector()
        probe = ego_transform.location + carla.Location(
            x=forward.x * distance_ahead + right.x * lateral_offset,
            y=forward.y * distance_ahead + right.y * lateral_offset,
            z=0.0,
        )

        try:
            waypoint = self.world.get_map().get_waypoint(
                probe,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
        except Exception as exc:
            print(f"[M4] Waypoint lookup warning: {exc}")
            waypoint = None

        if waypoint is None:
            return None

        lane_right = waypoint.transform.get_right_vector()
        location = waypoint.transform.location + carla.Location(
            x=lane_right.x * lateral_offset,
            y=lane_right.y * lateral_offset,
            z=0.0,
        )
        location.z += 0.05
        return carla.Transform(
            location,
            waypoint.transform.rotation,
        )

    def _record_hazard(
        self,
        hazard_type: str,
        actor: Optional[carla.Actor],
        transform: carla.Transform,
        distance_ahead: float,
        lateral_offset: float,
        severity: str,
        dynamic: bool = False,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        if actor is not None:
            self.actors.append(actor)

        hazard = {
                "id": actor.id if actor is not None else None,
                "type": hazard_type,
                "location": self._location_to_dict(transform.location),
                "distance_ahead": distance_ahead,
                "lateral_offset": lateral_offset,
                "severity": severity,
                "dynamic": dynamic,
                # The hazard description remains usable by the simulation
                # bridge even when its optional visual actor cannot spawn.
                "active": True,
            }
        if metadata:
            hazard.update(metadata)
        self.hazards.append(hazard)

    def _spawn_pothole(self) -> None:
        self._spawn_potholes(
            self._configured_pothole_positions(
                self.POTHOLE_POSITIONS
            )
        )

    def _spawn_potholes(self, positions) -> None:
        blueprint_library = self.world.get_blueprint_library()
        blueprint = None
        try:
            blueprint = blueprint_library.find("static.prop.dirtdebris01")
        except Exception as exc:
            print(f"[M4] Pothole asset unavailable: {exc}")

        for distance_ahead, lateral_offset in positions:
            transform = self._hazard_transform(
                distance_ahead,
                lateral_offset,
            )
            if transform is None:
                continue

            actor = (
                self.world.try_spawn_actor(blueprint, transform)
                if blueprint is not None
                else None
            )
            self._record_hazard(
                "pothole",
                actor,
                transform,
                distance_ahead,
                lateral_offset,
                "medium",
            )

        print(
            "[M4] Spawned deterministic physical potholes using "
            "static.prop.dirtdebris01."
        )

    def _spawn_parked_vehicle(self) -> None:
        self._spawn_parked_vehicle_at(28.0, 0.0)

    def _spawn_parked_vehicle_at(
        self,
        distance_ahead: float,
        lateral_offset: float,
    ) -> None:
        transform = self._hazard_transform(distance_ahead, lateral_offset)
        if transform is None:
            return

        blueprints = sorted(
            self.world.get_blueprint_library().filter("vehicle.*"),
            key=lambda blueprint_item: blueprint_item.id,
        )
        actor = (
            self.world.try_spawn_actor(blueprints[0], transform)
            if blueprints
            else None
        )
        self._record_hazard(
            "parked_vehicle",
            actor,
            transform,
            distance_ahead,
            lateral_offset,
            "high",
        )

    def _spawn_pedestrian(self) -> None:
        self._spawn_pedestrian_at(22.0, 1.5)

    def _spawn_pedestrian_at(
        self,
        distance_ahead: float,
        lateral_offset: float,
    ) -> None:
        transform = self._hazard_transform(distance_ahead, lateral_offset)
        if transform is None:
            return

        blueprints = sorted(
            self.world.get_blueprint_library().filter(
                "walker.pedestrian.*"
            ),
            key=lambda blueprint_item: blueprint_item.id,
        )
        actor = (
            self.world.try_spawn_actor(blueprints[0], transform)
            if blueprints
            else None
        )
        self._record_hazard(
            "pedestrian",
            actor,
            transform,
            distance_ahead,
            lateral_offset,
            "high",
            True,
        )

    def _spawn_construction_barricade(self) -> None:
        self._spawn_construction_barricade_at(36.0, 0.0)

    def _spawn_construction_barricade_at(
        self,
        distance_ahead: float,
        lateral_offset: float,
    ) -> None:
        transform = self._hazard_transform(distance_ahead, lateral_offset)
        if transform is None:
            return

        try:
            blueprint = self.world.get_blueprint_library().find(
                "static.prop.streetbarrier"
            )
        except Exception as exc:
            print(f"[M4] Barricade asset unavailable: {exc}")
            return

        actor = (
            self.world.try_spawn_actor(blueprint, transform)
        )
        self._record_hazard(
            "construction_barricade",
            actor,
            transform,
            distance_ahead,
            lateral_offset,
            "high",
        )

    def _setup_indian_road_hazards(self) -> None:
        """Spawn the deterministic set of physical Indian road hazards."""

        self._spawn_potholes(
            self._configured_pothole_positions(
                self.INDIAN_ROAD_POTHOLE_POSITIONS
            )
        )
        self._spawn_parked_vehicle_at(30.0, 2.0)
        self._spawn_pedestrian_at(22.0, 1.5)
        self._spawn_construction_barricade_at(36.0, -2.0)

    def _setup_pothole(self) -> None:
        """
        Create a visual pothole hazard.

        CARLA does not provide a universal built-in pothole actor,
        so the pothole is represented as a low road-hazard object.
        """

        if self.vehicle is None:
            print("[M4] Warning: no ego vehicle supplied.")
            return

        ego_transform = self.vehicle.get_transform()
        forward = ego_transform.get_forward_vector()

        pothole_location = ego_transform.location + carla.Location(
            x=forward.x * 22.0,
            y=forward.y * 22.0,
            z=0.05,
        )

        try:
            waypoint = self.world.get_map().get_waypoint(
                pothole_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )

            if waypoint is not None:
                pothole_location = waypoint.transform.location
                pothole_location.z += 0.05

        except Exception as exc:
            print(f"[M4] Waypoint lookup warning: {exc}")

        blueprint_library = self.world.get_blueprint_library()

        # Try a small static object for visual representation.
        props = blueprint_library.filter("static.prop.*")

        actor = None

        if props:
            blueprint = props[0]

            actor = self.world.try_spawn_actor(
                blueprint,
                carla.Transform(
                    pothole_location,
                    ego_transform.rotation,
                ),
            )

        hazard_id = actor.id if actor is not None else None

        if actor is not None:
            self.actors.append(actor)

        self.hazards.append(
            {
                "id": hazard_id,
                "type": "pothole",
                "location": {
                    "x": pothole_location.x,
                    "y": pothole_location.y,
                    "z": pothole_location.z,
                },
                "distance_ahead": 22.0,
                "lateral_offset": 0.0,
                "width": 1.5,
                "depth": 0.15,
                "severity": "medium",
                "dynamic": False,
                "active": True,
            }
        )

        print("[M4] Pothole hazard created.")
        print("[M4] Distance ahead: 22.0 m")

    # ============================================================
    # COMBINED
    # ============================================================

    def _setup_combined(self) -> None:
        """
        Combined multi-hazard scenario.
        """

        self._setup_static_obstacle()

        # Add pothole hazard without clearing the existing obstacle.
        self._add_pothole_hazard()

        print("[M4] Combined scenario configured.")

    def _add_pothole_hazard(self) -> None:
        """
        Add an additional pothole to the combined scenario.
        """

        if self.vehicle is None:
            return

        ego_transform = self.vehicle.get_transform()
        forward = ego_transform.get_forward_vector()

        # Place pothole at a different distance from the static obstacle.
        pothole_location = ego_transform.location + carla.Location(
            x=forward.x * 15.0,
            y=forward.y * 15.0,
            z=0.05,
        )

        try:
            waypoint = self.world.get_map().get_waypoint(
                pothole_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )

            if waypoint is not None:
                pothole_location = waypoint.transform.location
                pothole_location.z += 0.05

        except Exception:
            pass

        self.hazards.append(
            {
                "id": None,
                "type": "pothole",
                "location": {
                    "x": pothole_location.x,
                    "y": pothole_location.y,
                    "z": pothole_location.z,
                },
                "distance_ahead": 15.0,
                "lateral_offset": 0.0,
                "width": 1.5,
                "depth": 0.15,
                "severity": "medium",
                "dynamic": False,
                "active": True,
            }
        )

        print("[M4] Additional pothole hazard added.")

    # ============================================================
    # CLEAR / DESTROY
    # ============================================================

    def clear_scenario(self) -> None:
        """
        Remove actors and reset hazards.
        """

        self._destroy_actors()

        self.hazards = []
        self.active_scenario = "normal"
        self.elapsed_seconds = 0.0

    def destroy(self) -> None:
        """
        Public cleanup method.
        """

        self.clear_scenario()
        self.destination = None

        print("[M4] Scenario manager destroyed.")

    def _destroy_actors(self) -> None:
        """
        Safely destroy all actors created by this manager.
        """

        for actor in list(self.actors):
            try:
                if actor is not None and actor.is_alive:
                    actor.destroy()
            except Exception as exc:
                print(
                    f"[M4] Warning: could not destroy scenario actor: {exc}"
                )

        self.actors.clear()

    # ============================================================
    # HELPERS
    # ============================================================

    def _find_actor(self, actor_id: Optional[int]) -> Optional[carla.Actor]:
        if actor_id is None:
            return None

        try:
            actor = self.world.get_actor(int(actor_id))
            return actor
        except Exception:
            return None

    @staticmethod
    def _location_to_dict(
        location: Optional[carla.Location],
    ) -> Optional[Dict[str, float]]:
        if location is None:
            return None

        return {
            "x": float(location.x),
            "y": float(location.y),
            "z": float(location.z),
        }


# ================================================================
# DIRECT TEST
# ================================================================

if __name__ == "__main__":
    print("Supported scenarios:")
    print(ScenarioManager.list_scenarios())