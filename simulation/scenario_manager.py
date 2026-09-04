"""
CARLA scenario management for SIH 2026.

Supported scenarios:
    - normal
    - static_obstacle
    - dynamic_obstacle
    - pothole
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
        "static_obstacle",
        "dynamic_obstacle",
        "pothole",
        "combined",
    ]

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

    def set_scenario(self, scenario_name: str) -> bool:
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

        self.active_scenario = scenario_name

        if scenario_name == "normal":
            self._setup_normal()

        elif scenario_name == "static_obstacle":
            self._setup_static_obstacle()

        elif scenario_name == "dynamic_obstacle":
            self._setup_dynamic_obstacle()

        elif scenario_name == "pothole":
            self._setup_pothole()

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

        for hazard in self.hazards:
            if hazard.get("type") != "dynamic_obstacle":
                continue

            actor_id = hazard.get("id")
            actor = self._find_actor(actor_id)

            if actor is None:
                hazard["active"] = False
                continue

            speed = float(hazard.get("speed", 4.0))

            transform = actor.get_transform()
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

    # ============================================================
    # POTHOLE
    # ============================================================

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