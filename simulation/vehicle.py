import carla
import math


class VehicleManager:
    """
    M4 Ego Vehicle Manager for CARLA 0.9.16.

    Provides the API expected by test_m4.py:
        - spawn_ego_vehicle()
        - set_destination()
        - get_vehicle()
        - get_location()
        - get_speed()
        - get_speed_kmh()
        - get_distance_to_destination()
        - has_reached_destination()
        - apply_control()
        - stop_vehicle()
        - destroy_vehicle()

    Also provides compatibility aliases for newer code.
    """

    def __init__(
        self,
        world,
        vehicle_filter="vehicle.tesla.model3",
    ):
        if world is None:
            raise ValueError(
                "A valid CARLA world is required."
            )

        self.world = world
        self.vehicle_filter = vehicle_filter

        self.vehicle = None
        self.destination = None
        self.spawn_transform = None

        print(
            "[M4] Vehicle manager initialized"
        )

    # =========================================================
    # BLUEPRINT
    # =========================================================

    def _get_vehicle_blueprint(self):
        library = self.world.get_blueprint_library()

        try:
            blueprint = library.find(
                self.vehicle_filter
            )

            if blueprint is not None:
                return blueprint

        except Exception:
            pass

        vehicles = library.filter(
            "vehicle.*"
        )

        for blueprint in vehicles:

            try:
                wheels = blueprint.get_attribute(
                    "number_of_wheels"
                ).as_int()

                if wheels == 4:
                    return blueprint

            except Exception:
                continue

        raise RuntimeError(
            "No suitable four-wheeled vehicle "
            "blueprint found."
        )

    # =========================================================
    # SPAWN EGO VEHICLE
    # =========================================================

    def spawn_ego_vehicle(
        self,
        spawn_index=0,
        spawn_point=None,
        transform=None,
        role_name="hero",
    ):
        """
        Spawn the ego vehicle.

        Compatible arguments:
            spawn_index
            spawn_point
            transform
            role_name
        """

        if self.vehicle is not None:

            try:
                if self.vehicle.is_alive:
                    print(
                        "[M4] Ego vehicle already spawned."
                    )
                    return self.vehicle
            except Exception:
                pass

        blueprint = (
            self._get_vehicle_blueprint()
        )

        # Set role name if supported.
        try:
            blueprint.set_attribute(
                "role_name",
                role_name,
            )
        except Exception:
            pass

        # Determine spawn transform.
        selected_transform = (
            transform
            if transform is not None
            else spawn_point
        )

        if selected_transform is None:

            spawn_points = (
                self.world
                .get_map()
                .get_spawn_points()
            )

            if not spawn_points:
                raise RuntimeError(
                    "No CARLA spawn points available."
                )

            if (
                spawn_index < 0
                or spawn_index >= len(spawn_points)
            ):
                spawn_index = 0

            selected_transform = (
                spawn_points[spawn_index]
            )

        # Make a copy and lift vehicle slightly.
        location = selected_transform.location

        selected_transform = carla.Transform(
            carla.Location(
                x=location.x,
                y=location.y,
                z=location.z + 0.5,
            ),
            selected_transform.rotation,
        )

        # Try requested point first.
        vehicle = self.world.try_spawn_actor(
            blueprint,
            selected_transform,
        )

        # If occupied, try other spawn points.
        if vehicle is None:

            print(
                "[M4] Requested spawn point occupied. "
                "Trying another spawn point..."
            )

            spawn_points = (
                self.world
                .get_map()
                .get_spawn_points()
            )

            for point in spawn_points:

                point = carla.Transform(
                    carla.Location(
                        x=point.location.x,
                        y=point.location.y,
                        z=point.location.z + 0.5,
                    ),
                    point.rotation,
                )

                vehicle = (
                    self.world.try_spawn_actor(
                        blueprint,
                        point,
                    )
                )

                if vehicle is not None:

                    selected_transform = point
                    break

        if vehicle is None:

            raise RuntimeError(
                "Failed to spawn ego vehicle."
            )

        self.vehicle = vehicle
        self.spawn_transform = (
            selected_transform
        )

        self.vehicle.set_simulate_physics(
            True
        )

        try:
            self.vehicle.set_autopilot(
                False
            )
        except Exception:
            pass

        location = (
            self.vehicle.get_location()
        )

        print(
            f"Ego vehicle spawned: "
            f"{self.vehicle.type_id}"
        )

        print(
            f"Spawn location: "
            f"({location.x:.2f}, "
            f"{location.y:.2f}, "
            f"{location.z:.2f})"
        )

        return self.vehicle

    # =========================================================
    # COMPATIBILITY SPAWN ALIAS
    # =========================================================

    def spawn(
        self,
        spawn_index=0,
        transform=None,
    ):
        return self.spawn_ego_vehicle(
            spawn_index=spawn_index,
            transform=transform,
        )

    # =========================================================
    # VEHICLE ACCESS
    # =========================================================

    def get_vehicle(self):
        return self.vehicle

    def is_alive(self):
        if self.vehicle is None:
            return False

        try:
            return self.vehicle.is_alive
        except Exception:
            return False

    # =========================================================
    # DESTINATION
    # =========================================================

    def set_destination(
        self,
        destination,
    ):
        """
        Accept:
            carla.Location
            carla.Transform
            (x, y)
            (x, y, z)
        """

        if destination is None:
            raise ValueError(
                "Destination cannot be None."
            )

        if isinstance(
            destination,
            carla.Transform,
        ):

            destination = (
                destination.location
            )

        elif isinstance(
            destination,
            carla.Location,
        ):

            pass

        elif isinstance(
            destination,
            (tuple, list),
        ):

            if len(destination) == 2:

                destination = carla.Location(
                    x=float(destination[0]),
                    y=float(destination[1]),
                    z=0.6,
                )

            elif len(destination) >= 3:

                destination = carla.Location(
                    x=float(destination[0]),
                    y=float(destination[1]),
                    z=float(destination[2]),
                )

            else:

                raise ValueError(
                    "Destination must contain "
                    "x and y."
                )

        else:

            raise TypeError(
                "Unsupported destination type."
            )

        self.destination = destination

        print(
            f"Destination set: "
            f"({destination.x:.2f}, "
            f"{destination.y:.2f}, "
            f"{destination.z:.2f})"
        )

        return destination

    def get_destination(self):
        return self.destination

    # =========================================================
    # LOCATION
    # =========================================================

    def get_location(self):
        if not self.is_alive():
            return None

        return self.vehicle.get_location()

    def get_transform(self):
        if not self.is_alive():
            return None

        return self.vehicle.get_transform()

    def get_heading(self):
        transform = self.get_transform()

        if transform is None:
            return None

        return float(
            transform.rotation.yaw
        )

    def get_next_waypoint(
        self,
        current_location,
        route_waypoints,
    ):
        """Return the route waypoint closest to the current location."""

        if not route_waypoints:
            return None

        def point_xy(point):
            if hasattr(point, "transform"):
                point = point.transform.location

            if hasattr(point, "location"):
                point = point.location

            if hasattr(point, "x"):
                return point.x, point.y

            return point[0], point[1]

        current_x, current_y = point_xy(
            current_location
        )

        return min(
            route_waypoints,
            key=lambda waypoint: (
                (
                    point_xy(waypoint)[0]
                    - current_x
                ) ** 2
                +
                (
                    point_xy(waypoint)[1]
                    - current_y
                ) ** 2
            ),
        )

    def generate_route(
        self,
        start_location,
        destination_location,
    ):
        """Generate a route by following CARLA road waypoints."""

        carla_map = self.world.get_map()

        current_waypoint = carla_map.get_waypoint(
            start_location
        )

        destination_waypoint = carla_map.get_waypoint(
            destination_location
        )

        if (
            current_waypoint is None
            or destination_waypoint is None
        ):
            return []

        destination = (
            destination_waypoint.transform.location
        )
        route = []
        visited = set()

        for _ in range(500):
            location = current_waypoint.transform.location
            route.append(location)

            distance = math.sqrt(
                (location.x - destination.x) ** 2
                +
                (location.y - destination.y) ** 2
            )

            if distance <= 3.0:
                break

            waypoint_id = getattr(
                current_waypoint,
                "id",
                id(current_waypoint),
            )
            visited.add(waypoint_id)

            next_waypoints = current_waypoint.next(
                2.0
            )

            if not next_waypoints:
                break

            unvisited = [
                waypoint
                for waypoint in next_waypoints
                if getattr(
                    waypoint,
                    "id",
                    id(waypoint),
                ) not in visited
            ]

            candidates = (
                unvisited
                if unvisited
                else next_waypoints
            )

            current_waypoint = min(
                candidates,
                key=lambda waypoint: (
                    (
                        waypoint.transform.location.x
                        - destination.x
                    ) ** 2
                    +
                    (
                        waypoint.transform.location.y
                        - destination.y
                    ) ** 2
                ),
            )

        return route

    # =========================================================
    # VELOCITY
    # =========================================================

    def get_velocity(self):
        if not self.is_alive():
            return None

        return self.vehicle.get_velocity()

    def get_speed_mps(self):
        velocity = self.get_velocity()

        if velocity is None:
            return 0.0

        return math.sqrt(
            velocity.x ** 2
            + velocity.y ** 2
            + velocity.z ** 2
        )

    def get_speed_kmh(self):
        return (
            self.get_speed_mps()
            * 3.6
        )

    # Existing-test compatibility.
    def get_speed(self):
        return self.get_speed_kmh()

    # =========================================================
    # DISTANCE
    # =========================================================

    def get_distance_to_destination(self):

        if (
            self.destination is None
            or not self.is_alive()
        ):
            return None

        location = (
            self.vehicle.get_location()
        )

        dx = (
            location.x
            - self.destination.x
        )

        dy = (
            location.y
            - self.destination.y
        )

        dz = (
            location.z
            - self.destination.z
        )

        return math.sqrt(
            dx * dx
            + dy * dy
            + dz * dz
        )

    # =========================================================
    # DESTINATION CHECK
    # =========================================================

    def has_reached_destination(
        self,
        threshold=3.0,
    ):

        distance = (
            self.get_distance_to_destination()
        )

        if distance is None:
            return False

        return distance <= threshold

    def destination_reached(
        self,
        threshold=3.0,
    ):
        return self.has_reached_destination(
            threshold
        )

    # =========================================================
    # CONTROL
    # =========================================================

    def apply_control(
        self,
        throttle=0.0,
        steer=0.0,
        brake=0.0,
        hand_brake=False,
        reverse=False,
    ):

        if not self.is_alive():

            raise RuntimeError(
                "Ego vehicle is not available."
            )

        control = carla.VehicleControl()

        control.throttle = max(
            0.0,
            min(
                1.0,
                float(throttle),
            ),
        )

        control.steer = max(
            -1.0,
            min(
                1.0,
                float(steer),
            ),
        )

        control.brake = max(
            0.0,
            min(
                1.0,
                float(brake),
            ),
        )

        control.hand_brake = (
            bool(hand_brake)
        )

        control.reverse = (
            bool(reverse)
        )

        print(
            "M5 CONTROL SENT:",
            control.throttle,
            control.steer,
            control.brake,
        )

        self.vehicle.apply_control(
            control
        )

        print(
            "CARLA CONTROL APPLIED"
        )

        return control

    # =========================================================
    # STOP VEHICLE
    # =========================================================

    def stop_vehicle(self):

        if not self.is_alive():
            return

        print(
            "[M4] Applying brake..."
        )

        try:

            self.vehicle.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    steer=0.0,
                    brake=1.0,
                    hand_brake=True,
                )
            )

        except Exception as exc:

            print(
                f"[M4] Stop warning: {exc}"
            )

    # Compatibility alias.
    def stop(self):
        self.stop_vehicle()

    # =========================================================
    # STATE
    # =========================================================

    def get_state(self):

        if not self.is_alive():

            return {
                "available": False,
                "vehicle_id": None,
                "location": None,
                "rotation": None,
                "velocity": None,
                "speed_mps": 0.0,
                "speed_kmh": 0.0,
                "destination": None,
                "distance_to_destination": None,
                "destination_reached": False,
            }

        transform = (
            self.vehicle.get_transform()
        )

        velocity = (
            self.vehicle.get_velocity()
        )

        distance = (
            self.get_distance_to_destination()
        )

        return {
            "available": True,

            "vehicle_id": self.vehicle.id,

            "type_id": self.vehicle.type_id,

            "location": {
                "x": transform.location.x,
                "y": transform.location.y,
                "z": transform.location.z,
            },

            "rotation": {
                "pitch": transform.rotation.pitch,
                "yaw": transform.rotation.yaw,
                "roll": transform.rotation.roll,
            },

            "velocity": {
                "x": velocity.x,
                "y": velocity.y,
                "z": velocity.z,
            },

            "speed_mps": (
                self.get_speed_mps()
            ),

            "speed_kmh": (
                self.get_speed_kmh()
            ),

            "destination": (
                {
                    "x": self.destination.x,
                    "y": self.destination.y,
                    "z": self.destination.z,
                }
                if self.destination is not None
                else None
            ),

            "distance_to_destination": (
                distance
            ),

            "destination_reached": (
                self.has_reached_destination()
            ),
        }

    # =========================================================
    # DESTROY
    # =========================================================
    # =========================================================
    # SIMULATION STATE
    # =========================================================

    def get_simulation_state(self):
        """
        Return the vehicle state in the format expected
        by the M4 simulation test and integration pipeline.
        """

        if not self.is_alive():
            return {
                "vehicle": None,
                "location": None,
                "transform": None,
                "velocity": None,
                "speed": 0.0,
                "speed_mps": 0.0,
                "speed_kmh": 0.0,
                "destination": None,
                "distance_to_destination": None,
                "destination_reached": False,
            }

        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()

        speed_mps = math.sqrt(
            velocity.x ** 2
            + velocity.y ** 2
            + velocity.z ** 2
        )

        speed_kmh = speed_mps * 3.6

        distance = self.get_distance_to_destination()

        return {
            "vehicle": self.vehicle,

            "location": {
                "x": transform.location.x,
                "y": transform.location.y,
                "z": transform.location.z,
            },

            "transform": transform,

            "velocity": {
                "x": velocity.x,
                "y": velocity.y,
                "z": velocity.z,
            },

            "speed": speed_kmh,
            "speed_mps": speed_mps,
            "speed_kmh": speed_kmh,

            "destination": (
                {
                    "x": self.destination.x,
                    "y": self.destination.y,
                    "z": self.destination.z,
                }
                if self.destination is not None
                else None
            ),

            "distance_to_destination": distance,

            "destination_reached": (
                self.has_reached_destination()
            ),
        }
    def destroy_vehicle(self):

        if self.vehicle is None:
            return

        try:

            if self.vehicle.is_alive:

                try:
                    self.vehicle.apply_control(
                        carla.VehicleControl(
                            throttle=0.0,
                            brake=1.0,
                            steer=0.0,
                        )
                    )
                except Exception:
                    pass

                self.vehicle.destroy()

                print(
                    "Ego vehicle destroyed."
                )

        except Exception as exc:

            print(
                f"[M4] Vehicle destroy warning: "
                f"{exc}"
            )

        finally:

            self.vehicle = None

    # Compatibility aliases.
    def destroy(self):
        self.destroy_vehicle()

    def cleanup(self):
        self.destroy_vehicle()


# =============================================================
# COMPATIBILITY ALIASES
# =============================================================

Vehicle = VehicleManager
EgoVehicle = VehicleManager