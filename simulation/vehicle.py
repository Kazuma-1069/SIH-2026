import carla


class VehicleManager:
    """Manages the ego vehicle in the CARLA simulation."""

    def __init__(self, world, vehicle_filter="vehicle.tesla.model3"):
        if world is None:
            raise ValueError("A valid CARLA world is required.")

        self.world = world
        self.vehicle_filter = vehicle_filter
        self.vehicle = None

    def spawn_ego_vehicle(self, spawn_index=0):
        """
        Spawn the ego vehicle at a CARLA spawn point.

        Returns:
            carla.Vehicle: The spawned ego vehicle.
        """

        if self.vehicle is not None:
            print("Ego vehicle already exists.")
            return self.vehicle

        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter(self.vehicle_filter)

        if not vehicle_blueprints:
            raise RuntimeError(
                f"No CARLA vehicle found for filter: "
                f"{self.vehicle_filter}"
            )

        spawn_points = self.world.get_map().get_spawn_points()

        if not spawn_points:
            raise RuntimeError("No vehicle spawn points available.")

        if spawn_index < 0 or spawn_index >= len(spawn_points):
            raise IndexError(
                f"Invalid spawn index {spawn_index}. "
                f"Available indices: 0-{len(spawn_points) - 1}"
            )

        blueprint = vehicle_blueprints[0]
        spawn_transform = spawn_points[spawn_index]

        self.vehicle = self.world.try_spawn_actor(
            blueprint,
            spawn_transform,
        )

        if self.vehicle is None:
            raise RuntimeError(
                f"Failed to spawn ego vehicle at spawn point "
                f"{spawn_index}."
            )

        print(
            f"Ego vehicle spawned: "
            f"{self.vehicle.type_id}"
        )

        return self.vehicle

    def get_vehicle(self):
        """Return the currently spawned ego vehicle."""

        return self.vehicle

    def get_transform(self):
        """
        Return the ego vehicle's current transform.

        Returns:
            carla.Transform
        """

        self._require_vehicle()
        return self.vehicle.get_transform()

    def get_location(self):
        """
        Return the ego vehicle's current location.

        Returns:
            carla.Location
        """

        self._require_vehicle()
        return self.vehicle.get_location()

    def get_velocity(self):
        """
        Return the ego vehicle's current velocity.

        Returns:
            carla.Vector3D
        """

        self._require_vehicle()
        return self.vehicle.get_velocity()

    def get_speed(self):
        """
        Return the ego vehicle speed in km/h.
        """

        self._require_vehicle()

        velocity = self.vehicle.get_velocity()

        speed_ms = (
            velocity.x ** 2
            + velocity.y ** 2
            + velocity.z ** 2
        ) ** 0.5

        return speed_ms * 3.6

    def apply_control(
        self,
        throttle=0.0,
        steer=0.0,
        brake=0.0,
        reverse=False,
        hand_brake=False,
    ):
        """
        Apply basic vehicle control.

        Args:
            throttle: Throttle value from 0.0 to 1.0.
            steer: Steering value from -1.0 to 1.0.
            brake: Brake value from 0.0 to 1.0.
            reverse: Whether reverse gear is requested.
            hand_brake: Whether the hand brake is enabled.
        """

        self._require_vehicle()

        control = carla.VehicleControl(
            throttle=max(0.0, min(1.0, throttle)),
            steer=max(-1.0, min(1.0, steer)),
            brake=max(0.0, min(1.0, brake)),
            reverse=reverse,
            hand_brake=hand_brake,
        )

        self.vehicle.apply_control(control)

    def stop(self):
        """Stop the ego vehicle safely."""

        self._require_vehicle()

        self.vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=0.0,
            )
        )

    def destroy(self):
        """Destroy the ego vehicle actor."""

        if self.vehicle is not None:
            self.vehicle.destroy()
            self.vehicle = None
            print("Ego vehicle destroyed.")

    def _require_vehicle(self):
        """Ensure an ego vehicle has been spawned."""

        if self.vehicle is None:
            raise RuntimeError(
                "Ego vehicle is not available. "
                "Call spawn_ego_vehicle() first."
            )