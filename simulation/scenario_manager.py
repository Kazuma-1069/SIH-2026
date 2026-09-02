import carla


class ScenarioManager:
    """Creates and manages simulation scenarios in CARLA."""

    def __init__(self, world):
        if world is None:
            raise ValueError("A valid CARLA world is required.")

        self.world = world
        self.actors = []

    def set_weather(self, weather):
        """Apply a CARLA weather configuration."""

        if weather is None:
            raise ValueError("Weather cannot be None.")

        self.world.set_weather(weather)

    def get_weather(self):
        """Return the current weather configuration."""

        return self.world.get_weather()

    def spawn_vehicle(
        self,
        vehicle_filter="vehicle.tesla.model3",
        spawn_index=0,
        autopilot=False,
    ):
        """
        Spawn a vehicle for the current scenario.

        Args:
            vehicle_filter: CARLA vehicle blueprint filter.
            spawn_index: Index of the map spawn point.
            autopilot: Enable CARLA autopilot if True.

        Returns:
            Spawned CARLA vehicle actor.
        """

        blueprint_library = self.world.get_blueprint_library()
        blueprints = blueprint_library.filter(vehicle_filter)

        if not blueprints:
            raise RuntimeError(
                f"No vehicle blueprint found for: {vehicle_filter}"
            )

        spawn_points = self.world.get_map().get_spawn_points()

        if not spawn_points:
            raise RuntimeError("No spawn points available.")

        if spawn_index < 0 or spawn_index >= len(spawn_points):
            raise IndexError(
                f"Invalid spawn index {spawn_index}. "
                f"Available indices: 0-{len(spawn_points) - 1}"
            )

        vehicle = self.world.try_spawn_actor(
            blueprints[0],
            spawn_points[spawn_index],
        )

        if vehicle is None:
            raise RuntimeError(
                f"Failed to spawn vehicle at spawn point "
                f"{spawn_index}."
            )

        if autopilot:
            vehicle.set_autopilot(True)

        self.actors.append(vehicle)

        return vehicle

    def spawn_walker(
        self,
        spawn_index=0,
    ):
        """Spawn a pedestrian in the current scenario."""

        blueprint_library = self.world.get_blueprint_library()

        walker_blueprints = blueprint_library.filter(
            "walker.pedestrian.*"
        )

        if not walker_blueprints:
            raise RuntimeError("No pedestrian blueprints available.")

        spawn_points = self.world.get_map().get_spawn_points()

        if not spawn_points:
            raise RuntimeError("No spawn points available.")

        if spawn_index < 0 or spawn_index >= len(spawn_points):
            raise IndexError(
                f"Invalid spawn index {spawn_index}. "
                f"Available indices: 0-{len(spawn_points) - 1}"
            )

        transform = spawn_points[spawn_index]

        walker = self.world.try_spawn_actor(
            walker_blueprints[0],
            transform,
        )

        if walker is None:
            raise RuntimeError("Failed to spawn pedestrian.")

        self.actors.append(walker)

        return walker

    def get_actors(self):
        """Return actors created by this scenario manager."""

        return list(self.actors)

    def clear_scenario(self):
        """Destroy all actors created by this scenario."""

        for actor in self.actors:
            if actor is not None:
                try:
                    actor.destroy()
                except Exception:
                    pass

        self.actors.clear()

    def reset(self):
        """Clear actors and reset the scenario state."""

        self.clear_scenario()

    def create_basic_scenario(
        self,
        ego_vehicle_filter="vehicle.tesla.model3",
        ego_spawn_index=0,
        traffic_vehicle_count=0,
    ):
        """
        Create a basic scenario with an ego vehicle and optional traffic.

        Returns:
            Dictionary containing the created actors.
        """

        self.clear_scenario()

        ego_vehicle = self.spawn_vehicle(
            vehicle_filter=ego_vehicle_filter,
            spawn_index=ego_spawn_index,
            autopilot=False,
        )

        traffic_vehicles = []

        for index in range(traffic_vehicle_count):
            try:
                traffic_vehicle = self.spawn_vehicle(
                    vehicle_filter="vehicle.*",
                    spawn_index=ego_spawn_index + index + 1,
                    autopilot=True,
                )

                traffic_vehicles.append(traffic_vehicle)

            except (IndexError, RuntimeError):
                break

        return {
            "ego_vehicle": ego_vehicle,
            "traffic_vehicles": traffic_vehicles,
        }