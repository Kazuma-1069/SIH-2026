import carla


class WorldManager:
    """Manages the CARLA simulation world for the SIH system."""

    def __init__(self, client):
        if client is None:
            raise ValueError("A valid CARLA client is required.")

        self.client = client
        self.world = client.get_world()

    def get_world(self):
        """Return the current CARLA world."""
        return self.world

    def get_map_name(self):
        """Return the name of the currently loaded map."""
        return self.world.get_map().name

    def load_map(self, map_name):
        """
        Load a CARLA map.

        Example:
            manager.load_map("Town03")
        """

        if not map_name:
            raise ValueError("map_name cannot be empty.")

        print(f"Loading CARLA map: {map_name}")

        self.world = self.client.load_world(map_name)

        print(f"Map loaded: {self.world.get_map().name}")

        return self.world

    def set_weather(self, weather):
        """Apply a CARLA weather preset/object."""

        if weather is None:
            raise ValueError("Weather cannot be None.")

        self.world.set_weather(weather)

    def get_weather(self):
        """Return the current CARLA weather."""
        return self.world.get_weather()

    def set_synchronous_mode(self, enabled=True, fixed_delta_seconds=0.05):
        """
        Configure CARLA synchronous simulation mode.

        Synchronous mode gives the perception, planning and
        visualization modules predictable simulation frames.
        """

        settings = self.world.get_settings()

        settings.synchronous_mode = enabled

        if enabled:
            if fixed_delta_seconds <= 0:
                raise ValueError(
                    "fixed_delta_seconds must be greater than zero."
                )

            settings.fixed_delta_seconds = fixed_delta_seconds
        else:
            settings.fixed_delta_seconds = None

        self.world.apply_settings(settings)

    def tick(self):
        """
        Advance the simulation by one frame.

        Use this when synchronous mode is enabled.
        """
        return self.world.tick()

    def wait_for_tick(self):
        """Wait for the next simulation frame."""
        return self.world.wait_for_tick()

    def reset_world(self):
        """
        Reload the currently active map.

        This provides a simple simulation reset.
        """

        current_map = self.world.get_map().name

        print(f"Resetting CARLA world: {current_map}")

        self.world = self.client.reload_world()

        print("CARLA world reset.")

        return self.world

    def get_spawn_points(self):
        """Return available vehicle spawn points."""

        return self.world.get_map().get_spawn_points()