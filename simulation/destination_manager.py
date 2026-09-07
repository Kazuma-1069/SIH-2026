class DestinationManager:
    """
    Handles start and destination selection
    from CARLA map spawn points.
    """

    def __init__(self, world):
        self.world = world

        self.spawn_points = (
            world.get_map().get_spawn_points()
        )

        self.start_point = None
        self.destination_point = None


    def get_available_points(self):
        return self.spawn_points


    def set_start(self, index):

        self.start_point = (
            self.spawn_points[index]
        )

        return self.start_point


    def set_destination(self, index):
        import math

        candidate = self.spawn_points[index]
        if self.start_point is not None:
            s_loc = self.start_point.location
            c_loc = candidate.location
            dist = math.sqrt(
                (c_loc.x - s_loc.x) ** 2
                + (c_loc.y - s_loc.y) ** 2
                + (c_loc.z - s_loc.z) ** 2
            )
            if dist < 15.0:
                print(
                    f"[DestinationManager] Warning: destination index {index} is too close ({dist:.1f}m) to start. "
                    "Selecting an alternative distant spawn point."
                )
                for alt_idx, sp in enumerate(self.spawn_points):
                    alt_dist = math.sqrt(
                        (sp.location.x - s_loc.x) ** 2
                        + (sp.location.y - s_loc.y) ** 2
                        + (sp.location.z - s_loc.z) ** 2
                    )
                    if alt_dist >= 50.0:
                        candidate = sp
                        break

        self.destination_point = candidate
        return self.destination_point


    def get_start_location(self):

        if self.start_point is None:
            return None

        return self.start_point.location


    def get_destination_location(self):

        if self.destination_point is None:
            return None

        return self.destination_point.location