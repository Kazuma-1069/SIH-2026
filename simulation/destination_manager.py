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

        self.destination_point = (
            self.spawn_points[index]
        )

        return self.destination_point


    def get_start_location(self):

        if self.start_point is None:
            return None

        return self.start_point.location


    def get_destination_location(self):

        if self.destination_point is None:
            return None

        return self.destination_point.location