class CoordinateAdapter:
    """
    Converts CARLA world coordinates
    into planner grid coordinates.
    """

    def __init__(
        self,
        scale=5.0,
        grid_width=20,
        grid_height=20,
    ):

        self.scale = scale
        self.grid_width = grid_width
        self.grid_height = grid_height


    def world_to_grid(
        self,
        location,
    ):
        """
        CARLA Location:
            x,y,z

        Returns:
            planner grid coordinate
        """

        x = int(
            location[0] / self.scale
        )

        y = int(
            location[1] / self.scale
        )


        x = max(
            0,
            min(
                self.grid_width - 1,
                x
            )
        )

        y = max(
            0,
            min(
                self.grid_height - 1,
                y
            )
        )


        return [
            x,
            y,
        ]


    def grid_to_world(
        self,
        point,
    ):

        return [
            point[0] * self.scale,
            point[1] * self.scale,
        ]