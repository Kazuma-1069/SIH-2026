class CoordinateAdapter:
    """
    Converts CARLA world coordinates
    into planner grid coordinates.

    Uses a route origin so large CARLA map coordinates are mapped into
    a local grid window instead of being clamped near (0, 0).
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
        self.origin = None


    def set_origin(
        self,
        location,
    ):
        self.origin = [
            float(location[0]),
            float(location[1]),
        ]


    def _ensure_origin(
        self,
        location,
    ):
        if self.origin is None:
            self.set_origin(location)


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

        self._ensure_origin(location)

        x = int(
            (location[0] - self.origin[0])
            / self.scale
        )

        y = int(
            (location[1] - self.origin[1])
            / self.scale
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

        if self.origin is None:
            return [
                point[0] * self.scale,
                point[1] * self.scale,
            ]

        return [
            self.origin[0]
            + point[0] * self.scale,
            self.origin[1]
            + point[1] * self.scale,
        ]