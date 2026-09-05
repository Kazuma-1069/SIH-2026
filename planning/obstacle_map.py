"""
SIH-2026 Obstacle Map

Compatible with:
- Planner
- A*
- Dijkstra
- Bubble Shield
- Safety Checker
- Perception object updates
"""

import math


class ObstacleMap:


    def __init__(
        self,
        width=100,
        height=100
    ):

        self.width = width
        self.height = height


        self.grid = [
            [
                0
                for _ in range(width)
            ]
            for _ in range(height)
        ]


        self.obstacles = []



    # =====================================================
    # GRID FUNCTIONS
    # =====================================================


    def clear_grid(self):

        for y in range(self.height):

            for x in range(self.width):

                self.grid[y][x] = 0



    def set_obstacle(
        self,
        x,
        y=None,
        value=1
    ):

        if isinstance(x, tuple):

            if y is not None:
                value = y

            x, y = x

        if (
            0 <= x < self.width
            and
            0 <= y < self.height
        ):

            self.grid[y][x] = value



    def is_obstacle(
        self,
        x,
        y
    ):

        if (
            0 <= x < self.width
            and
            0 <= y < self.height
        ):

            return self.grid[y][x] == 1


        return True


    def is_occupied(
        self,
        x,
        y
    ):

        return self.is_obstacle(
            x,
            y
        )



    # =====================================================
    # OBJECT UPDATES FROM PERCEPTION
    # =====================================================


    def update_from_objects(
        self,
        objects
    ):

        """
        Update obstacle map from M2 perception objects.

        Expected:

        [
            {
                "position":[x,y],
                "radius":2
            }
        ]
        """


        self.obstacles.clear()

        self.clear_grid()


        if objects is None:
            return



        for obj in objects:


            if isinstance(
                obj,
                dict
            ):

                position = obj.get(
                    "position",
                    [0,0]
                )


                radius = obj.get(
                    "radius",
                    2
                )


            else:

                position = getattr(
                    obj,
                    "position",
                    [0,0]
                )


                radius = 2



            self.add_obstacle(
                position,
                radius
            )



    # =====================================================
    # ADD OBSTACLE
    # =====================================================


    def add_obstacle(
        self,
        position,
        radius=2.0,
        obstacle_type="unknown"
    ):


        self.obstacles.append(
            {
                "position": position,
                "radius": radius,
                "type": obstacle_type
            }
        )



        cx = int(
            position[0]
        )

        cy = int(
            position[1]
        )


        for dx in range(
            -int(radius),
            int(radius)+1
        ):

            for dy in range(
                -int(radius),
                int(radius)+1
            ):

                if (
                    dx*dx + dy*dy
                    <= radius*radius
                ):

                    self.set_obstacle(
                        cx + dx,
                        cy + dy
                    )



    def update(
        self,
        obstacles
    ):

        self.update_from_objects(
            obstacles
        )



    # =====================================================
    # BUBBLE SHIELD
    # =====================================================


    def distance_to_nearest_obstacle(
        self,
        ego_position
    ):


        if not self.obstacles:

            return float("inf")



        nearest = float("inf")



        for obstacle in self.obstacles:


            position = obstacle[
                "position"
            ]


            distance = math.sqrt(

                (
                    ego_position[0]
                    -
                    position[0]
                ) ** 2

                +

                (
                    ego_position[1]
                    -
                    position[1]
                ) ** 2

            )


            distance -= obstacle[
                "radius"
            ]


            nearest = min(
                nearest,
                distance
            )



        return max(
            nearest,
            0.0
        )



    def is_collision(
        self,
        position,
        safety_distance=2.0
    ):

        return (

            self.distance_to_nearest_obstacle(
                position
            )
            <
            safety_distance

        )



    # =====================================================
    # ACCESS
    # =====================================================


    def get_obstacles(
        self
    ):

        return self.obstacles



    def __len__(
        self
    ):

        return len(
            self.obstacles
        )