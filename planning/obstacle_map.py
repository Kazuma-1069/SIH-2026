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


    def is_path_blocked(
        self,
        path
    ):

        # An empty path has no waypoint that can be blocked.
        if not path:
            return False

        # Check each waypoint against the occupancy grid.
        for x, y in path:

            if self.is_occupied(
                x,
                y
            ):

                return True

        return False



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
                    [0, 0]
                )
                radius = obj.get(
                    "radius"
                )
                if radius is None:
                    radius = 1.0
                obstacle_type = obj.get(
                    "class_name",
                    obj.get("type", obj.get("hazard_type", "unknown"))
                )
                vehicle_relative = obj.get(
                    "vehicle_relative",
                    False
                )
                grid_position = obj.get(
                    "grid_position"
                )

            else:
                position = getattr(
                    obj,
                    "position",
                    [0, 0]
                )
                radius = getattr(
                    obj,
                    "radius",
                    1.0
                )
                if radius is None:
                    radius = 1.0
                obstacle_type = getattr(
                    obj,
                    "class_name",
                    getattr(obj, "hazard_type", "unknown")
                )
                vehicle_relative = getattr(
                    obj,
                    "vehicle_relative",
                    False
                )
                grid_position = getattr(
                    obj,
                    "grid_position",
                    None
                )

            map_position = (
                grid_position
                if grid_position is not None
                else position
            )

            # Directly add obstacle preserving its coordinate system flag.
            self.add_obstacle(
                position,
                radius,
                obstacle_type=obstacle_type,
                vehicle_relative=vehicle_relative,
                grid_position=map_position,
            )

    # =====================================================
    # ADD OBSTACLE
    # =====================================================

    def add_obstacle(
        self,
        position,
        radius=2.0,
        obstacle_type="unknown",
        vehicle_relative=False,
        grid_position=None,
    ):
        if radius is None or radius < 0:
            radius = 1.0
        radius = float(radius)

        if position is None or len(position) < 2:
            return

        grid_pos = grid_position if grid_position is not None else position

        self.obstacles.append(
            {
                "position": position,
                "grid_position": grid_pos,
                "radius": radius,
                "type": obstacle_type,
                "vehicle_relative": vehicle_relative,
            }
        )

        cx = int(
            grid_pos[0]
        )

        cy = int(
            grid_pos[1]
        )

        cell_radius = max(0, int(round(radius / 5.0))) if vehicle_relative else int(radius)

        for dx in range(
            -cell_radius,
            cell_radius + 1
        ):

            for dy in range(
                -cell_radius,
                cell_radius + 1
            ):

                if (
                    dx*dx + dy*dy
                    <= cell_radius*cell_radius
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
        ego_position,
        is_ego=True,
    ):

        if not self.obstacles:

            return float("inf")

        nearest = float("inf")

        for obstacle in self.obstacles:
            position = obstacle[
                "position"
            ]

            if obstacle.get("vehicle_relative", False) and is_ego:
                distance = math.sqrt(
                    position[0] ** 2 + position[1] ** 2
                )
            else:
                pos = obstacle.get("grid_position", position)
                grid_dist = math.sqrt(
                    (
                        ego_position[0]
                        -
                        pos[0]
                    ) ** 2
                    +
                    (
                        ego_position[1]
                        -
                        pos[1]
                    ) ** 2
                )
                scale = 5.0 if obstacle.get("vehicle_relative", False) else 1.0
                distance = grid_dist * scale

            distance -= float(obstacle.get("radius") or 0.0)

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