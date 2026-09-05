"""
SIH-2026 Integration Pipeline

Full autonomous driving data flow:

CARLA/M4
   ↓
M2 Perception
   ↓
M0 Navigation State
   ↓
M1 Planning
   ↓
M5 Vehicle Control
   ↓
CARLA Vehicle
   ↓
M3 Visualization
"""

from integration.data_adapter import (
    perception_to_planning_input
)

from planning.coordinate_adapter import (
    CoordinateAdapter
)

import math


class IntegrationPipeline:


    def __init__(
        self,
        perception_pipeline,
        planner,
        controller=None,
        dashboard=None,
        vehicle=None,
        destination=None,
    ):

        # M2
        self.perception_pipeline = (
            perception_pipeline
        )

        # M1
        self.planner = planner

        # M5
        self.controller = controller

        # M3
        self.dashboard = dashboard

        # M4 VehicleManager
        self.vehicle = vehicle

        # M0 destination
        self.destination = destination

        # CARLA -> Planner grid
        self.coordinate_adapter = (
            CoordinateAdapter()
        )

        self.road_waypoints = None


    def _build_road_route(
        self,
        start_location,
    ):

        if (
            self.vehicle is None
            or self.destination is None
        ):

            return []

        world = getattr(
            self.vehicle,
            "world",
            None
        )

        if world is None:
            return []

        carla_map = world.get_map()

        start_waypoint = carla_map.get_waypoint(
            start_location
        )

        destination_waypoint = carla_map.get_waypoint(
            self.destination
        )

        if (
            start_waypoint is None
            or destination_waypoint is None
        ):

            return []

        destination_location = (
            destination_waypoint.transform.location
        )

        route = []
        current_waypoint = start_waypoint
        visited = set()

        for _ in range(300):

            current_location = (
                current_waypoint.transform.location
            )

            route.append(
                [
                    current_location.x,
                    current_location.y,
                ]
            )

            distance_to_destination = math.sqrt(
                (
                    current_location.x
                    - destination_location.x
                ) ** 2
                +
                (
                    current_location.y
                    - destination_location.y
                ) ** 2
            )

            if distance_to_destination <= 3.0:
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
                        - destination_location.x
                    ) ** 2
                    +
                    (
                        waypoint.transform.location.y
                        - destination_location.y
                    ) ** 2
                ),
            )

        return route


    def process_frame(
        self,
        frame,
        show=False,
        save_path=None,
    ):


        # ==========================
        # M2 PERCEPTION
        # ==========================

        perception_output = (
            self.perception_pipeline
            .process_frame(frame)
        )


        # ==========================
        # M2 -> M1
        # ==========================

        planning_input = (
            perception_to_planning_input(
                perception_output
            )
        )


        # ==========================
        # M0 NAVIGATION
        # ==========================

        ego_position = None
        vehicle_location = None
        vehicle_heading = None


        if self.vehicle is not None:

            location = (
                self.vehicle.get_location()
            )

            if location is not None:

                vehicle_location = [
                    location.x,
                    location.y,
                ]

                transform = self.vehicle.get_transform()

                if transform is not None:
                    vehicle_heading = (
                        transform.rotation.yaw
                    )

                print(
                    "CARLA VEHICLE LOCATION:",
                    location.x,
                    location.y,
                    location.z,
                )

                ego_position = (
                    self.coordinate_adapter
                    .world_to_grid(
                        [
                            location.x,
                            location.y,
                        ]
                    )
                )

                planning_input[
                    "ego_position"
                ] = ego_position

                if self.road_waypoints is None:
                    route = []

                    if hasattr(
                        self.vehicle,
                        "generate_route",
                    ) and self.destination is not None:
                        route = (
                            self.vehicle.generate_route(
                                location,
                                self.destination,
                            )
                        ) or []

                    if not route:
                        route = (
                            self._build_road_route(
                                location
                            )
                        )

                    self.road_waypoints = [
                        [
                            point.x,
                            point.y,
                        ]
                        if hasattr(point, "x")
                        else list(point)
                        for point in route
                    ]

                route = self.road_waypoints or []

                print(
                    "ACTIVE ROUTE WAYPOINTS:",
                    len(route),
                )

                if self.road_waypoints:
                    planning_input[
                        "route_waypoints"
                    ] = route

                    planning_input[
                        "current_path"
                    ] = [
                        self.coordinate_adapter
                        .world_to_grid(
                            point
                        ) for point in route
                    ]



        # Destination -> planner goal

        if self.destination is not None:

            planning_input[
                "goal"
            ] = (
                self.coordinate_adapter
                .world_to_grid(
                    [
                        self.destination.x,
                        self.destination.y,
                    ]
                )
            )



        # ==========================
        # M0 DESTINATION CHECK
        # ==========================

        destination_reached = False


        if self.vehicle is not None:

            destination_reached = (
                self.vehicle
                .has_reached_destination()
            )



        # ==========================
        # M1 PLANNING
        # ==========================

        planning_output = (
            self.planner.plan(
                planning_input
            )
        )

        if (
            self.road_waypoints
            and planning_output.get(
                "path_safe",
                False
            )
            and planning_output.get(
                "action"
            ) != "STOP"
        ):

            planning_output[
                "waypoints"
            ] = self.road_waypoints
        print(
            "\n========== PLANNER DEBUG =========="
        )

        print(
            "PLANNING INPUT:"
        )

        print(
            planning_input
        )

        print(
            "PLANNING OUTPUT:"
        )

        print(
            planning_output
        )

        print(
            "=================================="
        )

        planning_output[
            "destination_reached"
        ] = destination_reached



        # ==========================
        # M5 CONTROL
        # ==========================

        control_command = None


        if destination_reached:

            control_command = {

                "throttle": 0.0,

                "steer": 0.0,

                "brake": 1.0,

            }


        elif self.controller is not None:


            control_command = (
                self.controller
                .compute_control(
                    planning_output,
                    vehicle_location,
                    vehicle_heading,
                )
            )



        # ==========================
        # M5 -> M4
        # ==========================

        if (
            self.vehicle is not None
            and control_command is not None
        ):


            self.vehicle.apply_control(

                throttle=(
                    control_command[
                        "throttle"
                    ]
                ),

                steer=(
                    control_command[
                        "steer"
                    ]
                ),

                brake=(
                    control_command[
                        "brake"
                    ]
                ),

            )



        # ==========================
        # M3 VISUALIZATION
        # ==========================

        if self.dashboard is not None:


            self.dashboard.render(

                perception_output=(
                    perception_output
                ),

                planning_output=(
                    planning_output
                ),

                control_output=(
                    control_command
                ),

                camera_frame=frame,

                show=show,

                save_path=save_path,

            )



        # ==========================
        # M6 OUTPUT
        # ==========================

        return (

            perception_output,

            planning_output,

            control_command,

        )