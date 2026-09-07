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

from integration.simulation_hazard_bridge import (
    bridge_scenario_hazards,
)

from planning.coordinate_adapter import (
    CoordinateAdapter
)

import importlib
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
        scenario_manager=None,
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

        # M4 ScenarioManager (simulation hazard ground truth)
        self.scenario_manager = scenario_manager

        # M0 destination
        self.destination = destination

        # CARLA -> Planner grid
        self.coordinate_adapter = (
            CoordinateAdapter()
        )

        self.road_waypoints = None


    def _supplement_simulation_hazards(
        self,
        perception_output,
    ):
        """
        Merge M4 ScenarioManager hazards into M2 perception output.

        YOLO detections are preserved; simulation hazards supplement them.
        """

        if (
            self.scenario_manager is None
            or self.vehicle is None
        ):
            return perception_output

        location = self.vehicle.get_location()
        transform = self.vehicle.get_transform()

        if location is None or transform is None:
            return perception_output

        bridged_hazards = bridge_scenario_hazards(
            self.scenario_manager.get_hazards(),
            ego_x=location.x,
            ego_y=location.y,
            ego_yaw_deg=transform.rotation.yaw,
        )

        if not bridged_hazards:
            return perception_output

        # Merge with existing perception hazards
        perception_output.hazards = (
            list(perception_output.hazards)
            + bridged_hazards
        )

        return perception_output


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

        try:
            route_planner_module = importlib.import_module(
                "agents.navigation.global_route_planner"
            )
            GlobalRoutePlanner = (
                route_planner_module.GlobalRoutePlanner
            )

            route_planner = GlobalRoutePlanner(
                carla_map,
                sampling_resolution=2.0,
            )
            traced_route = route_planner.trace_route(
                start_location,
                self.destination,
            )
            route_locations = [
                [waypoint.transform.location.x,
                 waypoint.transform.location.y]
                for waypoint, _ in traced_route
            ]
            if route_locations:
                return route_locations
        except Exception as exc:
            print(f"[M0] GlobalRoutePlanner unavailable: {exc}")

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

        perception_output = (
            self._supplement_simulation_hazards(
                perception_output
            )
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

                    # Anchor origin so ego vehicle is represented inside the grid
                    # with adequate forward, lateral, and rear margin rather than
                    # clamped to boundaries when driving in negative world directions.
                    yaw_deg = vehicle_heading if vehicle_heading is not None else 0.0
                    yaw_rad = math.radians(yaw_deg)
                    forward_x = math.cos(yaw_rad)
                    forward_y = math.sin(yaw_rad)
                    right_x = -math.sin(yaw_rad)
                    right_y = math.cos(yaw_rad)

                    pts_x = [location.x] + [p[0] for p in self.road_waypoints[:20]]
                    pts_y = [location.y] + [p[1] for p in self.road_waypoints[:20]]
                    pts_x.extend([
                        location.x + 35.0 * forward_x,
                        location.x - 15.0 * forward_x,
                        location.x - 25.0 * right_x,
                        location.x + 25.0 * right_x,
                    ])
                    pts_y.extend([
                        location.y + 35.0 * forward_y,
                        location.y - 15.0 * forward_y,
                        location.y - 20.0 * right_y,
                        location.y + 20.0 * right_y,
                    ])

                    origin_x = min(pts_x)
                    origin_y = min(pts_y)

                    self.coordinate_adapter.set_origin(
                        [
                            origin_x,
                            origin_y,
                        ]
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

                planning_input[
                    "start"
                ] = ego_position

                route = self.road_waypoints or []

                print(
                    "ACTIVE ROUTE WAYPOINTS:",
                    len(route),
                )

                if self.road_waypoints:
                    planning_input[
                        "route_waypoints"
                    ] = route

                    start_idx = 0
                    if location is not None and route:
                        min_dist_sq = float("inf")
                        for idx, pt in enumerate(route):
                            d_sq = (pt[0] - location.x) ** 2 + (pt[1] - location.y) ** 2
                            if d_sq < min_dist_sq:
                                min_dist_sq = d_sq
                                start_idx = idx

                    active_route = route[start_idx:] if start_idx < len(route) else route

                    local_route = []
                    accum_dist = 0.0
                    prev_pt = [location.x, location.y] if location is not None else None
                    for point in active_route:
                        gx = (point[0] - self.coordinate_adapter.origin[0]) / self.coordinate_adapter.scale
                        gy = (point[1] - self.coordinate_adapter.origin[1]) / self.coordinate_adapter.scale
                        if 0 <= gx < self.coordinate_adapter.grid_width and 0 <= gy < self.coordinate_adapter.grid_height:
                            local_route.append(point)
                            if prev_pt is not None:
                                accum_dist += math.hypot(point[0] - prev_pt[0], point[1] - prev_pt[1])
                            prev_pt = point
                            if accum_dist >= 20.0:
                                break
                        elif local_route:
                            break

                    raw_grid_points = [
                        self.coordinate_adapter
                        .world_to_grid(
                            point
                        ) for point in (local_route or route)
                    ]
                    dense_path = []
                    for p in raw_grid_points:
                        if not dense_path:
                            dense_path.append(list(p))
                        else:
                            p0 = dense_path[-1]
                            dist = max(abs(p[0] - p0[0]), abs(p[1] - p0[1]))
                            if dist <= 1:
                                if list(p) != dense_path[-1]:
                                    dense_path.append(list(p))
                            else:
                                for step in range(1, dist + 1):
                                    ix = int(round(p0[0] + (p[0] - p0[0]) * step / dist))
                                    iy = int(round(p0[1] + (p[1] - p0[1]) * step / dist))
                                    if [ix, iy] != dense_path[-1]:
                                        dense_path.append([ix, iy])

                    planning_input[
                        "current_path"
                    ] = dense_path

                    road_grid_cells = set()
                    for pt in route:
                        rgx = int((pt[0] - self.coordinate_adapter.origin[0]) / self.coordinate_adapter.scale)
                        rgy = int((pt[1] - self.coordinate_adapter.origin[1]) / self.coordinate_adapter.scale)
                        for ddx in (-1, 0, 1):
                            for ddy in (-1, 0, 1):
                                cgx = rgx + ddx
                                cgy = rgy + ddy
                                if 0 <= cgx < self.coordinate_adapter.grid_width and 0 <= cgy < self.coordinate_adapter.grid_height:
                                    cell_wx = self.coordinate_adapter.origin[0] + cgx * self.coordinate_adapter.scale
                                    cell_wy = self.coordinate_adapter.origin[1] + cgy * self.coordinate_adapter.scale
                                    min_d = min(math.hypot(cell_wx - wp[0], cell_wy - wp[1]) for wp in route)
                                    if min_d <= 5.5:
                                        road_grid_cells.add((cgx, cgy))

                    if "drivable_space" not in planning_input:
                        planning_input["drivable_space"] = {}
                    planning_input["drivable_space"]["drivable_corridor"] = [
                        list(c) for c in road_grid_cells
                    ]
                    planning_input["drivable_space"]["allow_detour"] = True

                planning_input[
                    "require_road_route"
                ] = True


            # Keep perception positions in vehicle-relative meters. The
            # planner map receives an explicit grid projection instead.
            self._add_planner_grid_positions(
                planning_input,
                location,
                vehicle_heading,
            )



            # If ego vehicle is in CARLA and at an active traffic light, query its state
            if hasattr(self.vehicle, "get_vehicle"):
                carla_veh = self.vehicle.get_vehicle()
                if carla_veh is not None and hasattr(carla_veh, "is_at_traffic_light") and carla_veh.is_at_traffic_light():
                    tl_state = carla_veh.get_traffic_light_state()
                    state_map = {0: "RED", 1: "YELLOW", 2: "GREEN"}
                    tl_str = state_map.get(int(tl_state), "UNKNOWN")
                    planning_input["traffic_light"] = tl_str
                    planning_input["traffic_light_state"] = tl_str

        # Destination -> planner goal

        if self.destination is not None:
            dest_gx = (self.destination.x - self.coordinate_adapter.origin[0]) / self.coordinate_adapter.scale
            dest_gy = (self.destination.y - self.coordinate_adapter.origin[1]) / self.coordinate_adapter.scale
            if 0 <= dest_gx < self.coordinate_adapter.grid_width and 0 <= dest_gy < self.coordinate_adapter.grid_height:
                planning_input["goal"] = [int(dest_gx), int(dest_gy)]
            elif planning_input.get("current_path"):
                planning_input["goal"] = list(planning_input["current_path"][-1])
            else:
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

        planner_waypoints = planning_output.get(
            "waypoints"
        )

        if (
            self.vehicle is not None
            and isinstance(
                planner_waypoints,
                (list, tuple),
            )
            and planner_waypoints
        ):

            # Use the stored global road waypoints only when the planner is simply following the current path without replanning.
            # When a replanned safe detour is generated (replanned=True) we must keep the planner‑provided waypoints.
            following_global_route = (
                planning_output.get("algorithm") == "CURRENT_PATH"
                and not planning_output.get("replanned", False)
                and planning_output.get("action") in ("PROCEED_FORWARD", "SLOW_AND_REROUTE", "SLOW")
                and self.road_waypoints
            )

            if following_global_route:
                planning_output["waypoints"] = list(self.road_waypoints)
            else:
                planning_output["waypoints"] = [
                    self.coordinate_adapter.grid_to_world(point)
                    for point in planner_waypoints
                ]

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

    def _add_planner_grid_positions(
        self,
        planning_input,
        ego_location,
        ego_heading,
    ):
        """Project vehicle-relative metric hazards into the planner grid."""

        if ego_heading is None:
            return

        yaw_rad = math.radians(ego_heading)
        forward_x = math.cos(yaw_rad)
        forward_y = math.sin(yaw_rad)
        right_x = -math.sin(yaw_rad)
        right_y = math.cos(yaw_rad)

        ego_grid = self.coordinate_adapter.world_to_grid(
            [ego_location.x, ego_location.y]
        )

        for collection_name in (
            "primary_objects",
            "fallback_anomalies",
        ):
            for obstacle in planning_input.get(collection_name, []):
                if not obstacle.get("vehicle_relative", False):
                    continue

                forward_distance, lateral_offset = obstacle["position"]
                world_position = [
                    ego_location.x
                    + forward_distance * forward_x
                    + lateral_offset * right_x,
                    ego_location.y
                    + forward_distance * forward_y
                    + lateral_offset * right_y,
                ]
                grid_pos = (
                    self.coordinate_adapter.world_to_grid(
                        world_position
                    )
                )
                if forward_distance > 1.0 and grid_pos == ego_grid:
                    step_cells = max(1, int(round(forward_distance / self.coordinate_adapter.scale)))
                    step_x = int(round(step_cells * forward_x))
                    step_y = int(round(step_cells * forward_y))
                    if step_x == 0 and step_y == 0:
                        step_x = 1 if forward_x >= 0.0 else -1
                    cand_x = ego_grid[0] + step_x
                    cand_y = ego_grid[1] + step_y
                    grid_pos = [
                        max(0, min(self.coordinate_adapter.grid_width - 1, cand_x)),
                        max(0, min(self.coordinate_adapter.grid_height - 1, cand_y)),
                    ]
                    if grid_pos == ego_grid:
                        if ego_grid[0] < self.coordinate_adapter.grid_width - 1:
                            grid_pos[0] = ego_grid[0] + 1
                        elif ego_grid[0] > 0:
                            grid_pos[0] = ego_grid[0] - 1
                        elif ego_grid[1] < self.coordinate_adapter.grid_height - 1:
                            grid_pos[1] = ego_grid[1] + 1
                        elif ego_grid[1] > 0:
                            grid_pos[1] = ego_grid[1] - 1
                obstacle["grid_position"] = grid_pos
