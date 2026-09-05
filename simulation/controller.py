<<<<<<< HEAD
"""
SIH-2026 M5 Vehicle Controller

Converts M1 planner output into
CARLA throttle, steering and brake commands.

Flow:

M1 Planner
    |
    | waypoints + speed + safety
    ↓
M5 Controller
    |
    | throttle / steer / brake
    ↓
CARLA Vehicle
"""

import math


class VehicleController:


    def __init__(
        self,
        waypoint_threshold=2.0,
        lookahead_distance=6.0,
    ):

        # Current waypoint index
        self.current_waypoint = 0

        # Distance before switching waypoint
        self.waypoint_threshold = (
            waypoint_threshold
        )

        self.lookahead_distance = (
            lookahead_distance
        )

        self.last_heading_error = 0.0

        self.last_steer = 0.0
        self.max_steer_change = 0.15


    def _distance(
        self,
        point_a,
        point_b,
    ):

        dx = (
            point_a[0]
            -
            point_b[0]
        )

        dy = (
            point_a[1]
            -
            point_b[1]
        )

        return math.sqrt(
            dx * dx
            +
            dy * dy
        )


    def _calculate_steering(
        self,
        vehicle_location,
        target_point,
        vehicle_heading=None,
        path_heading=None,
        lateral_error=0.0,
        pursuit_distance=None,
    ):

        dx = (
            target_point[0]
            -
            vehicle_location[0]
        )

        dy = (
            target_point[1]
            -
            vehicle_location[1]
        )


        target_angle = math.atan2(
            dy,
            dx
        )

        if vehicle_heading is None:
            return 0.0

        current_angle = math.radians(
            vehicle_heading
        )

        heading_error = (
            target_angle
            -
            current_angle
        )

        heading_error = (
            heading_error
            + math.pi
        ) % (2 * math.pi) - math.pi

        path_error = 0.0
        if path_heading is not None:
            path_error = (
                path_heading
                -
                current_angle
                + math.pi
            ) % (2 * math.pi) - math.pi

        lateral_heading_error = math.atan2(
            -lateral_error,
            max(
                pursuit_distance
                if pursuit_distance is not None
                else self.lookahead_distance,
                1.0,
            ),
        )

        # CARLA uses positive steering for a positive yaw correction.
        raw_steer = (
            0.70 * heading_error
            +
            0.20 * path_error
            +
            0.10 * lateral_heading_error
        ) / math.pi

        self.last_heading_error = heading_error

        desired_steer = max(
            -1.0,
            min(
                1.0,
                raw_steer
            )
        )
        steer_change = max(
            -self.max_steer_change,
            min(
                self.max_steer_change,
                desired_steer - self.last_steer
            )
        )
        steer = self.last_steer + steer_change
        steer = max(
            -1.0,
            min(
                1.0,
                steer,
            ),
        )
        self.last_steer = steer

        print("CURRENT YAW:", vehicle_heading)
        print("TARGET ANGLE:", math.degrees(target_angle))
        print("HEADING ERROR:", math.degrees(heading_error))
        print("RAW STEER:", raw_steer)


        return steer


    def compute_control(
        self,
        planning_output,
        vehicle_location=None,
        vehicle_heading=None,
    ):


        if planning_output is None:

            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }



        action = planning_output.get(
            "action",
            "STOP"
        )

=======
class VehicleController:
    """
    M5 Vehicle Controller

    Converts planning outputs into
    throttle, steering and brake commands.
    """

    def compute_control(self, planning_output):

        action = planning_output.get(
            "action",
            "PROCEED_FORWARD"
        )

        target_speed = planning_output.get(
            "target_speed_mps",
            5.0
        )
>>>>>>> d58e2256777630ae62c8c2eadc284e68ee816a36

        path_safe = planning_output.get(
            "path_safe",
            True
        )

<<<<<<< HEAD
        bubble_safe = planning_output.get(
            "bubble_safe",
            True
        )

        bubble_emergency = planning_output.get(
            "bubble_emergency",
            False
        )

        bubble_path_safe = planning_output.get(
            "bubble_path_safe",
            True
        )

        current_path_blocked = planning_output.get(
            "current_path_blocked",
            False
        )

        safety_stop_required = (
            path_safe is False
            or bubble_safe is False
            or bubble_emergency
            or bubble_path_safe is False
        )


        destination_reached = (
            planning_output.get(
                "destination_reached",
                False
            )
        )


        target_speed = planning_output.get(
            "target_speed_mps",
            0.0
        )


        waypoints = planning_output.get(
            "waypoints",
            None
        )

        if (
            not isinstance(waypoints, (list, tuple))
            or not waypoints
        ):

            waypoints = None

        if (
            planning_output.get(
                "bubble_emergency",
                False
            )
            or planning_output.get(
                "bubble_safe",
                True
            ) is False
        ):

            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }



        # ==========================
        # EMERGENCY SAFETY STOP
        # ==========================

        if not path_safe:

=======
        if not path_safe:
>>>>>>> d58e2256777630ae62c8c2eadc284e68ee816a36
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }

<<<<<<< HEAD


        # ==========================
        # DESTINATION ARRIVED
        # ==========================

        if destination_reached:

            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }



        # ==========================
        # PLANNER STOP COMMAND
        # ==========================

        if action == "STOP":

            if not waypoints:
                return {
                    "throttle": 0.0,
                    "steer": 0.0,
                    "brake": 1.0,
                }
            target_speed = max(
                target_speed,
                2.0,
            )
        # ==========================

        steer = 0.0


        if (
            vehicle_location is not None
            and waypoints
        ):


            # Prevent index overflow

            self.current_waypoint = min(
                self.current_waypoint,
                len(waypoints)-1
            )


            vehicle_angle = (
                math.radians(vehicle_heading)
                if vehicle_heading is not None
                else None
            )
            heading_x = (
                math.cos(vehicle_angle)
                if vehicle_angle is not None
                else None
            )
            heading_y = (
                math.sin(vehicle_angle)
                if vehicle_angle is not None
                else None
            )

            # Find the closest route waypoint that is ahead of the vehicle.
            forward_indices = []
            for index in range(len(waypoints)):
                dx = waypoints[index][0] - vehicle_location[0]
                dy = waypoints[index][1] - vehicle_location[1]
                is_ahead = (
                    vehicle_angle is None
                    or dx * heading_x + dy * heading_y >= 0.0
                )
                if is_ahead:
                    forward_indices.append(index)

            if not forward_indices:
                forward_indices = [len(waypoints) - 1]

            nearest_index = min(
                forward_indices,
                key=lambda index: self._distance(
                    vehicle_location,
                    waypoints[index],
                ),
            )
            nearest_index = max(
                self.current_waypoint,
                nearest_index,
            )
            forward_indices = [
                index
                for index in forward_indices
                if index >= nearest_index
            ] or [nearest_index]

            dynamic_lookahead = max(
                2.0,
                self.lookahead_distance * (
                    0.70
                    +
                    0.06 * max(target_speed, 0.0)
                ),
            )
            lookahead_index = forward_indices[-1]
            lookahead_remaining = dynamic_lookahead
            previous_point = vehicle_location
            for index in forward_indices:
                lookahead_remaining -= self._distance(
                    previous_point,
                    waypoints[index],
                )
                previous_point = waypoints[index]
                if lookahead_remaining <= 0.0:
                    lookahead_index = index
                    break

            target_index = lookahead_index
            self.current_waypoint = max(
                self.current_waypoint,
                nearest_index,
            )
            target = waypoints[target_index]

            path_heading = None
            lateral_error = 0.0
            if nearest_index < len(waypoints) - 1:
                route_start = waypoints[nearest_index]
                route_end = waypoints[nearest_index + 1]
            elif nearest_index > 0:
                route_start = waypoints[nearest_index - 1]
                route_end = waypoints[nearest_index]
            else:
                route_start = waypoints[nearest_index]
                route_end = target

            route_dx = route_end[0] - route_start[0]
            route_dy = route_end[1] - route_start[1]
            route_length = math.hypot(route_dx, route_dy)
            if route_length > 0.0:
                path_heading = math.atan2(
                    route_dy,
                    route_dx,
                )
                lateral_error = (
                    route_dx * (vehicle_location[1] - route_start[1])
                    -
                    route_dy * (vehicle_location[0] - route_start[0])
                ) / route_length

            steer = self._calculate_steering(
                vehicle_location,
                target,
                vehicle_heading,
                path_heading,
                lateral_error,
                dynamic_lookahead,
            )

            print(
                "CURRENT WAYPOINT INDEX:",
                nearest_index,
            )
            print(
                "TARGET WAYPOINT:",
                target,
            )



        # ==========================
        # SPEED CONTROL
        # ==========================

        target_speed *= max(
            0.35,
            1.0 - 0.65 * abs(steer),
        )

        throttle = min(
            max(
                target_speed / 10.0,
                0.0,
            ),
            0.7,
        )

        brake = 0.0
        if safety_stop_required:
            throttle = 0.0
            steer = 0.0
            brake = 1.0

        print(
            "SAFETY/BUBBLE STATUS:",
            {
                "path_safe": path_safe,
                "current_path_blocked": current_path_blocked,
                "bubble_safe": bubble_safe,
                "bubble_emergency": bubble_emergency,
                "bubble_path_safe": bubble_path_safe,
                "stop_required": safety_stop_required,
            },
        )

        print(
            "FINAL STEER:",
            steer,
        )
        print(
            "TARGET SPEED:",
            target_speed,
        )
        print(
            "FINAL THROTTLE:",
            throttle,
        )
        print(
            "FINAL BRAKE:",
            brake,
        )



        return {

            "throttle": throttle,

            "steer": steer,

            "brake": brake,

=======
        if action == "STOP":
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }

        return {
            "throttle": min(
                target_speed / 10.0,
                0.7
            ),
            "steer": 0.0,
            "brake": 0.0,
>>>>>>> d58e2256777630ae62c8c2eadc284e68ee816a36
        }