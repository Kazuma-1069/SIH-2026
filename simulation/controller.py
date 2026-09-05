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
    ):

        # Current waypoint index
        self.current_waypoint = 0

        # Distance before switching waypoint
        self.waypoint_threshold = (
            waypoint_threshold
        )


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

        # A proportional heading correction gives M5
        # smooth steering while retaining CARLA's range.
        steer = (
            heading_error
            /
            math.pi
        )


        return max(
            -1.0,
            min(
                1.0,
                steer
            )
        )


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


        path_safe = planning_output.get(
            "path_safe",
            True
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

            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }



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

            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }



        # ==========================
        # WAYPOINT FOLLOWING
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


            target = (
                waypoints[
                    self.current_waypoint
                ]
            )


            distance = self._distance(
                vehicle_location,
                target,
            )


            # Move to next waypoint

            if (
                distance
                <
                self.waypoint_threshold
                and
                self.current_waypoint
                <
                len(waypoints)-1
            ):

                self.current_waypoint += 1


                target = (
                    waypoints[
                        self.current_waypoint
                    ]
                )


            steer = (
                self._calculate_steering(
                    vehicle_location,
                    target,
                    vehicle_heading,
                )
            )



        # ==========================
        # SPEED CONTROL
        # ==========================

        throttle = min(
            max(
                target_speed / 10.0,
                0.0,
            ),
            0.7,
        )



        return {

            "throttle": throttle,

            "steer": steer,

            "brake": 0.0,

        }