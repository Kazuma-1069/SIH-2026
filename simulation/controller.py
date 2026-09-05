class VehicleController:
    """
    M5 Vehicle Controller

    Converts planning output into
    throttle, steering and brake commands.
    """

    def compute_control(self, planning_output):

        action = planning_output.get(
            "action",
            "STOP"
        )

        target_speed = planning_output.get(
            "target_speed_mps",
            0.0
        )

        path_safe = planning_output.get(
            "path_safe",
            True
        )

        # Safety override
        if not path_safe:
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }

        # Stop command
        if action == "STOP":
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }

        # Normal driving
        return {
            "throttle": min(
                target_speed / 10.0,
                0.7
            ),
            "steer": 0.0,
            "brake": 0.0,
        }