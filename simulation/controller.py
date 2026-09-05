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

        path_safe = planning_output.get(
            "path_safe",
            True
        )

        if not path_safe:
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
            }

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
        }