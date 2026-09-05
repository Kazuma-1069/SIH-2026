from simulation.controller import VehicleController


def test_stop_command():
    controller = VehicleController()

    output = controller.compute_control(
        {
            "action": "STOP",
            "path_safe": False,
        }
    )

    assert output["brake"] == 1.0


def test_forward_command():
    controller = VehicleController()

    output = controller.compute_control(
        {
            "action": "PROCEED_FORWARD",
            "target_speed_mps": 5.0,
            "path_safe": True,
        }
    )

    assert output["throttle"] > 0
    