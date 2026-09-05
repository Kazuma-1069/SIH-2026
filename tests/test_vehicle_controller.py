from simulation.controller import VehicleController


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
    assert output["brake"] == 0.0


def test_emergency_stop():
    controller = VehicleController()

    output = controller.compute_control(
        {
            "action": "STOP",
            "target_speed_mps": 0.0,
            "path_safe": False,
        }
    )

    assert output["throttle"] == 0.0
    assert output["brake"] == 1.0