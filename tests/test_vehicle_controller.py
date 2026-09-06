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
def test_controller_stops_when_all_waypoints_are_behind_vehicle():
    controller = VehicleController()

    output = controller.compute_control(
        {
            "action": "PROCEED_FORWARD",
            "target_speed_mps": 5.0,
            "path_safe": True,
            "waypoints": [[-5.0, 0.0]],
        },
        vehicle_location=[0.0, 0.0],
        vehicle_heading=0.0,
    )

    assert output["throttle"] == 0.0
    assert output["brake"] == 1.0
