from simulation.vehicle import VehicleManager


class FakeVehicle:
    def __init__(self):
        self.received_control = None

    def apply_control(self, control):
        self.received_control = control


class FakeWorld:
    pass


def test_vehicle_manager_receives_control():
    vehicle_manager = VehicleManager(
        world=FakeWorld()
    )

    fake_vehicle = FakeVehicle()

    vehicle_manager.vehicle = fake_vehicle

    control_command = {
        "throttle": 0.5,
        "steer": 0.1,
        "brake": 0.0,
    }

    vehicle_manager.vehicle.apply_control(
        control_command
    )

    assert (
        fake_vehicle.received_control["throttle"]
        == 0.5
    )

    assert (
        fake_vehicle.received_control["steer"]
        == 0.1
    )

    assert (
        fake_vehicle.received_control["brake"]
        == 0.0
    )