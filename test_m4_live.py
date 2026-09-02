import time

from simulation.carla_bridge import CarlaBridge
from simulation.world_manager import WorldManager
from simulation.vehicle import VehicleManager
from simulation.sensors import SensorManager


def main():
    bridge = CarlaBridge()
    world = bridge.connect()

    world_manager = WorldManager(
        bridge.get_client()
    )

    world_manager.set_synchronous_mode(
        enabled=True,
        fixed_delta_seconds=0.05,
    )

    vehicle_manager = VehicleManager(world)

    vehicle = vehicle_manager.spawn_ego_vehicle(
        spawn_index=0
    )

    sensor_manager = SensorManager(
        world,
        vehicle,
    )

    sensor_manager.spawn_rgb_camera()
    sensor_manager.spawn_depth_camera()
    sensor_manager.spawn_lidar()

    print("\nM4 LIVE SENSOR TEST STARTED")
    print("--------------------------------")

    try:
        for _ in range(20):
            frame_id = world_manager.tick()

            # Allow asynchronous sensor callbacks
            time.sleep(0.05)

            rgb = sensor_manager.get_latest_data(
                "rgb_camera"
            )

            depth = sensor_manager.get_latest_data(
                "depth_camera"
            )

            lidar = sensor_manager.get_latest_data(
                "lidar"
            )

            print(
                f"CARLA frame: {frame_id} | "
                f"RGB: {rgb is not None} | "
                f"Depth: {depth is not None} | "
                f"LiDAR: {lidar is not None}"
            )

            if rgb is not None:
                print(
                    "  RGB:",
                    rgb["data"].shape,
                    rgb["data"].dtype,
                )

            if depth is not None:
                print(
                    "  Depth:",
                    depth["data"].shape,
                    depth["data"].dtype,
                )

            if lidar is not None:
                print(
                    "  LiDAR:",
                    lidar["data"].shape,
                    lidar["data"].dtype,
                )

        print("\nM4 SENSOR TEST COMPLETE")

    finally:
        sensor_manager.destroy_all()
        vehicle_manager.destroy()

        # Restore asynchronous mode
        world_manager.set_synchronous_mode(
            enabled=False
        )

        bridge.disconnect()

        print("M4 resources cleaned up.")


if __name__ == "__main__":
    main()