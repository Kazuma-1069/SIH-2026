import json
import os
import time

from simulation.carla_bridge import CarlaBridge
from simulation.world_manager import WorldManager
from simulation.vehicle import VehicleManager

from integration.evaluation import (
    EvaluationTracker,
    LiveCARLAEvaluator,
)


def run_evaluation(scenario="normal_driving", duration=30):

    bridge = CarlaBridge()
    world = bridge.connect()

    world_manager = WorldManager(
        bridge.get_client()
    )

    vehicle_manager = VehicleManager(world)

    vehicle = None
    tracker = EvaluationTracker(scenario)
    live_evaluator = None

    try:
        world_manager.set_synchronous_mode(
            enabled=True,
            fixed_delta_seconds=0.05,
        )

        vehicle = vehicle_manager.spawn_ego_vehicle(
            spawn_index=0
        )

        live_evaluator = LiveCARLAEvaluator(
            world,
            vehicle,
            tracker,
        )

        live_evaluator.attach_collision_sensor()

        print()
        print("=" * 60)
        print("M6 LIVE CARLA EVALUATION")
        print("=" * 60)
        print(f"Scenario: {scenario}")
        print(f"Duration: {duration}s")
        print(f"Vehicle: {vehicle.type_id}")
        print()

        tracker.start()

        start_time = time.perf_counter()

        while time.perf_counter() - start_time < duration:

            frame_id = world_manager.tick()

            location = vehicle_manager.get_location()
            speed_kmh = vehicle_manager.get_speed()
            speed = speed_kmh / 3.6

            tracker.update_vehicle(
                location,
                speed,
            )

            actors = world.get_actors().filter(
                "vehicle.*"
            )

            live_evaluator.check_near_collision(
                actors,
                threshold=5.0,
            )

            print(
                f"Frame {frame_id} | "
                f"Location: "
                f"({location.x:.2f}, "
                f"{location.y:.2f}, "
                f"{location.z:.2f}) | "
                f"Speed: {speed:.2f} m/s"
            )

        tracker.finish(
            destination_reached=False
        )

        summary = tracker.summary()

        os.makedirs(
            "outputs/reports",
            exist_ok=True,
        )

        timestamp = time.strftime(
            "%Y%m%d_%H%M%S"
        )

        report_path = (
            f"outputs/reports/"
            f"m6_{scenario}_{timestamp}.json"
        )

        with open(
            report_path,
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(
                summary,
                file,
                indent=2,
            )

        print()
        print("=" * 60)
        print("M6 EVALUATION COMPLETE")
        print("=" * 60)
        print(json.dumps(summary, indent=2))
        print()
        print(f"Report: {report_path}")

    finally:

        if live_evaluator is not None:
            live_evaluator.destroy()

        if vehicle is not None:
            vehicle_manager.destroy()

        world_manager.set_synchronous_mode(
            enabled=False
        )

        bridge.disconnect()


if __name__ == "__main__":
    run_evaluation()