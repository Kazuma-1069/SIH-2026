import time
import cv2

from simulation.carla_bridge import CarlaBridge
from simulation.vehicle import VehicleManager
from simulation.sensors import SensorManager

from perception.yolo_detector import YOLODetector
from perception.object_tracker import ObjectTracker
from perception.depth_estimator import DepthEstimator
from perception.perception_pipeline import PerceptionPipeline

from planning.planner import Planner

from integration.pipeline import IntegrationPipeline


def create_system():

    # --------------------------------------------------
    # M4 - CARLA
    # --------------------------------------------------

    bridge = CarlaBridge(
        host="127.0.0.1",
        port=2010,
    )

    world = bridge.connect()

    vehicle_manager = VehicleManager(world)

    vehicle = vehicle_manager.spawn_ego_vehicle(
        spawn_index=0
    )

    sensor_manager = SensorManager(
        world,
        vehicle,
    )

    sensor_manager.spawn_rgb_camera(
        width=1280,
        height=720,
    )

    sensor_manager.spawn_depth_camera(
        width=1280,
        height=720,
    )

    sensor_manager.spawn_lidar()

    # --------------------------------------------------
    # M2 - PERCEPTION
    # --------------------------------------------------

    detector = YOLODetector(
        model_path="models/yolo/yolov8n.pt",
        confidence_threshold=0.40,
    )

    tracker = ObjectTracker()

    depth_estimator = DepthEstimator()

    perception = PerceptionPipeline(
        detector=detector,
        tracker=tracker,
        depth_estimator=depth_estimator,
    )

    # --------------------------------------------------
    # M1 - PLANNING
    # --------------------------------------------------

    planner = Planner()

    # --------------------------------------------------
    # M2 -> M1
    # --------------------------------------------------

    pipeline = IntegrationPipeline(
        perception_pipeline=perception,
        planner=planner,
        dashboard=None,
    )

    return (
        bridge,
        vehicle_manager,
        sensor_manager,
        pipeline,
    )


def main():

    print("=" * 60)
    print("SIH-2026 AUTONOMOUS SYSTEM")
    print("=" * 60)

    bridge = None
    vehicle_manager = None
    sensor_manager = None

    try:

        (
            bridge,
            vehicle_manager,
            sensor_manager,
            pipeline,
        ) = create_system()

        print()
        print("M4 CARLA       : READY")
        print("M2 Perception  : READY")
        print("M1 Planning    : READY")
        print("M3 Integration : READY")
        print()
        print("Waiting for camera frames...")
        print("Press Q in the camera window to stop.")
        print()

        last_frame = None

        while True:

            frame = sensor_manager.get_latest_frame(
                "rgb_camera"
            )

            if frame is None:
                time.sleep(0.05)
                continue

            # Process each new CARLA frame once.
            sensor_data = sensor_manager.get_latest_data(
                "rgb_camera"
            )

            if sensor_data is not None:
                frame_id = sensor_data["frame"]

                if frame_id == last_frame:
                    time.sleep(0.01)
                    continue

                last_frame = frame_id

            # --------------------------------------------------
            # M4 -> M2 -> M1
            # --------------------------------------------------

            perception_output, planning_output = (
                pipeline.process_frame(
                    frame,
                    show=False,
                )
            )

            # --------------------------------------------------
            # Console status
            # --------------------------------------------------

            print(
                f"\rFrame: {perception_output.frame_id} | "
                f"Objects: {len(perception_output.objects)} | "
                f"Action: {planning_output['action']} | "
                f"Speed: "
                f"{planning_output['target_speed_mps']:.1f} m/s | "
                f"Algorithm: {planning_output['algorithm']}",
                end="",
                flush=True,
            )

            # --------------------------------------------------
            # Camera display
            # --------------------------------------------------

            display = frame.copy()

            for obj in perception_output.objects:

                x1, y1, x2, y2 = obj.bbox

                cv2.rectangle(
                    display,
                    (x1, y1),
                    (x2, y2),
                    (255, 255, 255),
                    2,
                )

                label = (
                    f"{obj.class_name} "
                    f"ID:{obj.track_id} "
                    f"{obj.confidence:.2f}"
                )

                if obj.distance is not None:
                    label += (
                        f" {obj.distance:.1f}m"
                    )

                cv2.putText(
                    display,
                    label,
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            # Planning information
            cv2.putText(
                display,
                f"ACTION: {planning_output['action']}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                display,
                f"SPEED: "
                f"{planning_output['target_speed_mps']:.1f} m/s",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(
                "SIH-2026 | CARLA + Perception + Planning",
                display,
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    except KeyboardInterrupt:

        print("\nStopping system...")

    except Exception as exc:

        print("\nSYSTEM ERROR:")
        print(exc)
        raise

    finally:

        cv2.destroyAllWindows()

        if sensor_manager is not None:
            sensor_manager.destroy_all()

        if vehicle_manager is not None:
            vehicle_manager.destroy()

        if bridge is not None:
            bridge.disconnect()

        print("\nSystem stopped.")


if __name__ == "__main__":
    main()