"""
SIH-2026 Autonomous Driving System Runner

Full closed-loop:

CARLA/M4
    ↓
Sensors
    ↓
M2 Perception
    ↓
M0 Navigation
    ↓
M1 Planning
    ↓
M5 Control
    ↓
CARLA Vehicle
    ↓
M3 Visualization
"""
import sys
import os

sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            ".."
        )
    )
)
import time
import cv2


from simulation.destination_manager import (
    DestinationManager
)

from simulation.controller import (
    VehicleController
)

from simulation.carla_bridge import (
    CarlaBridge
)

from simulation.vehicle import (
    VehicleManager
)

from simulation.sensors import (
    SensorManager
)


from perception.yolo_detector import (
    YOLODetector
)

from perception.object_tracker import (
    ObjectTracker
)

from perception.depth_estimator import (
    DepthEstimator
)

from perception.perception_pipeline import (
    PerceptionPipeline
)


from planning.planner import (
    Planner
)


from integration.pipeline import (
    IntegrationPipeline
)


from visualization.dashboard import (
    Dashboard
)



def create_system():


    # ==========================
    # M4 CARLA
    # ==========================

    bridge = CarlaBridge(
        host="127.0.0.1",
        port=2010,
    )


    world = bridge.connect()



    # ==========================
    # M0 START + DESTINATION
    # ==========================

    destination_manager = (
        DestinationManager(world)
    )


    vehicle_manager = (
        VehicleManager(world)
    )


    spawn_points = (
        destination_manager
        .get_available_points()
    )


    print(
        f"Available spawn points: {len(spawn_points)}"
    )


    start_index = int(
        input(
            "Choose start spawn index: "
        )
    )


    destination_index = int(
        input(
            "Choose destination spawn index: "
        )
    )



    start_transform = (
        destination_manager
        .set_start(start_index)
    )


    destination_transform = (
        destination_manager
        .set_destination(
            destination_index
        )
    )



    # ==========================
    # SPAWN VEHICLE
    # ==========================

    vehicle = (
        vehicle_manager
        .spawn_ego_vehicle(
            transform=start_transform
        )
    )


    vehicle_manager.set_destination(
        destination_transform.location
    )



    # ==========================
    # M4 SENSORS
    # ==========================

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



    # ==========================
    # M2 PERCEPTION
    # ==========================

    detector = YOLODetector(
        model_path=(
            "models/yolo/yolov8n.pt"
        ),
        confidence_threshold=0.40,
    )


    tracker = ObjectTracker()


    depth_estimator = DepthEstimator()



    perception = PerceptionPipeline(
        detector=detector,
        tracker=tracker,
        depth_estimator=depth_estimator,
    )



    # ==========================
    # M1 PLANNING
    # ==========================

    planner = Planner()



    # ==========================
    # M5 CONTROL
    # ==========================

    controller = VehicleController()



    # ==========================
    # M3 VISUALIZATION
    # ==========================

    dashboard = Dashboard()



    # ==========================
    # COMPLETE PIPELINE
    # ==========================

    pipeline = IntegrationPipeline(

        perception_pipeline=perception,

        planner=planner,

        controller=controller,

        dashboard=dashboard,

        vehicle=vehicle_manager,

        destination=(
            destination_transform.location
        ),
    )



    return (

        bridge,

        vehicle_manager,

        sensor_manager,

        pipeline,

    )





def main():


    print("=" * 60)

    print(
        "SIH-2026 AUTONOMOUS SYSTEM"
    )

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

        print(
            "M4 CARLA          : READY"
        )

        print(
            "M2 Perception     : READY"
        )

        print(
            "M1 Planning       : READY"
        )

        print(
            "M5 Control        : READY"
        )

        print(
            "M3 Visualization  : READY"
        )

        print()


        print(
            "Waiting for camera frames..."
        )

        print(
            "Press Q to stop."
        )



        last_frame = None



        while True:



            frame = (
                sensor_manager
                .get_latest_frame(
                    "rgb_camera"
                )
            )



            if frame is None:

                time.sleep(0.05)

                continue



            sensor_data = (
                sensor_manager
                .get_latest_data(
                    "rgb_camera"
                )
            )



            if sensor_data is not None:


                frame_id = (
                    sensor_data["frame"]
                )


                if frame_id == last_frame:

                    time.sleep(0.01)

                    continue


                last_frame = frame_id



            (
                perception_output,

                planning_output,

                control_command,

            ) = pipeline.process_frame(

                frame,

                show=False,

            )



            print(

                f"\rFrame: {perception_output.frame_id} | "

                f"Objects: {len(perception_output.objects)} | "

                f"Hazards: {len(perception_output.hazards)} | "

                f"Action: {planning_output['action']} | "

                f"Speed: {planning_output['target_speed_mps']:.1f} m/s | "

                f"Control: {control_command}",

                end="",

                flush=True,

            )



            display = frame.copy()



            for obj in perception_output.objects:


                x1, y1, x2, y2 = obj.bbox



                cv2.rectangle(

                    display,

                    (x1, y1),

                    (x2, y2),

                    (255,255,255),

                    2,

                )



                cv2.putText(

                    display,

                    obj.class_name,

                    (x1, max(20,y1-8)),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.5,

                    (255,255,255),

                    1,

                )



            cv2.putText(

                display,

                f"ACTION: {planning_output['action']}",

                (20,35),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.8,

                (255,255,255),

                2,

            )



            cv2.imshow(

                "SIH-2026 Autonomous System",

                display,

            )



            key = cv2.waitKey(1) & 0xFF



            if key == ord("q"):

                break



    except KeyboardInterrupt:


        print(
            "\nStopping system..."
        )



    except Exception as exc:


        print(
            "\nSYSTEM ERROR:"
        )

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



        print(
            "\nSystem stopped."
        )




if __name__ == "__main__":

    main()