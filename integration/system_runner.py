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

carla_python_api = os.path.join(
    os.environ.get(
        "CARLA_PYTHON_API",
        r"C:\CARLA_0.9.16\PythonAPI\carla",
    )
)
if os.path.isdir(carla_python_api) and carla_python_api not in sys.path:
    sys.path.insert(0, carla_python_api)


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

from simulation.scenario_manager import (
    ScenarioManager
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


    start_env = os.getenv("START_SPAWN_INDEX")
    if start_env is not None and start_env.strip() != "":
        start_index = int(start_env.strip())
    else:
        start_index = int(
            input(
                "Choose start spawn index: "
            )
        )

    dest_env = os.getenv("DEST_SPAWN_INDEX")
    if dest_env is not None and dest_env.strip() != "":
        destination_index = int(dest_env.strip())
    else:
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

    scenario_name = os.getenv("M4_SCENARIO")
    if not scenario_name:
        print("\nSelect M4 Scenario:")
        print("  1. normal_driving (default)")
        print("  2. bike_ahead")
        print("  3. human_crossing")
        print("  4. static_obstacle")
        print("  5. dynamic_obstacle")
        print("  6. indian_road_hazards")
        print("  7. combined_indian_road")
        try:
            choice = input("Choose scenario [1-7 or name, default: 1]: ").strip()
        except (EOFError, KeyboardInterrupt):
            choice = "1"
        scenario_map = {
            "1": "normal_driving",
            "2": "bike_ahead",
            "3": "human_crossing",
            "4": "static_obstacle",
            "5": "dynamic_obstacle",
            "6": "indian_road_hazards",
            "7": "combined_indian_road",
        }
        scenario_name = scenario_map.get(choice, choice if choice else "normal_driving")

    scenario_manager = ScenarioManager(
        world,
        vehicle=vehicle,
    )
    scenario_manager.set_scenario(
        scenario_name
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

        scenario_manager=scenario_manager,
    )



    return (

        bridge,

        vehicle_manager,

        sensor_manager,

        pipeline,

        scenario_manager,

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

    scenario_manager = None



    try:


        (

            bridge,

            vehicle_manager,

            sensor_manager,

            pipeline,

            scenario_manager,

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

            if scenario_manager is not None:
                scenario_manager.update_dynamic_obstacles()



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

            if perception_output.frame_id <= 3:
                ego_loc = vehicle_manager.get_location() if vehicle_manager else None
                ego_spd = vehicle_manager.get_speed() if vehicle_manager else 0.0
                all_actors = bridge.world.get_actors() if bridge and bridge.world else []
                ego_id = vehicle_manager.vehicle.id if vehicle_manager and vehicle_manager.vehicle else -1
                npc_actors = [a for a in all_actors if a.id != ego_id]
                npc_vehicles = [a for a in npc_actors if "vehicle" in a.type_id]
                bikes = [a for a in npc_vehicles if any(k in a.type_id for k in ["bike", "cycle", "ninja", "vespa", "yzf", "low_rider", "cross", "century", "omafiets"])]
                pedestrians = [a for a in npc_actors if "walker" in a.type_id]
                yolo_known = [o for o in perception_output.objects if o.class_name != "unknown"]
                yolo_unknown = [o for o in perception_output.objects if o.class_name == "unknown"]
                ctrl = control_command or {}
                loc_str = f"({ego_loc.x:.1f}, {ego_loc.y:.1f})" if ego_loc else "None"
                print(f"\n[DEBUG] Scenario: {scenario_manager.active_scenario if scenario_manager else 'none'}")
                print(f"[DEBUG] Ego location: {loc_str}")
                print(f"[DEBUG] Ego speed: {ego_spd:.1f} m/s")
                print(f"[DEBUG] NPC actor count: {len(npc_actors)}")
                print(f"[DEBUG] NPC vehicle count: {len(npc_vehicles)}")
                print(f"[DEBUG] Motorcycle/bike count: {len(bikes)}")
                print(f"[DEBUG] Pedestrian count: {len(pedestrians)}")
                print(f"[DEBUG] Rickshaw/three-wheeler count: 0 (No CARLA 0.9.16 asset)")
                print(f"[DEBUG] Hazard count: {len(perception_output.hazards)}")
                print(f"[DEBUG] YOLO detections: {len(yolo_known)}")
                print(f"[DEBUG] Unknown detections: {len(yolo_unknown)}")
                print(f"[DEBUG] Traffic light state: {planning_output.get('traffic_light_state', 'NONE')}")
                print(f"[DEBUG] Goal: {planning_output.get('destination', [])}")
                print(f"[DEBUG] Global route points: {len(pipeline.road_waypoints) if pipeline and pipeline.road_waypoints else 0}")
                print(f"[DEBUG] Local waypoints: {len(planning_output.get('waypoints', []))}")
                bike_haz = next((h for h in (planning_output.get('hazards') or []) if 'bike' in str(h.get('class_name', '')).lower() or 'bike' in str(h.get('hazard_type', '')).lower()), None)
                bike_grid = bike_haz.get('grid_position') if bike_haz else None
                bike_dist = bike_haz.get('distance') if bike_haz else None
                ego_grid_pos = pipeline.coordinate_adapter.world_to_grid([ego_loc.x, ego_loc.y]) if ego_loc else None
                print(f"[DEBUG] Ego grid position: {ego_grid_pos}")
                print(f"[DEBUG] Bike grid position: {bike_grid}")
                print(f"[DEBUG] Bike distance: {bike_dist} m")
                print(f"[DEBUG] Bubble safe: {planning_output.get('bubble_safe', True)}")
                print(f"[DEBUG] Bubble emergency: {planning_output.get('bubble_emergency', False)}")
                print(f"[DEBUG] Path safe: {planning_output.get('path_safe', True)}")
                print(f"[DEBUG] Planner action: {planning_output.get('action')}")
                print(f"[DEBUG] Target speed: {planning_output.get('target_speed_mps', 0.0):.1f} m/s")
                print(f"[DEBUG] M5 throttle: {ctrl.get('throttle', 0.0):.2f}")
                print(f"[DEBUG] M5 steer: {ctrl.get('steer', 0.0):.2f}")
                print(f"[DEBUG] M5 brake: {ctrl.get('brake', 0.0):.2f}")

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

            max_frames = int(os.getenv("MAX_FRAMES", "0") or "0")
            if max_frames > 0 and perception_output.frame_id >= max_frames:
                print(f"\nReached MAX_FRAMES ({max_frames}). Stopping.")
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

        if scenario_manager is not None:

            scenario_manager.destroy()



        if bridge is not None:

            bridge.disconnect()



        print(
            "\nSystem stopped."
        )




if __name__ == "__main__":

    main()