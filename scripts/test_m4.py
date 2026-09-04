from __future__ import annotations

import math
import os
import random
import sys
import time

import carla


HOST = os.getenv("M4_CARLA_HOST", "127.0.0.1")
PORT = int(os.getenv("M4_CARLA_PORT", "2010"))

SCENARIO = os.getenv(
    "M4_SCENARIO",
    "normal"
)

MAX_FRAMES = 800


class SensorManager:

    def __init__(
        self,
        world,
        vehicle
    ):
        self.world = world
        self.vehicle = vehicle

        self.sensors = {}
        self.data = {}

        self.frames = {}



    def callback(
        self,
        name
    ):

        def receive(data):

            self.data[name] = data
            self.frames[name] = data.frame


        return receive



    def spawn(self):

        library = (
            self.world
            .get_blueprint_library()
        )


        transform = carla.Transform(
            carla.Location(
                x=1.5,
                z=1.8
            )
        )


        rgb_bp = library.find(
            "sensor.camera.rgb"
        )

        rgb_bp.set_attribute(
            "image_size_x",
            "320"
        )

        rgb_bp.set_attribute(
            "image_size_y",
            "180"
        )

        rgb_bp.set_attribute(
            "fov",
            "90"
        )


        rgb = self.world.spawn_actor(
            rgb_bp,
            transform,
            attach_to=self.vehicle
        )


        rgb.listen(
            self.callback(
                "rgb_camera"
            )
        )


        self.sensors[
            "rgb_camera"
        ] = rgb



        depth_bp = library.find(
            "sensor.camera.depth"
        )


        depth_bp.set_attribute(
            "image_size_x",
            "320"
        )

        depth_bp.set_attribute(
            "image_size_y",
            "180"
        )


        depth = self.world.spawn_actor(
            depth_bp,
            transform,
            attach_to=self.vehicle
        )


        depth.listen(
            self.callback(
                "depth_camera"
            )
        )


        self.sensors[
            "depth_camera"
        ] = depth



        lidar_bp = library.find(
            "sensor.lidar.ray_cast"
        )


        lidar_bp.set_attribute(
            "range",
            "30"
        )

        lidar_bp.set_attribute(
            "channels",
            "16"
        )

        lidar_bp.set_attribute(
            "points_per_second",
            "20000"
        )


        lidar = self.world.spawn_actor(
            lidar_bp,
            carla.Transform(
                carla.Location(
                    z=2.2
                )
            ),
            attach_to=self.vehicle
        )


        lidar.listen(
            self.callback(
                "lidar"
            )
        )


        self.sensors[
            "lidar"
        ] = lidar



    def wait_for_data(
        self,
        timeout=5
    ):

        start = time.time()


        while (
            time.time() - start
            <
            timeout
        ):

            if len(
                self.data
            ) == 3:

                return True


            time.sleep(
                0.05
            )


        return False



    def get_status(self):

        return {

            "rgb_camera":
                "rgb_camera"
                in self.data,

            "depth_camera":
                "depth_camera"
                in self.data,

            "lidar":
                "lidar"
                in self.data,

        }



    def destroy(self):

        for sensor in (
            self.sensors.values()
        ):

            try:
                sensor.stop()
            except:
                pass


            try:
                sensor.destroy()
            except:
                pass


        self.sensors.clear()
        self.data.clear()
class ScenarioManager:

    def __init__(
        self,
        world,
        vehicle,
        scenario
    ):

        self.world = world
        self.vehicle = vehicle
        self.scenario = scenario

        self.actors = []
        self.hazards = []



    def forward_location(
        self,
        distance
    ):

        transform = (
            self.vehicle
            .get_transform()
        )

        yaw = math.radians(
            transform.rotation.yaw
        )

        return carla.Location(
            x=transform.location.x +
              math.cos(yaw) * distance,

            y=transform.location.y +
              math.sin(yaw) * distance,

            z=transform.location.z
        )



    def create(self):

        if self.scenario == "normal":
            return


        if self.scenario in (
            "static_obstacle",
            "combined"
        ):

            self.spawn_static()



        if self.scenario in (
            "dynamic_obstacle",
            "combined"
        ):

            self.spawn_dynamic()



        if self.scenario in (
            "pothole",
            "combined"
        ):

            self.spawn_pothole()



    def spawn_static(self):

        bp = (
            self.world
            .get_blueprint_library()
            .find(
                "static.prop.streetbarrier"
            )
        )


        actor = self.world.try_spawn_actor(
            bp,
            carla.Transform(
                self.forward_location(
                    30
                )
            )
        )


        if actor:

            self.actors.append(
                actor
            )

            self.hazards.append(
                {
                    "type":
                    "static_obstacle",

                    "id":
                    actor.id,

                    "location":
                    actor.get_location()
                }
            )


            print(
                "[M4] Static obstacle spawned"
            )



    def spawn_dynamic(self):

        vehicles = (
            self.world
            .get_blueprint_library()
            .filter(
                "vehicle.*"
            )
        )


        bp = random.choice(
            vehicles
        )


        actor = self.world.try_spawn_actor(
            bp,
            carla.Transform(
                self.forward_location(
                    35
                )
            )
        )


        if actor:

            actor.set_simulate_physics(
                True
            )

            self.actors.append(
                actor
            )


            self.hazards.append(
                {
                    "type":
                    "dynamic_obstacle",

                    "id":
                    actor.id,

                    "location":
                    actor.get_location()
                }
            )


            print(
                "[M4] Dynamic obstacle spawned"
            )



    def spawn_pothole(self):

        bp = (
            self.world
            .get_blueprint_library()
            .find(
                "static.prop.trafficcone01"
            )
        )


        actor = self.world.try_spawn_actor(
            bp,
            carla.Transform(
                self.forward_location(
                    40
                )
            )
        )


        if actor:

            self.actors.append(
                actor
            )


            self.hazards.append(
                {
                    "type":
                    "pothole",

                    "id":
                    actor.id,

                    "location":
                    actor.get_location()
                }
            )


            print(
                "[M4] Pothole hazard spawned"
            )



    def update(
        self,
        frame
    ):

        for hazard in self.hazards:

            if hazard["type"] != "dynamic_obstacle":
                continue


            actor = self.world.get_actor(
                hazard["id"]
            )


            if actor and actor.is_alive:

                if frame % 10 == 0:

                    actor.set_target_velocity(
                        carla.Vector3D(
                            x=3.0,
                            y=0.0,
                            z=0.0
                        )
                    )



    def get_hazards(self):

        output = []


        for hazard in self.hazards:

            actor = self.world.get_actor(
                hazard["id"]
            )


            if actor and actor.is_alive:

                output.append(
                    {
                        "type":
                        hazard["type"],

                        "id":
                        actor.id,

                        "location":
                        actor.get_location()
                    }
                )


        return output



    def destroy(self):

        for actor in self.actors:

            try:

                if actor.is_alive:
                    actor.destroy()

            except:
                pass


        self.actors.clear()
        self.hazards.clear()



class VehicleManager:

    def __init__(
        self,
        world
    ):

        self.world = world
        self.vehicle = None
        self.destination = None



    def spawn(self):

        bp = (
            self.world
            .get_blueprint_library()
            .find(
                "vehicle.tesla.model3"
            )
        )


        spawn_points = (
            self.world
            .get_map()
            .get_spawn_points()
        )


        spawn = random.choice(
            spawn_points
        )


        spawn.location.z += 0.5


        self.vehicle = (
            self.world
            .spawn_actor(
                bp,
                spawn
            )
        )


        return self.vehicle



    def set_destination(self):

        points = (
            self.world
            .get_map()
            .get_spawn_points()
        )


        current = (
            self.vehicle
            .get_location()
        )


        destination = max(
            points,
            key=lambda p:
            p.location.distance(
                current
            )
        )


        self.destination = (
            destination.location
        )



    def get_speed(self):

        velocity = (
            self.vehicle
            .get_velocity()
        )


        return (
            math.sqrt(
                velocity.x ** 2 +
                velocity.y ** 2 +
                velocity.z ** 2
            )
            * 3.6
        )



    def distance_to_destination(self):

        if self.destination is None:
            return 0


        return (
            self.vehicle
            .get_location()
            .distance(
                self.destination
            )
        )



    def apply_control(
        self,
        throttle,
        brake,
        steer
    ):

        self.vehicle.apply_control(
            carla.VehicleControl(
                throttle=throttle,
                brake=brake,
                steer=steer
            )
        )



    def stop(self):

        self.vehicle.apply_control(
            carla.VehicleControl(
                brake=1.0
            )
        )



    def destroy(self):

        if self.vehicle:

            try:
                self.vehicle.destroy()
            except:
                pass
def main():

    print("=" * 60)
    print(
        "[M4] SIH 2026 CARLA INTEGRATION TEST"
    )
    print("=" * 60)

    print(
        "[M4] Scenario:",
        SCENARIO
    )


    client = None
    world = None

    vehicle_manager = None
    sensor_manager = None
    scenario_manager = None


    try:

        client = carla.Client(
            HOST,
            PORT
        )

        client.set_timeout(
            5.0
        )


        world = client.get_world()


        print(
            "[M4] CARLA connected"
        )



        settings = world.get_settings()

        settings.synchronous_mode = True

        settings.fixed_delta_seconds = 0.05

        settings.no_rendering_mode = False


        world.apply_settings(
            settings
        )


        print(
            "[M4] Synchronous mode enabled"
        )



        vehicle_manager = VehicleManager(
            world
        )


        vehicle = (
            vehicle_manager
            .spawn()
        )


        print(
            "[M4] Ego vehicle ready"
        )



        world.tick()



        vehicle_manager.set_destination()



        scenario_manager = ScenarioManager(
            world,
            vehicle,
            SCENARIO
        )


        scenario_manager.create()



        print(
            "[M4] Scenario active:",
            SCENARIO
        )


        print(
            "[M4] Hazards:",
            len(
                scenario_manager
                .get_hazards()
            )
        )



        sensor_manager = SensorManager(
            world,
            vehicle
        )


        sensor_manager.spawn()


        print(
            "[M4] Sensors active:"
        )


        for sensor in sensor_manager.sensors:

            print(
                "     -",
                sensor
            )


        sensor_manager.wait_for_data()



        status = sensor_manager.get_status()


        print(
            "[M4] Sensor status:"
        )

        print(
            "RGB:",
            status["rgb_camera"]
        )

        print(
            "Depth:",
            status["depth_camera"]
        )

        print(
            "LiDAR:",
            status["lidar"]
        )



        print(
            "\n[M4] Starting vehicle movement test..."
        )


        for frame in range(
            MAX_FRAMES
        ):


            world.tick()



            scenario_manager.update(
                frame
            )



            distance = (
                vehicle_manager
                .distance_to_destination()
            )


            speed = (
                vehicle_manager
                .get_speed()
            )



            if frame % 20 == 0:


                print(
                    "\n[M4] Frame:",
                    frame
                )

                print(
                    "Speed:",
                    f"{speed:.2f}",
                    "km/h"
                )

                print(
                    "Distance:",
                    f"{distance:.2f}",
                    "m"
                )

                print(
                    "Hazards:",
                    len(
                        scenario_manager
                        .get_hazards()
                    )
                )



            if distance < 5:

                print(
                    "[M4] Destination reached"
                )

                break



            throttle = 0.35
            brake = 0.0
            steer = 0.0



            hazards = (
                scenario_manager
                .get_hazards()
            )


            ego_location = (
                vehicle
                .get_location()
            )



            for hazard in hazards:


                hazard_distance = (
                    ego_location
                    .distance(
                        hazard["location"]
                    )
                )


                if hazard_distance < 15:


                    print(
                        "[M4] Hazard:",
                        hazard["type"],
                        hazard_distance
                    )


                    throttle = 0.0

                    brake = 0.5



            vehicle_manager.apply_control(
                throttle,
                brake,
                steer
            )



        print(
            "\n[M4] Simulation completed"
        )



    except Exception as e:

        print(
            "[M4 ERROR]",
            type(e).__name__,
            e
        )



    finally:


        if sensor_manager:

            sensor_manager.destroy()



        if scenario_manager:

            scenario_manager.destroy()



        if vehicle_manager:

            vehicle_manager.stop()

            vehicle_manager.destroy()



        if world:

            settings = world.get_settings()

            settings.synchronous_mode = False

            settings.fixed_delta_seconds = None


            world.apply_settings(
                settings
            )



        print(
            "[M4] Cleanup complete"
        )



if __name__ == "__main__":

    sys.exit(
        main()
    )