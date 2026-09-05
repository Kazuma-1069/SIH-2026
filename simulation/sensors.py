"""
SIH-2026 M4 Sensor Manager

CARLA 0.9.16

Sensors:
- RGB Camera
- Depth Camera
- LiDAR

Output contract:

RGB:
{
    "frame": int,
    "image": numpy.ndarray
}

Depth:
{
    "frame": int,
    "image": carla.Image
}

LiDAR:
{
    "frame": int,
    "data": carla.LidarMeasurement
}
"""

import carla
import numpy as np
import cv2


class SensorManager:


    def __init__(
        self,
        world,
        vehicle
    ):

        if world is None:
            raise ValueError(
                "CARLA world required"
            )

        if vehicle is None:
            raise ValueError(
                "Vehicle required"
            )


        self.world = world
        self.vehicle = vehicle


        self.sensors = {}

        self.sensor_data = {}

        self.sensor_frames = {}


        print(
            "[M4] SensorManager initialized"
        )



    # =====================================================
    # TRANSFORMS
    # =====================================================

    def _default_camera_transform(self):

        return carla.Transform(
            carla.Location(
                x=1.5,
                z=2.4
            )
        )


    def _default_lidar_transform(self):

        return carla.Transform(
            carla.Location(
                z=2.5
            )
        )



    # =====================================================
    # CONVERSION
    # =====================================================

    def _convert_rgb_image(
        self,
        image
    ):

        if hasattr(
            image,
            "shape"
        ):
            return image


        array = np.frombuffer(
            image.raw_data,
            dtype=np.uint8
        )


        array = array.reshape(
            (
                image.height,
                image.width,
                4
            )
        )


        return cv2.cvtColor(
            array,
            cv2.COLOR_BGRA2BGR
        )



    # =====================================================
    # CALLBACKS
    # =====================================================

    def _rgb_callback(
        self,
        image
    ):

        self.sensor_data[
            "rgb_camera"
        ] = {

            "frame": image.frame,

            "image": self._convert_rgb_image(
                image
            )
        }


        self.sensor_frames[
            "rgb_camera"
        ] = image.frame



    def _depth_callback(
        self,
        image
    ):

        self.sensor_data[
            "depth_camera"
        ] = {

            "frame": image.frame,

            "image": image

        }


        self.sensor_frames[
            "depth_camera"
        ] = image.frame



    def _lidar_callback(
        self,
        point_cloud
    ):

        self.sensor_data[
            "lidar"
        ] = {

            "frame": point_cloud.frame,

            "data": point_cloud

        }


        self.sensor_frames[
            "lidar"
        ] = point_cloud.frame



    # =====================================================
    # SPAWN RGB
    # =====================================================

    def spawn_rgb_camera(
        self,
        width=1280,
        height=720
    ):

        print(
            "[M4] Spawning RGB camera..."
        )


        blueprint = (
            self.world
            .get_blueprint_library()
            .find(
                "sensor.camera.rgb"
            )
        )


        blueprint.set_attribute(
            "image_size_x",
            str(width)
        )

        blueprint.set_attribute(
            "image_size_y",
            str(height)
        )


        camera = (
            self.world.spawn_actor(
                blueprint,
                self._default_camera_transform(),
                attach_to=self.vehicle
            )
        )


        camera.listen(
            self._rgb_callback
        )


        self.sensors[
            "rgb_camera"
        ] = camera


        print(
            "[M4] RGB camera active"
        )


        return camera



    # =====================================================
    # SPAWN DEPTH
    # =====================================================

    def spawn_depth_camera(
        self,
        width=1280,
        height=720
    ):

        print(
            "[M4] Spawning depth camera..."
        )


        blueprint = (
            self.world
            .get_blueprint_library()
            .find(
                "sensor.camera.depth"
            )
        )


        blueprint.set_attribute(
            "image_size_x",
            str(width)
        )

        blueprint.set_attribute(
            "image_size_y",
            str(height)
        )


        camera = (
            self.world.spawn_actor(
                blueprint,
                self._default_camera_transform(),
                attach_to=self.vehicle
            )
        )


        camera.listen(
            self._depth_callback
        )


        self.sensors[
            "depth_camera"
        ] = camera


        print(
            "[M4] Depth camera active"
        )


        return camera



    # =====================================================
    # SPAWN LIDAR
    # =====================================================

    def spawn_lidar(
        self
    ):

        print(
            "[M4] Spawning LiDAR..."
        )


        blueprint = (
            self.world
            .get_blueprint_library()
            .find(
                "sensor.lidar.ray_cast"
            )
        )


        lidar = (
            self.world.spawn_actor(
                blueprint,
                self._default_lidar_transform(),
                attach_to=self.vehicle
            )
        )


        lidar.listen(
            self._lidar_callback
        )


        self.sensors[
            "lidar"
        ] = lidar


        print(
            "[M4] LiDAR active"
        )


        return lidar



    # =====================================================
    # ACCESS
    # =====================================================

    def get_sensor(
        self,
        name
    ):

        return self.sensors.get(
            name
        )



    def get_sensor_data(
        self,
        name
    ):

        return self.sensor_data.get(
            name
        )



    def get_latest_data(
        self,
        name=None
    ):

        if name:

            return self.sensor_data.get(
                name
            )


        return self.sensor_data



    def get_latest_frame(
        self,
        name="rgb_camera"
    ):

        data = (
            self.sensor_data.get(
                name
            )
        )


        if data is None:
            return None


        if isinstance(
            data,
            dict
        ):

            return data.get(
                "image"
            )


        return data



    def get_sensor_frames(
        self
    ):

        return self.sensor_frames



    def list_sensors(
        self
    ):

        return list(
            self.sensors.keys()
        )



    def get_active_sensors(
        self
    ):

        return self.sensors



    def has_sensor_data(
        self,
        name
    ):

        return name in self.sensor_data



    def all_sensors_ready(
        self
    ):

        return (
            len(self.sensor_data)
            ==
            len(self.sensors)
        )



    # =====================================================
    # CLEANUP
    # =====================================================

    def destroy(
        self
    ):

        for sensor in self.sensors.values():

            try:

                sensor.stop()

                sensor.destroy()

            except Exception:

                pass


        self.sensors.clear()

        self.sensor_data.clear()


        print(
            "[M4] Sensors destroyed"
        )



    def destroy_all(
        self
    ):

        self.destroy()



    def cleanup(
        self
    ):

        self.destroy()



    def close(
        self
    ):

        self.destroy()