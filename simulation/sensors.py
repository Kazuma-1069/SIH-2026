import carla
import numpy as np


class SensorManager:
    """Manages CARLA sensors attached to the ego vehicle."""

    def __init__(self, world, vehicle):
        if world is None:
            raise ValueError("A valid CARLA world is required.")

        if vehicle is None:
            raise ValueError("A valid CARLA vehicle is required.")

        self.world = world
        self.vehicle = vehicle

        self.sensors = {}
        self.latest_data = {}

    def _spawn_sensor(self, blueprint_id, transform, name, attributes=None):
        """Create and attach a CARLA sensor to the ego vehicle."""

        blueprint_library = self.world.get_blueprint_library()
        blueprint = blueprint_library.find(blueprint_id)

        if attributes:
            for key, value in attributes.items():
                blueprint.set_attribute(key, str(value))

        sensor = self.world.spawn_actor(
            blueprint,
            transform,
            attach_to=self.vehicle,
        )

        self.sensors[name] = sensor

        return sensor

    def spawn_rgb_camera(
        self,
        name="rgb_camera",
        width=1280,
        height=720,
        fov=90,
        x=1.5,
        y=0.0,
        z=2.4,
    ):
        """Spawn an RGB camera and return the sensor actor."""

        transform = carla.Transform(
            carla.Location(x=x, y=y, z=z)
        )

        camera = self._spawn_sensor(
            "sensor.camera.rgb",
            transform,
            name,
            {
                "image_size_x": width,
                "image_size_y": height,
                "fov": fov,
            },
        )

        camera.listen(
            lambda image: self._process_camera(image, name)
        )

        return camera

    def spawn_depth_camera(
        self,
        name="depth_camera",
        width=1280,
        height=720,
        fov=90,
        x=1.5,
        y=0.0,
        z=2.4,
    ):
        """Spawn a depth camera and return the sensor actor."""

        transform = carla.Transform(
            carla.Location(x=x, y=y, z=z)
        )

        camera = self._spawn_sensor(
            "sensor.camera.depth",
            transform,
            name,
            {
                "image_size_x": width,
                "image_size_y": height,
                "fov": fov,
            },
        )

        camera.listen(
            lambda image: self._process_depth(image, name)
        )

        return camera

    def spawn_lidar(
        self,
        name="lidar",
        channels=32,
        range_m=50.0,
        points_per_second=56000,
        rotation_frequency=20,
        upper_fov=10.0,
        lower_fov=-30.0,
        x=0.0,
        y=0.0,
        z=2.5,
    ):
        """Spawn a LiDAR sensor and return the sensor actor."""

        transform = carla.Transform(
            carla.Location(x=x, y=y, z=z)
        )

        lidar = self._spawn_sensor(
            "sensor.lidar.ray_cast",
            transform,
            name,
            {
                "channels": channels,
                "range": range_m,
                "points_per_second": points_per_second,
                "rotation_frequency": rotation_frequency,
                "upper_fov": upper_fov,
                "lower_fov": lower_fov,
            },
        )

        lidar.listen(
            lambda point_cloud: self._process_lidar(
                point_cloud,
                name,
            )
        )

        return lidar

    def _process_camera(self, image, name):
        """
        Convert CARLA RGB image to a NumPy array.

        Output shape:
            (height, width, 4)

        Channels:
            BGRA
        """

        array = np.frombuffer(
            image.raw_data,
            dtype=np.uint8,
        )

        array = array.reshape(
            (image.height, image.width, 4)
        )

        self.latest_data[name] = {
            "frame": image.frame,
            "timestamp": image.timestamp,
            "data": array,
        }

    def _process_depth(self, image, name):
        """
        Convert CARLA depth image to normalized depth.

        CARLA depth is encoded using RGB channels.
        Output is depth in meters.
        """

        array = np.frombuffer(
            image.raw_data,
            dtype=np.uint8,
        )

        array = array.reshape(
            (image.height, image.width, 4)
        )

        depth = (
            array[:, :, 2].astype(np.float32)
            + array[:, :, 1].astype(np.float32) * 256.0
            + array[:, :, 0].astype(np.float32) * 256.0 * 256.0
        )

        depth /= (
            256.0 * 256.0 * 256.0 - 1.0
        )

        depth *= 1000.0

        self.latest_data[name] = {
            "frame": image.frame,
            "timestamp": image.timestamp,
            "data": depth,
        }

    def _process_lidar(self, point_cloud, name):
        """
        Convert CARLA LiDAR data to a NumPy array.

        Output shape:
            (N, 4)

        Columns:
            X, Y, Z, intensity
        """

        points = np.frombuffer(
            point_cloud.raw_data,
            dtype=np.float32,
        )

        points = points.reshape((-1, 4))

        self.latest_data[name] = {
            "frame": point_cloud.frame,
            "timestamp": point_cloud.timestamp,
            "data": points,
        }

    def get_latest_data(self, sensor_name):
        """Return the latest data produced by a sensor."""

        return self.latest_data.get(sensor_name)

    def get_sensor(self, sensor_name):
        """Return a spawned sensor actor."""

        return self.sensors.get(sensor_name)

    def destroy_sensor(self, sensor_name):
        """Destroy one sensor."""

        sensor = self.sensors.pop(sensor_name, None)

        if sensor is not None:
            sensor.stop()
            sensor.destroy()

        self.latest_data.pop(sensor_name, None)

    def destroy_all(self):
        """Destroy all sensors managed by this class."""

        for sensor_name in list(self.sensors.keys()):
            self.destroy_sensor(sensor_name)

        print("All CARLA sensors destroyed.")