"""
SIH 2026 - CARLA Sensor Manager

CARLA version:
    0.9.16

Managed sensors:
    - RGB camera
    - Depth camera
    - LiDAR
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

import carla


class SensorManager:
    """
    Manages all virtual sensors attached to the ego vehicle.

    The manager keeps the sensor API simple for M4 and exposes the
    latest sensor data for M2 perception.
    """

    def __init__(
        self,
        world: carla.World,
        vehicle: carla.Vehicle,
    ):
        self.world = world
        self.vehicle = vehicle

        self.blueprint_library = world.get_blueprint_library()

        self.sensors: Dict[str, carla.Sensor] = {}

        self.sensor_data: Dict[str, Any] = {
            "rgb_camera": None,
            "depth_camera": None,
            "lidar": None,
        }

        self.sensor_frames: Dict[str, Optional[int]] = {
            "rgb_camera": None,
            "depth_camera": None,
            "lidar": None,
        }

        self.sensor_status: Dict[str, bool] = {
            "rgb_camera": False,
            "depth_camera": False,
            "lidar": False,
        }

        self.lock = threading.Lock()

        self.callbacks: Dict[str, Optional[Callable]] = {
            "rgb_camera": None,
            "depth_camera": None,
            "lidar": None,
        }

        print("[M4] SensorManager initialized")

    # ============================================================
    # TRANSFORM HELPERS
    # ============================================================

    @staticmethod
    def _default_camera_transform() -> carla.Transform:
        return carla.Transform(
            carla.Location(
                x=1.5,
                y=0.0,
                z=2.2,
            ),
            carla.Rotation(
                pitch=-5.0,
                yaw=0.0,
                roll=0.0,
            ),
        )

    @staticmethod
    def _default_lidar_transform() -> carla.Transform:
        return carla.Transform(
            carla.Location(
                x=0.0,
                y=0.0,
                z=2.5,
            ),
            carla.Rotation(
                pitch=0.0,
                yaw=0.0,
                roll=0.0,
            ),
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def _rgb_callback(self, image: carla.Image) -> None:
        try:
            raw = image.raw_data

            import numpy as np

            array = np.frombuffer(raw, dtype=np.uint8)

            array = array.reshape(
                (image.height, image.width, 4)
            )

            # CARLA camera data is BGRA.
            rgb = array[:, :, :3][:, :, ::-1].copy()

            with self.lock:
                self.sensor_data["rgb_camera"] = rgb
                self.sensor_frames["rgb_camera"] = image.frame
                self.sensor_status["rgb_camera"] = True

            callback = self.callbacks.get("rgb_camera")

            if callback is not None:
                callback(rgb, image.frame)

        except Exception as exc:
            print(f"[M4] RGB callback error: {exc}")

    def _depth_callback(self, image: carla.Image) -> None:
        try:
            import numpy as np

            array = np.frombuffer(
                image.raw_data,
                dtype=np.uint8,
            )

            array = array.reshape(
                (image.height, image.width, 4)
            )

            # CARLA depth encoding:
            # R + G*256 + B*256^2
            depth = (
                array[:, :, 0].astype(np.float32)
                + array[:, :, 1].astype(np.float32) * 256.0
                + array[:, :, 2].astype(np.float32) * 65536.0
            )

            depth /= 16777215.0

            # Convert normalized CARLA depth to meters.
            depth *= 1000.0

            with self.lock:
                self.sensor_data["depth_camera"] = depth
                self.sensor_frames["depth_camera"] = image.frame
                self.sensor_status["depth_camera"] = True

            callback = self.callbacks.get("depth_camera")

            if callback is not None:
                callback(depth, image.frame)

        except Exception as exc:
            print(f"[M4] Depth callback error: {exc}")

    def _lidar_callback(self, point_cloud: carla.LidarMeasurement) -> None:
        try:
            import numpy as np

            points = np.frombuffer(
                point_cloud.raw_data,
                dtype=np.float32,
            )

            points = points.reshape((-1, 4)).copy()

            with self.lock:
                self.sensor_data["lidar"] = points
                self.sensor_frames["lidar"] = point_cloud.frame
                self.sensor_status["lidar"] = True

            callback = self.callbacks.get("lidar")

            if callback is not None:
                callback(points, point_cloud.frame)

        except Exception as exc:
            print(f"[M4] LiDAR callback error: {exc}")

    # ============================================================
    # RGB CAMERA
    # ============================================================

    def spawn_rgb_camera(
        self,
        transform: Optional[carla.Transform] = None,
        callback: Optional[Callable] = None,
        image_size_x: int = 640,
        image_size_y: int = 360,
        fov: float = 90.0,
        sensor_tick: float = 0.0,
        **kwargs,
    ) -> carla.Sensor:
        """
        Spawn the RGB camera.

        Returns:
            CARLA RGB camera sensor actor.
        """

        print("[M4] Spawning RGB camera...")

        blueprint = self.blueprint_library.find(
            "sensor.camera.rgb"
        )

        blueprint.set_attribute(
            "image_size_x",
            str(image_size_x),
        )

        blueprint.set_attribute(
            "image_size_y",
            str(image_size_y),
        )

        blueprint.set_attribute(
            "fov",
            str(fov),
        )

        if sensor_tick > 0:
            blueprint.set_attribute(
                "sensor_tick",
                str(sensor_tick),
            )

        camera_transform = (
            transform
            if transform is not None
            else self._default_camera_transform()
        )

        self.callbacks["rgb_camera"] = callback

        camera = self.world.spawn_actor(
            blueprint,
            camera_transform,
            attach_to=self.vehicle,
        )

        self.sensors["rgb_camera"] = camera

        camera.listen(self._rgb_callback)

        print("[M4] RGB camera active")

        return camera

    # ============================================================
    # DEPTH CAMERA
    # ============================================================

    def spawn_depth_camera(
        self,
        transform: Optional[carla.Transform] = None,
        callback: Optional[Callable] = None,
        image_size_x: int = 640,
        image_size_y: int = 360,
        fov: float = 90.0,
        sensor_tick: float = 0.0,
        **kwargs,
    ) -> carla.Sensor:
        """
        Spawn the depth camera.

        Returns:
            CARLA depth camera sensor actor.
        """

        print("[M4] Spawning depth camera...")

        blueprint = self.blueprint_library.find(
            "sensor.camera.depth"
        )

        blueprint.set_attribute(
            "image_size_x",
            str(image_size_x),
        )

        blueprint.set_attribute(
            "image_size_y",
            str(image_size_y),
        )

        blueprint.set_attribute(
            "fov",
            str(fov),
        )

        if sensor_tick > 0:
            blueprint.set_attribute(
                "sensor_tick",
                str(sensor_tick),
            )

        depth_transform = (
            transform
            if transform is not None
            else self._default_camera_transform()
        )

        self.callbacks["depth_camera"] = callback

        depth_camera = self.world.spawn_actor(
            blueprint,
            depth_transform,
            attach_to=self.vehicle,
        )

        self.sensors["depth_camera"] = depth_camera

        depth_camera.listen(self._depth_callback)

        print("[M4] Depth camera active")

        return depth_camera

    # ============================================================
    # LIDAR
    # ============================================================

    def spawn_lidar(
        self,
        transform: Optional[carla.Transform] = None,
        callback: Optional[Callable] = None,
        channels: int = 16,
        range: float = 50.0,
        points_per_second: int = 56000,
        rotation_frequency: float = 20.0,
        upper_fov: float = 10.0,
        lower_fov: float = -30.0,
        sensor_tick: float = 0.0,
        **kwargs,
    ) -> carla.Sensor:
        """
        Spawn a LiDAR sensor.

        Returns:
            CARLA LiDAR sensor actor.
        """

        print("[M4] Spawning LiDAR...")

        blueprint = self.blueprint_library.find(
            "sensor.lidar.ray_cast"
        )

        blueprint.set_attribute(
            "channels",
            str(channels),
        )

        blueprint.set_attribute(
            "range",
            str(range),
        )

        blueprint.set_attribute(
            "points_per_second",
            str(points_per_second),
        )

        blueprint.set_attribute(
            "rotation_frequency",
            str(rotation_frequency),
        )

        blueprint.set_attribute(
            "upper_fov",
            str(upper_fov),
        )

        blueprint.set_attribute(
            "lower_fov",
            str(lower_fov),
        )

        if sensor_tick > 0:
            blueprint.set_attribute(
                "sensor_tick",
                str(sensor_tick),
            )

        lidar_transform = (
            transform
            if transform is not None
            else self._default_lidar_transform()
        )

        self.callbacks["lidar"] = callback

        lidar = self.world.spawn_actor(
            blueprint,
            lidar_transform,
            attach_to=self.vehicle,
        )

        self.sensors["lidar"] = lidar

        lidar.listen(self._lidar_callback)

        print("[M4] LiDAR active")

        return lidar

    # ============================================================
    # GENERIC SENSOR ACCESS
    # ============================================================

    def get_sensor(self, name: str) -> Optional[carla.Sensor]:
        return self.sensors.get(name)

    def get_sensor_data(self, name: str) -> Any:
        with self.lock:
            return self.sensor_data.get(name)

    def get_latest_data(self) -> Dict[str, Any]:
        with self.lock:
            return dict(self.sensor_data)

    def get_sensor_status(self) -> Dict[str, bool]:
        with self.lock:
            return dict(self.sensor_status)

    def get_sensor_frames(self) -> Dict[str, Optional[int]]:
        with self.lock:
            return dict(self.sensor_frames)

    def has_sensor_data(self, name: str) -> bool:
        with self.lock:
            return self.sensor_status.get(name, False)

    def all_sensors_ready(self) -> bool:
        with self.lock:
            return all(
                self.sensor_status.get(name, False)
                for name in (
                    "rgb_camera",
                    "depth_camera",
                    "lidar",
                )
            )

    # ============================================================
    # SENSOR LIST
    # ============================================================

    def list_sensors(self) -> list[str]:
        return list(self.sensors.keys())

    def get_active_sensors(self) -> Dict[str, carla.Sensor]:
        return dict(self.sensors)

    # ============================================================
    # STOP SENSOR
    # ============================================================

    def stop_sensor(self, name: str) -> bool:
        sensor = self.sensors.get(name)

        if sensor is None:
            return False

        try:
            sensor.stop()
        except Exception:
            pass

        with self.lock:
            self.sensor_status[name] = False

        return True

    # ============================================================
    # DESTROY / CLEANUP
    # ============================================================

    def destroy(self) -> None:
        """
        Stop and destroy all sensors.

        This method fixes the previous cleanup warning:
            'SensorManager' object has no attribute 'destroy'
        """

        sensors = list(self.sensors.items())

        for name, sensor in sensors:
            try:
                sensor.stop()
            except Exception:
                pass

            try:
                sensor.destroy()
            except Exception:
                pass

            with self.lock:
                self.sensor_status[name] = False
                self.sensor_data[name] = None
                self.sensor_frames[name] = None

        self.sensors.clear()

        print("All CARLA sensors destroyed.")

    # Alias for compatibility with code that uses cleanup().
    def cleanup(self) -> None:
        self.destroy()

    # Alias for compatibility with code that uses close().
    def close(self) -> None:
        self.destroy()

    # ============================================================
    # CONTEXT MANAGER
    # ============================================================

    def __enter__(self) -> "SensorManager":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.destroy()