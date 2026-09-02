"""
Standardized output contract for the CARLA simulation module.

M4 simulation
    ↓
SimulationOutput
    ↓
Integration layer
    ↓
M2 / M1 / M3
"""

from dataclasses import dataclass, field
from typing import List, Optional
import time


@dataclass
class VehicleState:
    """Current state of the ego vehicle."""

    vehicle_id: str

    x: float
    y: float
    z: float

    velocity_x: float
    velocity_y: float
    velocity_z: float

    speed_kmh: float

    yaw: float
    pitch: float
    roll: float


@dataclass
class SensorFrame:
    """Metadata describing a sensor frame."""

    sensor_name: str

    frame_id: int

    timestamp: float


@dataclass
class SimulationOutput:
    """
    Standardized output produced by M4.

    This contains simulation state and sensor-frame metadata.
    Raw sensor data remains owned by SensorManager and is
    passed through the integration pipeline when required.
    """

    timestamp: float = field(
        default_factory=time.time
    )

    frame_id: int = 0

    map_name: str = ""

    vehicle: Optional[VehicleState] = None

    sensors: List[SensorFrame] = field(
        default_factory=list
    )

    source: str = "CARLA"

    def to_dict(self):
        """Convert the simulation output into a dictionary."""

        vehicle_data = None

        if self.vehicle is not None:
            vehicle_data = {
                "vehicle_id": self.vehicle.vehicle_id,
                "position": {
                    "x": self.vehicle.x,
                    "y": self.vehicle.y,
                    "z": self.vehicle.z,
                },
                "velocity": {
                    "x": self.vehicle.velocity_x,
                    "y": self.vehicle.velocity_y,
                    "z": self.vehicle.velocity_z,
                },
                "speed_kmh": self.vehicle.speed_kmh,
                "rotation": {
                    "yaw": self.vehicle.yaw,
                    "pitch": self.vehicle.pitch,
                    "roll": self.vehicle.roll,
                },
            }

        return {
            "timestamp": self.timestamp,
            "frame_id": self.frame_id,
            "map_name": self.map_name,
            "vehicle": vehicle_data,
            "sensors": [
                {
                    "sensor_name": sensor.sensor_name,
                    "frame_id": sensor.frame_id,
                    "timestamp": sensor.timestamp,
                }
                for sensor in self.sensors
            ],
            "source": self.source,
        }
