from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class Vector3:
    """Three-dimensional vector."""

    x: float
    y: float
    z: float


@dataclass
class TransformState:
    """Vehicle position and orientation."""

    location: Vector3
    rotation: Vector3


@dataclass
class VehicleState:
    """Current state of the ego vehicle."""

    actor_id: int
    type_id: str
    transform: TransformState
    velocity: Vector3
    speed_kmh: float


@dataclass
class DestinationState:
    """Final destination assigned to the vehicle."""

    location: Vector3
    distance_m: Optional[float] = None
    reached: bool = False


@dataclass
class SensorData:
    """
    Sensor data produced by CARLA.

    data may contain:
        RGB image
        Depth image
        LiDAR NumPy array
        or another sensor representation.
    """

    name: str
    frame: int
    timestamp: float
    data: Any = None
    width: Optional[int] = None
    height: Optional[int] = None


@dataclass
class SimulationOutput:
    """
    Standard M4 simulation output.

    Flow:

        CARLA/M4
            ↓
        SimulationOutput
            ↓
        M2 Perception / M0 Integration
    """

    frame: int
    timestamp: float

    vehicle: VehicleState

    destination: Optional[
        DestinationState
    ] = None

    sensors: Dict[
        str,
        SensorData
    ] = field(default_factory=dict)

    scenario: Optional[str] = None

    weather: Optional[Dict[str, Any]] = None

    metadata: Dict[str, Any] = field(
        default_factory=dict
    )

    def add_sensor(
        self,
        sensor_data: SensorData,
    ):
        """Add or update sensor data."""

        self.sensors[
            sensor_data.name
        ] = sensor_data

    def get_sensor(
        self,
        sensor_name: str,
    ) -> Optional[SensorData]:
        """Return a sensor packet by name."""

        return self.sensors.get(
            sensor_name
        )

    def has_sensor(
        self,
        sensor_name: str,
    ) -> bool:
        """Check whether sensor data exists."""

        return sensor_name in self.sensors

    def destination_reached(self):
        """Return whether the destination has been reached."""

        if self.destination is None:
            return False

        return self.destination.reached

    def to_dict(self):
        """
        Convert the simulation output into
        a dictionary for logging/message passing.
        """

        return {
            "frame": self.frame,
            "timestamp": self.timestamp,
            "vehicle": {
                "actor_id": self.vehicle.actor_id,
                "type_id": self.vehicle.type_id,
                "transform": {
                    "location": {
                        "x": self.vehicle.transform.location.x,
                        "y": self.vehicle.transform.location.y,
                        "z": self.vehicle.transform.location.z,
                    },
                    "rotation": {
                        "x": self.vehicle.transform.rotation.x,
                        "y": self.vehicle.transform.rotation.y,
                        "z": self.vehicle.transform.rotation.z,
                    },
                },
                "velocity": {
                    "x": self.vehicle.velocity.x,
                    "y": self.vehicle.velocity.y,
                    "z": self.vehicle.velocity.z,
                },
                "speed_kmh": self.vehicle.speed_kmh,
            },
            "destination": (
                {
                    "location": {
                        "x": self.destination.location.x,
                        "y": self.destination.location.y,
                        "z": self.destination.location.z,
                    },
                    "distance_m": self.destination.distance_m,
                    "reached": self.destination.reached,
                }
                if self.destination is not None
                else None
            ),
            "sensors": {
                name: {
                    "name": packet.name,
                    "frame": packet.frame,
                    "timestamp": packet.timestamp,
                    "data": packet.data,
                    "width": packet.width,
                    "height": packet.height,
                }
                for name, packet in self.sensors.items()
            },
            "scenario": self.scenario,
            "weather": self.weather,
            "metadata": self.metadata,
        }