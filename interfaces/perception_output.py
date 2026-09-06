from dataclasses import dataclass, field
from typing import List, Optional
import time


@dataclass
class PerceptionObject:
    track_id: int
    class_id: int
    class_name: str

    confidence: float

    bbox: List[int]

    distance: Optional[float] = None
    position: Optional[List[float]] = None
    velocity: Optional[List[float]] = None
    predicted_position: Optional[List[float]] = None
    age: int = 0

    @property
    def center(self):
        x1, y1, x2, y2 = self.bbox

        return [
            (x1 + x2) / 2.0,
            (y1 + y2) / 2.0,
        ]


@dataclass
class RoadHazard:
    hazard_type: str
    confidence: float
    bbox: List[int]
    distance: Optional[float] = None
    position: Optional[List[float]] = None
    radius: Optional[float] = None
    metadata: Optional[dict] = None

    @property
    def center(self):
        x1, y1, x2, y2 = self.bbox

        return [
            (x1 + x2) / 2.0,
            (y1 + y2) / 2.0,
        ]


@dataclass
class PerceptionOutput:
    timestamp: float = field(
        default_factory=time.time
    )

    frame_id: int = 0

    image_width: int = 0
    image_height: int = 0

    objects: List[PerceptionObject] = field(
        default_factory=list
    )

    hazards: List[RoadHazard] = field(
        default_factory=list
    )

    predictions: List[dict] = field(
        default_factory=list
    )

    risk_assessments: List[dict] = field(
        default_factory=list
    )

    # M2 drivable-space output
    drivable_mask: Optional[object] = None
    environment: Optional[dict] = None

    # M2 road-edge output
    road_edges: List[dict] = field(
        default_factory=list
    )
    lidar_obstacles: List[dict] = field(
        default_factory=list
    )

    source: str = "CARLA_RGB_CAMERA"

    def to_dict(self):

        return {
            "timestamp": self.timestamp,
            "frame_id": self.frame_id,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "source": self.source,

            "objects": [
                {
                    "track_id": obj.track_id,
                    "class_id": obj.class_id,
                    "class_name": obj.class_name,
                    "confidence": obj.confidence,
                    "bbox": obj.bbox,
                    "distance": obj.distance,
                    "center": obj.center,
                    "position": obj.position,
                    "velocity": obj.velocity,
                    "predicted_position": obj.predicted_position,
                    "age": obj.age,
                }
                for obj in self.objects
            ],

            "hazards": [
                {
                    "hazard_type": hazard.hazard_type,
                    "confidence": hazard.confidence,
                    "bbox": hazard.bbox,
                    "distance": hazard.distance,
                    "center": hazard.center,
                    "position": hazard.position,
                    "radius": hazard.radius,
                    "metadata": hazard.metadata,
                }
                for hazard in self.hazards
            ],

            "road_edges": self.road_edges,
            "lidar_obstacles": self.lidar_obstacles,
            "predictions": self.predictions,
            "risk_assessments": self.risk_assessments,

            # Avoid converting a large numpy mask into JSON here.
            "has_drivable_mask": self.drivable_mask is not None,
        }