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
                }
                for obj in self.objects
            ],
        }