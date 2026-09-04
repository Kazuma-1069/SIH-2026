from interfaces.perception_output import RoadHazard


class HazardDetector:
    """
    Converts detector outputs into standardized road hazards.
    """

    HAZARD_CLASSES = {
        "pothole",
        "road_hazard",
        "debris",
    }

    def detect(self, detections):
        hazards = []

        for detection in detections:
            class_name = str(
                detection.get("class_name", "")
            ).lower()

            if class_name not in self.HAZARD_CLASSES:
                continue

            hazards.append(
                RoadHazard(
                    hazard_type=class_name,
                    confidence=float(
                        detection["confidence"]
                    ),
                    bbox=list(
                        map(int, detection["bbox"])
                    ),
                    distance=detection.get(
                        "distance"
                    ),
                )
            )

        return hazards