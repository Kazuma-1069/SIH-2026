from __future__ import annotations

from typing import Any, Dict, Iterable, List


DYNAMIC_CLASSES = {
    "person",
    "pedestrian",
    "bicycle",
    "bike",
    "motorcycle",
    "car",
    "truck",
    "bus",
}


class RiskAssessor:
    """Derive lightweight collision risk from existing M2 observations."""

    def __init__(self, lane_half_width_m: float = 2.0):
        self.lane_half_width_m = float(lane_half_width_m)

    def assess(
        self,
        objects: Iterable[Dict[str, Any]],
        image_width: int,
    ) -> List[Dict[str, Any]]:
        assessments = []
        for obj in objects:
            assessment = self._assess_object(obj, image_width)
            if assessment is not None:
                assessments.append(assessment)
        return assessments

    def _assess_object(
        self,
        obj: Dict[str, Any],
        image_width: int,
    ) -> Dict[str, Any] | None:
        bbox = obj.get("bbox") or []
        if len(bbox) != 4:
            return None

        distance = obj.get("distance")
        if distance is None:
            return None
        distance = max(0.0, float(distance))

        center_x = (float(bbox[0]) + float(bbox[2])) / 2.0
        lateral_m = distance * (
            center_x - float(image_width) / 2.0
        ) / max(float(image_width) / 2.0, 1.0)
        predicted_center = obj.get("predicted_position")
        predicted_lateral_m = lateral_m
        if isinstance(predicted_center, (list, tuple)) and len(predicted_center) >= 1:
            predicted_lateral_m = distance * (
                float(predicted_center[0]) - float(image_width) / 2.0
            ) / max(float(image_width) / 2.0, 1.0)

        class_name = str(obj.get("class_name", obj.get("hazard_type", "unknown"))).lower()
        path_conflict = (
            abs(lateral_m) <= self.lane_half_width_m
            or abs(predicted_lateral_m) <= self.lane_half_width_m
        )
        velocity = obj.get("velocity") or [0.0, 0.0]
        vertical_speed = abs(float(velocity[1])) if len(velocity) > 1 else 0.0
        dynamic = class_name in DYNAMIC_CLASSES

        if distance <= 5.0 and path_conflict:
            risk_level = "critical"
        elif distance <= 15.0 and path_conflict:
            risk_level = "high"
        elif dynamic and path_conflict and vertical_speed > 5.0:
            risk_level = "high"
        elif path_conflict:
            risk_level = "medium"
        else:
            risk_level = "low"

        return {
            "track_id": int(obj.get("track_id", -1)),
            "class_name": class_name,
            "hazard_type": class_name,
            "distance_m": round(distance, 2),
            "lateral_offset_m": round(lateral_m, 2),
            "predicted_lateral_offset_m": round(predicted_lateral_m, 2),
            "velocity": [float(value) for value in velocity[:2]],
            "dynamic": dynamic,
            "path_conflict": path_conflict,
            "risk_level": risk_level,
            "severity": "high" if risk_level in {"high", "critical"} else "medium",
        }
