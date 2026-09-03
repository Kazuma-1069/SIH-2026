from interfaces.perception_output import PerceptionOutput


def perception_to_planning_input(perception_output):
    """
    Convert M2 PerceptionOutput into the standardized M1 planning input.
    """

    if isinstance(perception_output, PerceptionOutput):
        objects = perception_output.objects
    elif isinstance(perception_output, dict):
        objects = perception_output.get("objects", [])
    else:
        raise TypeError(
            "perception_output must be PerceptionOutput or dict"
        )

    primary_objects = []

    for obj in objects:
        if isinstance(obj, dict):
            primary_objects.append({
                "track_id": int(obj.get("track_id", -1)),
                "class_id": int(obj.get("class_id", -1)),
                "class_name": str(obj.get("class_name", "unknown")),
                "confidence": float(obj.get("confidence", 0.0)),
                "bbox": list(obj.get("bbox", [])),
                "distance": obj.get("distance"),
            })
        else:
            primary_objects.append({
                "track_id": int(obj.track_id),
                "class_id": int(obj.class_id),
                "class_name": str(obj.class_name),
                "confidence": float(obj.confidence),
                "bbox": list(obj.bbox),
                "distance": obj.distance,
            })

    return {
        "primary_objects": primary_objects,
        "fallback_anomalies": [],
        "drivable_space": {
            "obstacle_occupied_cells": len(primary_objects)
        },
        "confidence_uncertainty": {},
        "start": [0, 0],
        "goal": [10, 10],
    }