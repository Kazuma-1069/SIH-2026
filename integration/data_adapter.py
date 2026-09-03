from interfaces.perception_output import PerceptionOutput


def perception_to_planning_input(perception_output):
    objects = (
        perception_output.objects
        if isinstance(perception_output, PerceptionOutput)
        else perception_output.get("objects", [])
    )

    primary_objects = []

    for obj in objects:
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
        "drivable_space": {},
        "confidence_uncertainty": {},
        "start": [0, 0],
        "goal": [10, 10],
    }