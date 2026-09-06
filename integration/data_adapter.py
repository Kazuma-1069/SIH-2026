import math
from interfaces.perception_output import PerceptionOutput


def pixel_to_vehicle_coords(
    bbox,
    distance=None,
    image_width=1280,
    image_height=720,
    fov_deg=90.0,
    class_name=None,
):
    """
    Convert a 2D camera bounding box and distance into vehicle-relative
    metric coordinates: [distance_ahead_m, lateral_offset_m].

    Coordinate frame convention (ISO 8855 / CARLA vehicle frame):
        x (longitudinal): forward distance ahead of ego vehicle in meters (>= 0)
        y (lateral): lateral distance from vehicle centerline in meters
                     (positive = right, negative = left)
    """
    if bbox is None or len(bbox) != 4:
        return [0.0, 0.0]

    x1, y1, x2, y2 = map(float, bbox)

    width = float(image_width) if image_width and image_width > 0 else 1280.0

    # Optical center
    cx = width / 2.0

    # Horizontal field of view -> focal length in pixels
    # For 90 degree FOV, tan(45 deg) = 1.0, so fx = cx = width / 2.0
    fov_rad = math.radians(float(fov_deg) if fov_deg else 90.0)
    fx = cx / math.tan(fov_rad / 2.0) if fov_rad > 0 else cx

    # Resolve longitudinal distance (x) in meters
    dist = None
    if distance is not None:
        try:
            dist = float(distance)
            if dist <= 0:
                dist = None
        except (ValueError, TypeError):
            dist = None

    if dist is None:
        # Fallback distance estimation from known object height if available
        known_heights = {
            "person": 1.7,
            "pedestrian": 1.7,
            "car": 1.5,
            "truck": 3.0,
            "bus": 3.0,
            "motorcycle": 1.4,
            "bicycle": 1.2,
            "traffic light": 2.5,
            "stop sign": 2.0,
            "pothole": 0.3,
            "debris": 0.5,
            "construction_barricade": 1.0,
        }
        cls_key = str(class_name).lower() if class_name else ""
        known_h = known_heights.get(cls_key, 1.5)
        pixel_h = max(abs(y2 - y1), 1.0)
        dist = (known_h * fx) / pixel_h

    # Horizontal center of the bounding box
    u_center = (x1 + x2) / 2.0

    # Lateral offset (y) in meters: (u - cx) / fx = lateral_offset / dist
    lateral_offset = dist * ((u_center - cx) / fx)

    return [round(dist, 2), round(lateral_offset, 2)]


def perception_to_planning_input(
    perception_output
):
    """
    M2 PerceptionOutput -> M1 Planning Input
    Converts perception data into planning input with consistent
    vehicle-relative metric obstacle positions.
    """

    # ==========================
    # Extract perception data
    # ==========================

    if isinstance(
        perception_output,
        PerceptionOutput
    ):
        image_width = getattr(perception_output, "image_width", 1280) or 1280
        image_height = getattr(perception_output, "image_height", 720) or 720

        objects = (
            perception_output.objects
        )

        hazards = (
            perception_output.hazards
        )

        lidar_obstacles = (
            perception_output.lidar_obstacles
        )

    elif isinstance(
        perception_output,
        dict
    ):
        image_width = perception_output.get("image_width", 1280) or 1280
        image_height = perception_output.get("image_height", 720) or 720

        objects = (
            perception_output.get(
                "objects",
                []
            )
        )

        hazards = (
            perception_output.get(
                "hazards",
                []
            )
        )

        lidar_obstacles = (
            perception_output.get(
                "lidar_obstacles",
                []
            )
        )

    else:
        raise TypeError(
            "perception_output must be PerceptionOutput or dict"
        )

    # ==========================
    # Objects
    # ==========================

    primary_objects = []
    obstacle_positions = []

    for obj in objects:
        if isinstance(
            obj,
            dict
        ):
            bbox = list(
                obj.get(
                    "bbox",
                    []
                )
            )

            raw_pos = obj.get("position")
            class_name = str(obj.get("class_name", "unknown"))
            class_id = int(obj.get("class_id", -1))
            track_id = int(obj.get("track_id", -1))
            confidence = float(obj.get("confidence", 0.0))
            distance = obj.get("distance")
            radius = float(obj.get("radius", 1.0))

        else:
            bbox = list(obj.bbox)
            raw_pos = getattr(obj, "position", None)
            class_name = str(obj.class_name)
            class_id = int(obj.class_id)
            track_id = int(obj.track_id)
            confidence = float(obj.confidence)
            distance = getattr(obj, "distance", None)
            radius = getattr(obj, "radius", 1.0)

        pixel_center = None
        if len(bbox) == 4:
            pixel_center = [
                (bbox[0] + bbox[2]) / 2.0,
                (bbox[1] + bbox[3]) / 2.0,
            ]

        # Use explicitly provided metric position if distinct from raw pixel center
        if (
            raw_pos is not None
            and pixel_center is not None
            and (
                abs(raw_pos[0] - pixel_center[0]) > 1e-3
                or abs(raw_pos[1] - pixel_center[1]) > 1e-3
            )
        ):
            position = [float(raw_pos[0]), float(raw_pos[1])]
        elif len(bbox) == 4:
            position = pixel_to_vehicle_coords(
                bbox=bbox,
                distance=distance,
                image_width=image_width,
                image_height=image_height,
                class_name=class_name,
            )
        elif raw_pos is not None:
            position = [float(raw_pos[0]), float(raw_pos[1])]
        else:
            position = [0.0, 0.0]

        primary_objects.append(
            {
                "track_id": track_id,
                "class_id": class_id,
                "class_name": class_name,
                "confidence": confidence,
                "bbox": list(bbox),
                "distance": distance,
                "position": position,
                "radius": radius,
                "vehicle_relative": True,
            }
        )

        obstacle_positions.append(
            {
                "track_id": track_id,
                "class_name": class_name,
                "distance": distance,
                "confidence": confidence,
                "position": position,
                "radius": radius,
                "vehicle_relative": True,
            }
        )

    # ==========================
    # Hazards / potholes
    # ==========================

    fallback_anomalies = []

    for hazard in hazards:
        if isinstance(
            hazard,
            dict
        ):
            h_bbox = list(hazard.get("bbox", []))
            h_dist = hazard.get("distance")
            h_type = str(
                hazard.get(
                    "class_name",
                    hazard.get("hazard_type", "road_hazard"),
                )
            )
            h_conf = float(hazard.get("confidence", 0.0))
            h_raw_pos = hazard.get("position")
            h_radius = float(hazard.get("radius", 0.5))
        else:
            h_bbox = list(hazard.bbox)
            h_dist = getattr(hazard, "distance", None)
            h_type = str(hazard.hazard_type)
            h_conf = float(hazard.confidence)
            h_raw_pos = getattr(hazard, "position", None)
            h_radius = getattr(hazard, "radius", 0.5)

        h_pixel_center = None
        if len(h_bbox) == 4:
            h_pixel_center = [
                (h_bbox[0] + h_bbox[2]) / 2.0,
                (h_bbox[1] + h_bbox[3]) / 2.0,
            ]

        if (
            h_raw_pos is not None
            and h_pixel_center is not None
            and (
                abs(h_raw_pos[0] - h_pixel_center[0]) > 1e-3
                or abs(h_raw_pos[1] - h_pixel_center[1]) > 1e-3
            )
        ):
            h_pos = [float(h_raw_pos[0]), float(h_raw_pos[1])]
        elif len(h_bbox) == 4:
            h_pos = pixel_to_vehicle_coords(
                bbox=h_bbox,
                distance=h_dist,
                image_width=image_width,
                image_height=image_height,
                class_name=h_type,
            )
        elif h_raw_pos is not None:
            h_pos = [float(h_raw_pos[0]), float(h_raw_pos[1])]
        else:
            h_pos = [0.0, 0.0]

        fallback_anomalies.append(
            {
                "class_name": h_type,
                "confidence": h_conf,
                "bbox": list(h_bbox),
                "distance": h_dist,
                "position": h_pos,
                "radius": h_radius,
                "vehicle_relative": True,
            }
        )

    # ==========================
    # Output for M1
    # ==========================

    return {
        "primary_objects":
            primary_objects,

        "fallback_anomalies":
            fallback_anomalies,

        "lidar_obstacles":
            lidar_obstacles,

        "obstacle_positions":
            obstacle_positions,

        "drivable_space":
            {
                "obstacle_occupied_cells":
                    0
            },

        "confidence_uncertainty":
            {},

        # M0 overwrites these
        "start":
            [0, 0],

        "goal":
            [10, 10],

        "ego_position":
            [0.0, 0.0],
    }