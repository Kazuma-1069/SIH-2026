"""
Simulation-only bridge from M4 ScenarioManager hazards to M2 perception.

Supplements YOLO perception with CARLA ground-truth Indian road hazards
(potholes, barricades, parked vehicles, pedestrians) expressed in the
same vehicle-relative metric convention used by Task 1.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence

from interfaces.perception_output import RoadHazard


DEFAULT_PERCEPTION_RANGE_M = 60.0
DEFAULT_MIN_FORWARD_DISTANCE_M = 0.5
DEFAULT_MAX_LATERAL_DISTANCE_M = 20.0
SIMULATION_HAZARD_CONFIDENCE = 0.99

SUPPORTED_HAZARD_TYPES = {
    "pothole",
    "parked_vehicle",
    "pedestrian",
    "construction_barricade",
    "human_crossing",
    "bike_ahead",
    "sudden_stopping_car",
    "uneven_road",
    "no_road",
}

HAZARD_TYPE_ALIASES = {
    "static_obstacle": "parked_vehicle",
    "dynamic_obstacle": "parked_vehicle",
}

DEFAULT_HAZARD_RADIUS = {
    "pothole": 0.6,
    "construction_barricade": 1.2,
    "parked_vehicle": 2.0,
    "pedestrian": 1.0,
    "human_crossing": 1.0,
    "bike_ahead": 1.5,
    "sudden_stopping_car": 2.0,
    "uneven_road": 1.0,
    "no_road": 3.5,
}


def normalize_hazard_type(hazard_type: str) -> str:
    normalized = str(hazard_type or "road_hazard").strip().lower()
    return HAZARD_TYPE_ALIASES.get(normalized, normalized)


def world_to_vehicle_relative(
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    world_x: float,
    world_y: float,
) -> List[float]:
    """
    Convert a CARLA world XY location into vehicle-relative meters.

    Convention matches Task 1 / data_adapter:
        x = forward distance ahead of ego (meters)
        y = lateral offset (positive = right, negative = left)
    """

    delta_x = world_x - ego_x
    delta_y = world_y - ego_y
    yaw_rad = math.radians(ego_yaw_deg)

    forward_x = math.cos(yaw_rad)
    forward_y = math.sin(yaw_rad)
    right_x = -math.sin(yaw_rad)
    right_y = math.cos(yaw_rad)

    forward_distance = (
        delta_x * forward_x
        + delta_y * forward_y
    )
    lateral_offset = (
        delta_x * right_x
        + delta_y * right_y
    )

    return [
        round(forward_distance, 2),
        round(lateral_offset, 2),
    ]


def _extract_world_location(
    hazard: Dict[str, Any],
) -> Optional[Sequence[float]]:
    location = hazard.get("location")

    if not isinstance(location, dict):
        return None

    try:
        return (
            float(location["x"]),
            float(location["y"]),
        )
    except (KeyError, TypeError, ValueError):
        return None


def _hazard_position(
    hazard: Dict[str, Any],
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
) -> Optional[List[float]]:
    world_location = _extract_world_location(hazard)

    if world_location is not None:
        return world_to_vehicle_relative(
            ego_x,
            ego_y,
            ego_yaw_deg,
            world_location[0],
            world_location[1],
        )

    if (
        "distance_ahead" in hazard
        and "lateral_offset" in hazard
    ):
        try:
            return [
                round(float(hazard["distance_ahead"]), 2),
                round(float(hazard["lateral_offset"]), 2),
            ]
        except (TypeError, ValueError):
            return None

    return None


def is_hazard_in_perception_range(
    position: Sequence[float],
    max_range_m: float = DEFAULT_PERCEPTION_RANGE_M,
    min_forward_m: float = DEFAULT_MIN_FORWARD_DISTANCE_M,
    max_lateral_m: float = DEFAULT_MAX_LATERAL_DISTANCE_M,
) -> bool:
    forward_distance = float(position[0])
    lateral_offset = abs(float(position[1]))

    return (
        forward_distance >= min_forward_m
        and forward_distance <= max_range_m
        and lateral_offset <= max_lateral_m
    )


def scenario_hazard_to_road_hazard(
    hazard: Dict[str, Any],
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    max_range_m: float = DEFAULT_PERCEPTION_RANGE_M,
) -> Optional[RoadHazard]:
    if hazard.get("active") is False:
        return None

    hazard_type = normalize_hazard_type(
        hazard.get("type", "road_hazard")
    )

    position = _hazard_position(
        hazard,
        ego_x,
        ego_y,
        ego_yaw_deg,
    )

    if position is None:
        return None

    if not is_hazard_in_perception_range(
        position,
        max_range_m=max_range_m,
    ):
        return None

    radius = float(
        hazard.get(
            "radius",
            DEFAULT_HAZARD_RADIUS.get(
                hazard_type,
                0.8,
            ),
        )
    )

    return RoadHazard(
        hazard_type=hazard_type,
        confidence=SIMULATION_HAZARD_CONFIDENCE,
        bbox=[0, 0, 1, 1],
        distance=position[0],
        position=position,
        radius=radius,
        metadata={
            key: value
            for key, value in hazard.items()
            if key not in {
                "id",
                "type",
                "location",
                "distance_ahead",
                "lateral_offset",
                "active",
            }
        },
    )


def bridge_scenario_hazards(
    scenario_hazards: Iterable[Dict[str, Any]],
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    max_range_m: float = DEFAULT_PERCEPTION_RANGE_M,
) -> List[RoadHazard]:
    bridged: List[RoadHazard] = []

    for hazard in scenario_hazards:
        if not isinstance(hazard, dict):
            continue

        road_hazard = scenario_hazard_to_road_hazard(
            hazard,
            ego_x,
            ego_y,
            ego_yaw_deg,
            max_range_m=max_range_m,
        )

        if road_hazard is not None:
            bridged.append(road_hazard)

    return bridged
