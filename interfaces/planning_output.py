"""
Standardized output contract for the planning module.
"""


PLANNING_OUTPUT_SCHEMA = {
    "action": "PROCEED_FORWARD",
    "target_speed_mps": 5.0,
    "algorithm": "A_STAR",
    "hazard_count": 0,
    "waypoints": [],
    "path_safe": True,
    "safety_reason": "PATH_CLEAR",
    "confidence_uncertainty": {}
}