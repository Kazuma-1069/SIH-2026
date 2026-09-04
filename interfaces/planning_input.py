"""
Standardized input contract for the planning module.
"""


PLANNING_INPUT_SCHEMA = {
    "primary_objects": [],
    "fallback_anomalies": [],
    "drivable_space": {
        "obstacle_occupied_cells": 0
    },
    "confidence_uncertainty": {},
    "start": [0, 0],
    "goal": [10, 10]
}