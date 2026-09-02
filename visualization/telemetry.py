"""
Telemetry exporter for the SIH-2026 autonomous driving pipeline.

Collects runtime telemetry from:
    - Simulation
    - Perception
    - Planning

The exporter supports:
    - In-memory telemetry history
    - CSV export
    - JSON export
    - Runtime summary

No CARLA-specific code is required here.
"""


from __future__ import annotations

import csv
import json
import os
import time
from typing import Any, Dict, List


class TelemetryExporter:
    """
    Collect and export autonomous-driving telemetry.

    Each telemetry record is a flat dictionary so that it can be
    consumed easily by dashboards, logs, CSV files, or reports.
    """

    def __init__(self, max_records: int = 1000):
        self.max_records = max_records
        self.records: List[Dict[str, Any]] = []

    @staticmethod
    def _get(
        obj: Any,
        key: str,
        default: Any = None,
    ) -> Any:
        """Read a field from a dictionary or object."""

        if isinstance(obj, dict):
            return obj.get(key, default)

        return getattr(obj, key, default)

    def record(
        self,
        simulation_output: Any = None,
        perception_output: Any = None,
        planning_output: Any = None,
        timestamp: float | None = None,
    ) -> Dict[str, Any]:
        """
        Create and store one telemetry record.

        The method accepts dictionaries or project interface objects.
        Unknown fields are simply omitted rather than modifying the
        standardized project interfaces.
        """

        record: Dict[str, Any] = {
            "timestamp": (
                time.time()
                if timestamp is None
                else timestamp
            )
        }

        # ---------------------------------------------------------
        # Simulation telemetry
        # ---------------------------------------------------------

        if simulation_output is not None:
            record["simulation_frame_id"] = self._get(
                simulation_output,
                "frame_id",
            )

            record["vehicle_x"] = self._get(
                simulation_output,
                "vehicle_x",
            )

            record["vehicle_y"] = self._get(
                simulation_output,
                "vehicle_y",
            )

            record["vehicle_speed_mps"] = self._get(
                simulation_output,
                "speed_mps",
            )

            record["simulation_source"] = self._get(
                simulation_output,
                "source",
            )

        # ---------------------------------------------------------
        # Perception telemetry
        # ---------------------------------------------------------

        if perception_output is not None:
            objects = self._get(
                perception_output,
                "objects",
                [],
            )

            record["perception_frame_id"] = self._get(
                perception_output,
                "frame_id",
            )

            record["object_count"] = len(
                objects or []
            )

            record["perception_source"] = self._get(
                perception_output,
                "source",
            )

            confidences = []

            for obj in objects or []:
                confidence = self._get(
                    obj,
                    "confidence",
                )

                if confidence is not None:
                    confidences.append(
                        float(confidence)
                    )

            if confidences:
                record["mean_detection_confidence"] = (
                    sum(confidences)
                    / len(confidences)
                )

        # ---------------------------------------------------------
        # Planning telemetry
        # ---------------------------------------------------------

        if planning_output is not None:
            waypoints = self._get(
                planning_output,
                "waypoints",
                [],
            )

            record["planning_action"] = self._get(
                planning_output,
                "action",
            )

            record["target_speed_mps"] = self._get(
                planning_output,
                "target_speed_mps",
            )

            record["planning_algorithm"] = self._get(
                planning_output,
                "algorithm",
            )

            record["hazard_count"] = self._get(
                planning_output,
                "hazard_count",
            )

            record["waypoint_count"] = len(
                waypoints or []
            )

            record["path_safe"] = self._get(
                planning_output,
                "path_safe",
            )

            record["safety_reason"] = self._get(
                planning_output,
                "safety_reason",
            )

        self.records.append(record)

        # Prevent unbounded memory usage.
        if len(self.records) > self.max_records:
            self.records = self.records[
                -self.max_records:
            ]

        return record

    def latest(self) -> Dict[str, Any] | None:
        """Return the most recent telemetry record."""

        if not self.records:
            return None

        return self.records[-1]

    def get_records(self) -> List[Dict[str, Any]]:
        """Return a copy of all stored telemetry."""

        return list(self.records)

    def clear(self) -> None:
        """Clear the telemetry history."""

        self.records.clear()

    def summary(self) -> Dict[str, Any]:
        """
        Return a compact runtime summary.
        """

        if not self.records:
            return {
                "record_count": 0,
                "latest_timestamp": None,
                "latest_action": None,
                "latest_speed_mps": None,
                "latest_object_count": 0,
                "latest_hazard_count": 0,
                "latest_path_safe": None,
            }

        latest = self.records[-1]

        return {
            "record_count": len(self.records),
            "latest_timestamp": latest.get(
                "timestamp"
            ),
            "latest_action": latest.get(
                "planning_action"
            ),
            "latest_speed_mps": latest.get(
                "target_speed_mps"
            ),
            "latest_object_count": latest.get(
                "object_count",
                0,
            ),
            "latest_hazard_count": latest.get(
                "hazard_count",
                0,
            ),
            "latest_path_safe": latest.get(
                "path_safe"
            ),
        }

    def export_json(
        self,
        file_path: str,
    ) -> str:
        """Export telemetry history as JSON."""

        directory = os.path.dirname(file_path)

        if directory:
            os.makedirs(
                directory,
                exist_ok=True,
            )

        with open(
            file_path,
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(
                self.records,
                file,
                indent=2,
                default=str,
            )

        return file_path

    def export_csv(
        self,
        file_path: str,
    ) -> str:
        """Export telemetry history as CSV."""

        directory = os.path.dirname(file_path)

        if directory:
            os.makedirs(
                directory,
                exist_ok=True,
            )

        if not self.records:
            with open(
                file_path,
                "w",
                encoding="utf-8",
                newline="",
            ):
                pass

            return file_path

        fieldnames = sorted(
            {
                key
                for record in self.records
                for key in record
            }
        )

        with open(
            file_path,
            "w",
            encoding="utf-8",
            newline="",
        ) as file:
            writer = csv.DictWriter(
                file,
                fieldnames=fieldnames,
            )

            writer.writeheader()
            writer.writerows(self.records)

        return file_path


def record_telemetry(
    exporter: TelemetryExporter,
    simulation_output: Any = None,
    perception_output: Any = None,
    planning_output: Any = None,
) -> Dict[str, Any]:
    """
    Convenience wrapper for recording one telemetry sample.
    """

    return exporter.record(
        simulation_output=simulation_output,
        perception_output=perception_output,
        planning_output=planning_output,
    )