import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration.evaluation import EvaluationTracker
from visualization.telemetry import TelemetryExporter


class FakeLocation:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


def test_m6_tracker_records_extended_metrics():
    tracker = EvaluationTracker("combined_indian_road")
    tracker.start()
    tracker.update_vehicle(FakeLocation(0.0, 0.0), 5.0)
    tracker.update_vehicle(FakeLocation(3.0, 4.0), 4.0)

    tracker.record_control(
        {"throttle": 0.4, "steer": 0.1, "brake": 0.0}
    )
    tracker.record_control(
        {"throttle": 0.0, "steer": 0.3, "brake": 1.0}
    )
    tracker.record_detection_latency(0.02)
    tracker.record_detection_latency(0.04)
    tracker.record_replanning_latency(0.05)

    tracker.finish(destination_reached=False)
    summary = tracker.summary()

    assert summary["braking_events"] == 1
    assert summary["control_stability"] < 1.0
    assert summary["detection_latency_s"] == 0.03
    assert summary["replanning_latency_s"] == 0.05
    assert summary["path_length_m"] == 5.0


def test_telemetry_records_risk_predictions_and_control():
    exporter = TelemetryExporter()
    record = exporter.record(
        perception_output={
            "objects": [1],
            "predictions": [1],
            "risk_assessments": [1, 2],
        },
        planning_output={
            "action": "STOP",
            "replanned": True,
            "replan_count": 2,
        },
        control_output={
            "throttle": 0.0,
            "steer": 0.0,
            "brake": 1.0,
        },
    )

    assert record["prediction_count"] == 1
    assert record["risk_assessment_count"] == 2
    assert record["replanned"] is True
    assert record["brake"] == 1.0
