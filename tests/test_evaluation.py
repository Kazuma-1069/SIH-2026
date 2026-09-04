from dataclasses import dataclass, field
from typing import List, Dict
import time


@dataclass
class EvaluationMetrics:
    scenario: str
    destination_reached: bool = False
    collision_count: int = 0
    near_collision_count: int = 0
    replanning_count: int = 0
    potholes_detected: int = 0
    potholes_avoided: int = 0
    safety_violations: int = 0
    travel_time: float = 0.0
    path_length: float = 0.0
    max_speed: float = 0.0
    average_speed: float = 0.0
    notes: List[str] = field(default_factory=list)


class EvaluationTracker:
    """M6 evaluation tracker for the integrated autonomous-driving system."""

    def __init__(self, scenario: str):
        self.metrics = EvaluationMetrics(scenario=scenario)
        self._start_time = None
        self._last_location = None
        self._distance = 0.0
        self._speed_samples = []

    def start(self):
        self._start_time = time.perf_counter()

    def update_vehicle(self, location, speed_mps: float):
        if self._last_location is not None:
            dx = location.x - self._last_location.x
            dy = location.y - self._last_location.y
            dz = location.z - self._last_location.z

            self._distance += (dx * dx + dy * dy + dz * dz) ** 0.5

        self._last_location = location
        self._speed_samples.append(speed_mps)

        self.metrics.max_speed = max(
            self.metrics.max_speed,
            speed_mps,
        )

    def record_collision(self):
        self.metrics.collision_count += 1

    def record_near_collision(self):
        self.metrics.near_collision_count += 1

    def record_replan(self):
        self.metrics.replanning_count += 1

    def record_pothole(self, avoided: bool):
        self.metrics.potholes_detected += 1

        if avoided:
            self.metrics.potholes_avoided += 1

    def record_safety_violation(self):
        self.metrics.safety_violations += 1

    def finish(self, destination_reached: bool):
        self.metrics.destination_reached = destination_reached

        if self._start_time is not None:
            self.metrics.travel_time = (
                time.perf_counter() - self._start_time
            )

        self.metrics.path_length = self._distance

        if self._speed_samples:
            self.metrics.average_speed = (
                sum(self._speed_samples)
                / len(self._speed_samples)
            )

        return self.metrics

    def summary(self) -> Dict:
        return {
            "scenario": self.metrics.scenario,
            "destination_reached": self.metrics.destination_reached,
            "collision_count": self.metrics.collision_count,
            "near_collision_count": self.metrics.near_collision_count,
            "replanning_count": self.metrics.replanning_count,
            "potholes_detected": self.metrics.potholes_detected,
            "potholes_avoided": self.metrics.potholes_avoided,
            "safety_violations": self.metrics.safety_violations,
            "travel_time_s": round(self.metrics.travel_time, 3),
            "path_length_m": round(self.metrics.path_length, 3),
            "max_speed_mps": round(self.metrics.max_speed, 3),
            "average_speed_mps": round(self.metrics.average_speed, 3),
            "notes": self.metrics.notes,
        }


class FakeLocation:
    """Small CARLA-like location object for unit testing."""

    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


def test_evaluation_tracker_records_metrics():
    tracker = EvaluationTracker("normal_driving")

    tracker.start()

    tracker.update_vehicle(FakeLocation(0, 0), 5.0)
    tracker.update_vehicle(FakeLocation(3, 4), 10.0)

    tracker.record_replan()
    tracker.record_pothole(avoided=True)

    metrics = tracker.finish(destination_reached=True)

    assert metrics.destination_reached is True
    assert metrics.replanning_count == 1
    assert metrics.potholes_detected == 1
    assert metrics.potholes_avoided == 1
    assert metrics.path_length == 5.0
    assert metrics.max_speed == 10.0
    assert metrics.average_speed == 7.5


def test_evaluation_detects_collision():
    tracker = EvaluationTracker("static_obstacle")

    tracker.start()
    tracker.record_collision()
    tracker.record_near_collision()

    metrics = tracker.finish(destination_reached=False)

    assert metrics.collision_count == 1
    assert metrics.near_collision_count == 1
    assert metrics.destination_reached is False