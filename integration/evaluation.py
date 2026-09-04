import carla
from dataclasses import dataclass, field
from typing import List, Dict
import math
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
    """Collects M6 metrics from the integrated driving system."""

    def __init__(self, scenario: str):
        self.metrics = EvaluationMetrics(scenario=scenario)

        self._start_time = None
        self._last_location = None
        self._distance = 0.0
        self._speed_samples = []

    def start(self):
        self._start_time = time.perf_counter()

    def update_vehicle(self, location, speed_mps):
        if self._last_location is not None:
            dx = location.x - self._last_location.x
            dy = location.y - self._last_location.y
            dz = location.z - self._last_location.z

            self._distance += math.sqrt(
                dx * dx + dy * dy + dz * dz
            )

        self._last_location = location

        speed_mps = max(0.0, float(speed_mps))
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

    def record_pothole(self, avoided=True):
        self.metrics.potholes_detected += 1

        if avoided:
            self.metrics.potholes_avoided += 1

    def record_safety_violation(self):
        self.metrics.safety_violations += 1

    def finish(self, destination_reached=False):
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
            "destination_reached":
                self.metrics.destination_reached,
            "collision_count":
                self.metrics.collision_count,
            "near_collision_count":
                self.metrics.near_collision_count,
            "replanning_count":
                self.metrics.replanning_count,
            "potholes_detected":
                self.metrics.potholes_detected,
            "potholes_avoided":
                self.metrics.potholes_avoided,
            "safety_violations":
                self.metrics.safety_violations,
            "travel_time_s":
                round(self.metrics.travel_time, 3),
            "path_length_m":
                round(self.metrics.path_length, 3),
            "max_speed_mps":
                round(self.metrics.max_speed, 3),
            "average_speed_mps":
                round(self.metrics.average_speed, 3),
            "notes":
                self.metrics.notes,
        }


def carla_speed_mps(vehicle):
    """
    Convert CARLA vehicle velocity into speed in m/s.
    """
    velocity = vehicle.get_velocity()

    return math.sqrt(
        velocity.x ** 2
        + velocity.y ** 2
        + velocity.z ** 2
    )
class LiveCARLAEvaluator:
    """Connects CARLA runtime events to EvaluationTracker."""

    def __init__(self, world, vehicle, tracker):
        self.world = world
        self.vehicle = vehicle
        self.tracker = tracker

        self.collision_sensor = None
        self.collision_events = 0

        self._last_near_collision_time = 0.0
        self.near_collision_cooldown = 1.0

    def attach_collision_sensor(self):
        """Attach a CARLA collision sensor to the ego vehicle."""

        blueprint_library = self.world.get_blueprint_library()

        collision_bp = blueprint_library.find(
            "sensor.other.collision"
        )

        self.collision_sensor = self.world.spawn_actor(
            collision_bp,
            carla.Transform(),
            attach_to=self.vehicle,
        )

        self.collision_sensor.listen(
            self._on_collision
        )

        return self.collision_sensor

    def _on_collision(self, event):
        """Record a real CARLA collision event."""

        self.collision_events += 1
        self.tracker.record_collision()

    def check_near_collision(
        self,
        obstacle_actors,
        threshold=5.0,
    ):
        """
        Check distance from ego vehicle to nearby actors.

        A near collision is recorded when an obstacle enters
        the specified distance threshold.
        """

        ego_location = self.vehicle.get_location()

        now = time.perf_counter()

        if now - self._last_near_collision_time < self.near_collision_cooldown:
            return

        for actor in obstacle_actors:

            if actor.id == self.vehicle.id:
                continue

            if not actor.is_alive:
                continue

            distance = ego_location.distance(
                actor.get_location()
            )

            if distance <= threshold:
                self.tracker.record_near_collision()
                self._last_near_collision_time = now
                break

    def destroy(self):
        """Destroy the CARLA evaluation sensor."""

        if self.collision_sensor is not None:
            self.collision_sensor.stop()
            self.collision_sensor.destroy()
            self.collision_sensor = None