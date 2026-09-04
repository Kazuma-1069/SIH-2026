from planning.obstacle_map import ObstacleMap
from planning.astar import AStarPlanner
from planning.dijkstra import DijkstraPlanner
from planning.path_optimizer import PathOptimizer
from planning.safety_checker import SafetyChecker,BubbleShield


class Planner:
    """
    High-level M1 planning pipeline.

    Pipeline:
        Perception data
            -> ObstacleMap
            -> Current-path check
            -> A*
            -> Dijkstra fallback
            -> PathOptimizer
            -> SafetyChecker
            -> Planning output

    Dynamic replanning:
        If the current path becomes blocked, M1 generates a new path
        while preserving the original destination.
    """

    def __init__(self, width=20, height=20):
        self.obstacle_map = ObstacleMap(width, height)

        self.astar = AStarPlanner(self.obstacle_map)
        self.dijkstra = DijkstraPlanner(self.obstacle_map)
        self.optimizer = PathOptimizer(self.obstacle_map)
        self.safety_checker = SafetyChecker(self.obstacle_map)
        self.bubble_shield = BubbleShield(
            self.obstacle_map,
            radius=2.0,
            emergency_radius=1.0,
        )

        self.default_speed_mps = 5.0
        self.reduced_speed_mps = 2.0

        # Remember the previously generated path.
        self.current_path = []

        # Keep track of the number of replanning events.
        self.replan_count = 0

    def _normalize_point(self, point, name):
        """Convert a point to an integer grid coordinate."""
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(
                f"{name} must contain exactly two coordinates"
            )

        return (int(point[0]), int(point[1]))

    def _find_path(self, start, goal):
        """
        Generate a path using A* with Dijkstra as fallback.

        Returns:
            (path, algorithm)
        """

        raw_path = self.astar.find_path(start, goal)
        algorithm = "A_STAR"

        if not raw_path:
            raw_path = self.dijkstra.find_path(start, goal)
            algorithm = "DIJKSTRA"

        return raw_path, algorithm

    def _is_current_path_blocked(self):
        """Check whether the currently active path is blocked."""
        if not self.current_path:
            return False

        return self.obstacle_map.is_path_blocked(
            self.current_path
        )

    def plan(self, perception_data):
        """
        Generate a safe path and high-level driving decision.

        Expected perception_data:
        {
            "primary_objects": [],
            "fallback_anomalies": [],
            "drivable_space": {},
            "confidence_uncertainty": {}
        }

        Optional:
        {
            "start": [0, 0],
            "goal": [10, 10],
            "current_path": [[0, 0], [1, 0], ...]
        }
        """

        if not isinstance(perception_data, dict):
            raise TypeError(
                "perception_data must be a dictionary"
            )

        primary_objects = perception_data.get(
            "primary_objects",
            []
        )

        fallback_anomalies = perception_data.get(
            "fallback_anomalies",
            []
        )

        drivable_space = perception_data.get(
            "drivable_space",
            {}
        )

        confidence = perception_data.get(
            "confidence_uncertainty",
            {}
        )

        # Combine all perceived hazards.
        hazards = primary_objects + fallback_anomalies

        # Update the occupancy grid from perception.
        self.obstacle_map.update_from_objects(
            hazards
        )

        # Start and destination.
        ego_position = self._normalize_point(
            perception_data.get(
                "ego_position",
                perception_data.get("start", [0, 0])
            ),
            "ego_position"
        )

        goal = self._normalize_point(
            perception_data.get(
                "goal",
                [10, 10]
            ),
            "goal"
        )

        start = ego_position

        # Check if the currently active route is blocked before
        # evaluating the bubble shield emergency state.
        current_path_blocked = self._is_current_path_blocked()

        # Bubble Shield check around the current ego position.
        bubble_result = self.bubble_shield.check(ego_position)

        if bubble_result["emergency"]:
            self.current_path = []

            return {
                "action": "STOP",
                "target_speed_mps": 0.0,
                "algorithm": "BUBBLE_SHIELD",
                "hazard_count": len(hazards),
                "waypoints": [],
                "path_safe": False,
                "safety_reason": "BUBBLE_SHIELD_EMERGENCY",
                "confidence_uncertainty": confidence,
                "replanned": False,
                "current_path_blocked": current_path_blocked,
                "replan_count": self.replan_count,
                "destination": list(goal),
                "bubble_safe": False,
                "bubble_emergency": True,
                "bubble_distance": bubble_result["distance"],
                "bubble_reason": bubble_result["reason"],
            }

        # Allow the integration layer to provide the
        # currently active path.
        supplied_current_path = perception_data.get(
            "current_path"
        )

        if supplied_current_path is not None:
            self.current_path = [
                self._normalize_point(
                    point,
                    "current_path point"
                )
                for point in supplied_current_path
            ]

        obstacle_cells = drivable_space.get(
            "obstacle_occupied_cells",
            0
        )

        if not isinstance(
            obstacle_cells,
            (int, float)
        ):
            obstacle_cells = 0

        # Determine whether an active route has become unsafe.
        current_path_blocked = (
            self._is_current_path_blocked()
        )

        # A fresh hazard with no active path also requires
        # obstacle-aware planning.
        hazard_present = (
            len(hazards) > 0
            or obstacle_cells > 0
        )

        # Decide whether a replan is required.
        replanning_required = (
            current_path_blocked
            or (
                hazard_present
                and not self.current_path
            )
        )

        # Generate a new path.
        #
        # For a first plan, this creates the initial route.
        # For a blocked current route, this creates a new
        # route around the updated obstacle map.
        if replanning_required or not self.current_path:
            raw_path, algorithm = self._find_path(
                start,
                goal
            )

            if not raw_path:
                self.current_path = []

                if replanning_required:
                    self.replan_count += 1

                return {
                    "action": "STOP",
                    "target_speed_mps": 0.0,
                    "algorithm": algorithm,
                    "hazard_count": len(hazards),
                    "waypoints": [],
                    "path_safe": False,
                    "safety_reason": "NO_PATH_FOUND",
                    "confidence_uncertainty": confidence,
                    "replanned": replanning_required,
                    "current_path_blocked": current_path_blocked,
                    "replan_count": self.replan_count,
                    "destination": list(goal),
                }

            # Optimize the newly generated route.
            optimized_path = self.optimizer.optimize(
                raw_path
            )

            # Validate generated path with Bubble Shield
            bubble_path_result = self.bubble_shield.check_path(
                optimized_path
            )

            if not bubble_path_result["safe"]:
                return {
                    "action": "STOP",
                    "target_speed_mps": 0.0,
                    "algorithm": algorithm,
                    "hazard_count": len(hazards),
                    "waypoints": [],
                    "path_safe": False,
                    "safety_reason": "BUBBLE_SHIELD_PATH_BLOCKED",
                    "confidence_uncertainty": confidence,
                    "replanned": True,
                    "current_path_blocked": current_path_blocked,
                    "replan_count": self.replan_count,
                    "destination": list(goal)
                }

            # Validate the complete candidate path.
            safety_result = (
                self.safety_checker.validate_path(
                    optimized_path
                )
            )

            # Safety failure means the vehicle must stop.
            if not safety_result["safe"]:
                self.current_path = []

                if replanning_required:
                    self.replan_count += 1

                return {
                    "action": "STOP",
                    "target_speed_mps": 0.0,
                    "algorithm": algorithm,
                    "hazard_count": len(hazards),
                    "waypoints": [],
                    "path_safe": False,
                    "safety_reason": safety_result["reason"],
                    "confidence_uncertainty": confidence,
                    "replanned": replanning_required,
                    "current_path_blocked": current_path_blocked,
                    "replan_count": self.replan_count,
                    "destination": list(goal),
                }

            # Accept the new safe route.
            self.current_path = optimized_path

            if replanning_required:
                self.replan_count += 1

        else:
            # Existing route is still usable.
            algorithm = "CURRENT_PATH"
            optimized_path = self.current_path

            safety_result = (
                self.safety_checker.validate_path(
                    optimized_path
                )
            )

        # Determine vehicle behavior.
        if current_path_blocked:
            action = "REROUTE"
            target_speed = self.reduced_speed_mps

        elif hazard_present:
            action = "SLOW_AND_REROUTE"
            target_speed = self.reduced_speed_mps

        else:
            action = "PROCEED_FORWARD"
            target_speed = self.default_speed_mps

        # Final M1 planning output.
        if "bubble_path_result" not in locals():
            bubble_path_result = {
                "safe": True
            }
        return {
            "action": action,
            "target_speed_mps": target_speed,
            "algorithm": algorithm,
            "hazard_count": len(hazards),
            "waypoints": optimized_path,
            "path_safe": safety_result["safe"],
            "safety_reason": safety_result["reason"],
            "confidence_uncertainty": confidence,

            # Dynamic replanning information.
            "replanned": replanning_required,
            "current_path_blocked": current_path_blocked,
            "replan_count": self.replan_count,

            # Destination is preserved across replanning.
            "destination": list(goal),

            # Bubble Shield status.
            "bubble_safe": bubble_result["safe"],
            "bubble_emergency": bubble_result["emergency"],
            "bubble_distance": bubble_result["distance"],
            "bubble_reason": bubble_result["reason"],
            "bubble_path_safe": bubble_path_result["safe"],
        }