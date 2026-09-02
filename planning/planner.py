from planning.obstacle_map import ObstacleMap
from planning.astar import AStarPlanner
from planning.dijkstra import DijkstraPlanner
from planning.path_optimizer import PathOptimizer
from planning.safety_checker import SafetyChecker


class Planner:
    """
    High-level M1 planning pipeline.

    Pipeline:
        Perception data
            -> ObstacleMap
            -> A*
            -> Dijkstra fallback
            -> PathOptimizer
            -> SafetyChecker
            -> Planning output
    """

    def __init__(self, width=20, height=20):
        self.obstacle_map = ObstacleMap(width, height)

        self.astar = AStarPlanner(self.obstacle_map)
        self.dijkstra = DijkstraPlanner(self.obstacle_map)
        self.optimizer = PathOptimizer(self.obstacle_map)
        self.safety_checker = SafetyChecker(self.obstacle_map)

        self.default_speed_mps = 5.0
        self.reduced_speed_mps = 2.0

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
            "goal": [10, 10]
        }
        """

        primary_objects = perception_data.get(
            "primary_objects", []
        )

        fallback_anomalies = perception_data.get(
            "fallback_anomalies", []
        )

        drivable_space = perception_data.get(
            "drivable_space", {}
        )

        confidence = perception_data.get(
            "confidence_uncertainty", {}
        )

        # Combine all perceived obstacles.
        hazards = primary_objects + fallback_anomalies

        # Update occupancy grid.
        self.obstacle_map.update_from_objects(hazards)

        # Default start and goal for the prototype.
        start = tuple(
            perception_data.get("start", [0, 0])
        )

        goal = tuple(
            perception_data.get("goal", [10, 10])
        )

        obstacle_cells = drivable_space.get(
            "obstacle_occupied_cells", 0
        )

        # Select planning behavior.
        if hazards or obstacle_cells > 0:
            action = "SLOW_AND_REROUTE"
            target_speed = self.reduced_speed_mps
        else:
            action = "PROCEED_FORWARD"
            target_speed = self.default_speed_mps

        # Try A* first.
        raw_path = self.astar.find_path(start, goal)
        algorithm = "A_STAR"

        # Dijkstra is the fallback planner.
        if not raw_path:
            raw_path = self.dijkstra.find_path(start, goal)
            algorithm = "DIJKSTRA"

        # Optimize the path.
        optimized_path = self.optimizer.optimize(
            raw_path
        )

        # Safety validation.
        safety_result = self.safety_checker.validate_path(
            optimized_path
        )

        # If the path is unsafe, don't claim it is valid.
        if not safety_result["safe"]:
            action = "STOP"
            target_speed = 0.0

        return {
            "action": action,
            "target_speed_mps": target_speed,
            "algorithm": algorithm,
            "hazard_count": len(hazards),
            "waypoints": optimized_path,
            "path_safe": safety_result["safe"],
            "safety_reason": safety_result["reason"],
            "confidence_uncertainty": confidence
        }