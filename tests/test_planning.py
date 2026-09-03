import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planning.obstacle_map import ObstacleMap
from planning.astar import AStarPlanner
from planning.dijkstra import DijkstraPlanner


def make_map():
    return ObstacleMap(width=5, height=5)


def test_astar_finds_path():
    obstacle_map = make_map()
    planner = AStarPlanner(obstacle_map)

    path = planner.find_path((0, 0), (4, 4))

    assert path
    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)


def test_astar_avoids_obstacle():
    obstacle_map = make_map()

    # Block the direct route.
    obstacle_map.set_obstacle((1, 0), True)

    planner = AStarPlanner(obstacle_map)
    path = planner.find_path((0, 0), (2, 0))

    assert path
    assert (1, 0) not in path


def test_astar_returns_empty_for_blocked_goal():
    obstacle_map = make_map()
    obstacle_map.set_obstacle((4, 4), True)

    planner = AStarPlanner(obstacle_map)
    path = planner.find_path((0, 0), (4, 4))

    assert path == []


def test_dijkstra_finds_path():
    obstacle_map = make_map()
    planner = DijkstraPlanner(obstacle_map)

    path = planner.find_path((0, 0), (4, 4))

    assert path
    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)


def test_dijkstra_avoids_obstacle():
    obstacle_map = make_map()

    obstacle_map.set_obstacle((1, 0), True)

    planner = DijkstraPlanner(obstacle_map)
    path = planner.find_path((0, 0), (2, 0))

    assert path
    assert (1, 0) not in path


def test_dijkstra_returns_empty_for_blocked_goal():
    obstacle_map = make_map()
    obstacle_map.set_obstacle((4, 4), True)

    planner = DijkstraPlanner(obstacle_map)
    path = planner.find_path((0, 0), (4, 4))

    assert path == []