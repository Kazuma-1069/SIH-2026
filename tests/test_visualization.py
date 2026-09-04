import sys
import os

sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            ".."
        )
    )
)
import matplotlib

matplotlib.use("Agg")

from visualization.dashboard import Dashboard


def test_dashboard_render():

    dashboard = Dashboard()

    perception_output = {
        "objects": [
            {
                "bbox": [10, 20, 100, 120],
                "class_name": "car",
                "confidence": 0.9,
                "distance": 20
            }
        ]
    }


    planning_output = {
        "waypoints": [
            [0, 0],
            [10, 10],
            [20, 20]
        ],
        "path_safe": True,
        "safety_reason": "clear"
    }


    simulation_output = {
        "vehicle_position": [5, 5],
        "destination": [50, 50],
        "speed": 10
    }


    control_output = {
        "throttle": 0.5,
        "brake": 0,
        "steering": 0.1
    }


    fig = dashboard.render(
        simulation_output=simulation_output,
        perception_output=perception_output,
        planning_output=planning_output,
        control_output=control_output,
        show=False
    )


    assert fig is not None