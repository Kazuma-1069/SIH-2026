"""
Path visualization for the SIH-2026 planning pipeline.

Consumes the standardized PlanningOutput contract from M1
and visualizes the planned path.

Expected PlanningOutput fields:
    - waypoints
    - path_safe
    - action
    - target_speed_mps
    - algorithm
    - safety_reason
"""


from __future__ import annotations

from typing import Any, Iterable, List, Tuple

import matplotlib.pyplot as plt


Point = Tuple[float, float]


class PathVisualizer:
    """
    Visualizes the planned path produced by the M1 planning module.

    The visualizer accepts the complete PlanningOutput dictionary rather
    than creating a separate M3-specific planning data structure.
    """

    def __init__(self, figsize: Tuple[int, int] = (10, 7)):
        self.figsize = figsize

    @staticmethod
    def _extract_point(waypoint: Any) -> Point:
        """
        Convert one waypoint into an (x, y) point.

        Supported formats:
            [x, y]
            (x, y)
            {"x": x, "y": y}
            {"X": x, "Y": y}
            {"position": [x, y]}
            {"position": {"x": x, "y": y}}
        """

        if isinstance(waypoint, dict):
            if "x" in waypoint and "y" in waypoint:
                return float(waypoint["x"]), float(waypoint["y"])

            if "X" in waypoint and "Y" in waypoint:
                return float(waypoint["X"]), float(waypoint["Y"])

            if "position" in waypoint:
                return PathVisualizer._extract_point(waypoint["position"])

            raise ValueError(
                f"Unsupported waypoint dictionary format: {waypoint}"
            )

        if isinstance(waypoint, (list, tuple)):
            if len(waypoint) < 2:
                raise ValueError(
                    f"Waypoint must contain at least x and y: {waypoint}"
                )

            return float(waypoint[0]), float(waypoint[1])

        raise ValueError(f"Unsupported waypoint type: {type(waypoint)}")

    def _extract_points(self, planning_output: dict) -> List[Point]:
        """
        Extract all path points from standardized PlanningOutput.
        """

        if not isinstance(planning_output, dict):
            raise TypeError("planning_output must be a dictionary")

        waypoints: Iterable[Any] = planning_output.get("waypoints", [])

        if waypoints is None:
            return []

        return [self._extract_point(waypoint) for waypoint in waypoints]

    def plot(
        self,
        planning_output: dict,
        show: bool = True,
        save_path: str | None = None,
    ):
        """
        Plot the planned path from PlanningOutput.

        Args:
            planning_output:
                Standardized PlanningOutput dictionary from M1.

            show:
                Display the matplotlib window when True.

            save_path:
                Optional path for saving the generated visualization.

        Returns:
            matplotlib Figure object.
        """

        points = self._extract_points(planning_output)

        fig, ax = plt.subplots(figsize=self.figsize)

        if points:
            x_values = [point[0] for point in points]
            y_values = [point[1] for point in points]

            # Planned path
            ax.plot(
                x_values,
                y_values,
                marker="o",
                linewidth=2,
                label="Planned Path",
            )

            # Start point
            ax.scatter(
                x_values[0],
                y_values[0],
                s=100,
                marker="o",
                label="Start",
                zorder=3,
            )

            # Goal point
            ax.scatter(
                x_values[-1],
                y_values[-1],
                s=100,
                marker="X",
                label="Goal",
                zorder=3,
            )

        path_safe = planning_output.get("path_safe", False)
        action = planning_output.get("action", "UNKNOWN")
        target_speed = planning_output.get("target_speed_mps", 0.0)
        algorithm = planning_output.get("algorithm", "UNKNOWN")
        hazard_count = planning_output.get("hazard_count", 0)
        safety_reason = planning_output.get(
            "safety_reason",
            "UNKNOWN",
        )

        ax.set_title("SIH-2026 — Planned Path")

        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")

        ax.grid(True)
        ax.axis("equal")

        status = "SAFE" if path_safe else "UNSAFE"

        information = (
            f"Status: {status}\n"
            f"Action: {action}\n"
            f"Target Speed: {target_speed:.2f} m/s\n"
            f"Algorithm: {algorithm}\n"
            f"Hazards: {hazard_count}\n"
            f"Reason: {safety_reason}"
        )

        ax.text(
            0.02,
            0.98,
            information,
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(
                boxstyle="round",
                alpha=0.85,
            ),
        )

        ax.legend()
        fig.tight_layout()

        if save_path:
            fig.savefig(
                save_path,
                dpi=150,
                bbox_inches="tight",
            )

        if show:
            plt.show()

        return fig


def visualize_planning_output(
    planning_output: dict,
    show: bool = True,
    save_path: str | None = None,
):
    """
    Convenience function for visualizing a PlanningOutput dictionary.
    """

    visualizer = PathVisualizer()

    return visualizer.plot(
        planning_output=planning_output,
        show=show,
        save_path=save_path,
    )