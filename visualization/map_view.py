"""
2D map visualization for the SIH-2026 autonomous driving pipeline.

Consumes:
    - M1 planning output
    - M2 perception output

The map view is intentionally independent of CARLA internals.
It visualizes standardized project interfaces so it can later be
connected to the CARLA simulation through the integration layer.
"""

from __future__ import annotations

from typing import Any, Iterable, List, Tuple

import matplotlib.pyplot as plt


class MapView:
    """
    Visualize the vehicle's planning environment in 2D.

    Supported planning waypoint formats:
        - (x, y)
        - [x, y]
        - {"x": x, "y": y}
        - {"X": x, "Y": y}
        - {"position": [x, y]}

    Supported perception object position:
        - object["position"] = [x, y]
        - object["location"] = [x, y]
        - object["center"] = [x, y]

    If M2 only provides image bounding boxes, those are not converted
    into world coordinates here. World-coordinate conversion belongs
    to the perception/integration layer.
    """

    def __init__(
        self,
        figsize: Tuple[int, int] = (10, 8),
        grid_size: Tuple[int, int] = (20, 20),
    ):
        self.figsize = figsize
        self.grid_width = grid_size[0]
        self.grid_height = grid_size[1]

    @staticmethod
    def _get_value(
        obj: Any,
        key: str,
        default: Any = None,
    ) -> Any:
        """Read a value from either a dictionary or an object."""

        if isinstance(obj, dict):
            return obj.get(key, default)

        return getattr(obj, key, default)

    @staticmethod
    def _extract_xy(point: Any) -> Tuple[float, float] | None:
        """Convert supported point formats into an (x, y) tuple."""

        if point is None:
            return None

        if isinstance(point, dict):
            if "x" in point and "y" in point:
                return float(point["x"]), float(point["y"])

            if "X" in point and "Y" in point:
                return float(point["X"]), float(point["Y"])

            if "position" in point:
                return MapView._extract_xy(point["position"])

            if "location" in point:
                return MapView._extract_xy(point["location"])

        if hasattr(point, "x") and hasattr(point, "y"):
            return float(point.x), float(point.y)

        if isinstance(point, (list, tuple)) and len(point) >= 2:
            return float(point[0]), float(point[1])

        return None

    def _extract_waypoints(
        self,
        planning_output: Any,
    ) -> List[Tuple[float, float]]:
        """Extract M1 waypoints."""

        waypoints = self._get_value(
            planning_output,
            "waypoints",
            [],
        )

        result = []

        for waypoint in waypoints or []:
            point = self._extract_xy(waypoint)

            if point is not None:
                result.append(point)

        return result

    def _extract_object_positions(
        self,
        perception_output: Any,
    ) -> List[Tuple[float, float, Any]]:
        """
        Extract world-coordinate obstacle positions.

        Returns:
            (x, y, original_object)
        """

        objects = self._get_value(
            perception_output,
            "objects",
            [],
        )

        result = []

        for obj in objects or []:
            position = self._get_value(obj, "position")

            if position is None:
                position = self._get_value(obj, "location")

            if position is None:
                continue

            point = self._extract_xy(position)

            if point is not None:
                result.append(
                    (
                        point[0],
                        point[1],
                        obj,
                    )
                )

        return result

    @staticmethod
    def _object_label(obj: Any) -> str:
        """Create a compact obstacle label."""

        class_name = MapView._get_value(
            obj,
            "class_name",
            "obstacle",
        )

        track_id = MapView._get_value(
            obj,
            "track_id",
            None,
        )

        if track_id is not None:
            return f"{class_name} #{track_id}"

        return str(class_name)

    def render(
        self,
        planning_output: Any = None,
        perception_output: Any = None,
        vehicle_position: Any = None,
        start: Any = None,
        goal: Any = None,
        show: bool = True,
        save_path: str | None = None,
    ):
        """
        Render the 2D autonomous-driving map.

        Args:
            planning_output:
                M1 planning output dictionary.

            perception_output:
                M2 PerceptionOutput dataclass or dictionary.

            vehicle_position:
                Optional current vehicle position.

            start:
                Optional path start position. If omitted, uses the
                first planning waypoint.

            goal:
                Optional path goal position. If omitted, uses the
                final planning waypoint.

            show:
                Display the matplotlib window when True.

            save_path:
                Optional path for saving the figure.

        Returns:
            matplotlib Figure.
        """

        fig, ax = plt.subplots(figsize=self.figsize)

        # ---------------------------------------------------------
        # Planning path
        # ---------------------------------------------------------

        waypoints = []

        if planning_output is not None:
            waypoints = self._extract_waypoints(
                planning_output
            )

        if waypoints:
            xs = [point[0] for point in waypoints]
            ys = [point[1] for point in waypoints]

            ax.plot(
                xs,
                ys,
                marker="o",
                linewidth=2,
                label="Planned path",
            )

            # Start and goal default to first/last waypoint.
            if start is None:
                start = waypoints[0]

            if goal is None:
                goal = waypoints[-1]

        # ---------------------------------------------------------
        # Start position
        # ---------------------------------------------------------

        start_xy = self._extract_xy(start)

        if start_xy is not None:
            ax.scatter(
                [start_xy[0]],
                [start_xy[1]],
                marker="o",
                s=90,
                label="Start",
                zorder=5,
            )

        # ---------------------------------------------------------
        # Goal position
        # ---------------------------------------------------------

        goal_xy = self._extract_xy(goal)

        if goal_xy is not None:
            ax.scatter(
                [goal_xy[0]],
                [goal_xy[1]],
                marker="*",
                s=160,
                label="Goal",
                zorder=5,
            )

        # ---------------------------------------------------------
        # Vehicle position
        # ---------------------------------------------------------

        vehicle_xy = self._extract_xy(
            vehicle_position
        )

        if vehicle_xy is not None:
            ax.scatter(
                [vehicle_xy[0]],
                [vehicle_xy[1]],
                marker="s",
                s=100,
                label="Vehicle",
                zorder=6,
            )

        # ---------------------------------------------------------
        # M2 world-coordinate obstacles
        # ---------------------------------------------------------

        obstacles = []

        if perception_output is not None:
            obstacles = self._extract_object_positions(
                perception_output
            )

        for x, y, obj in obstacles:
            ax.scatter(
                [x],
                [y],
                marker="x",
                s=100,
                zorder=5,
            )

            ax.annotate(
                self._object_label(obj),
                (x, y),
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=8,
            )

        # ---------------------------------------------------------
        # Planning metadata
        # ---------------------------------------------------------

        action = "UNKNOWN"
        speed = None
        algorithm = "UNKNOWN"
        path_safe = None
        safety_reason = ""

        if planning_output is not None:
            action = self._get_value(
                planning_output,
                "action",
                "UNKNOWN",
            )

            speed = self._get_value(
                planning_output,
                "target_speed_mps",
            )

            algorithm = self._get_value(
                planning_output,
                "algorithm",
                "UNKNOWN",
            )

            path_safe = self._get_value(
                planning_output,
                "path_safe",
            )

            safety_reason = self._get_value(
                planning_output,
                "safety_reason",
                "",
            )

        metadata = (
            f"Action: {action}\n"
            f"Algorithm: {algorithm}"
        )

        if speed is not None:
            metadata += f"\nTarget speed: {float(speed):.2f} m/s"

        if path_safe is not None:
            metadata += f"\nPath safe: {path_safe}"

        if safety_reason:
            metadata += f"\nReason: {safety_reason}"

        ax.text(
            0.02,
            0.98,
            metadata,
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(
                boxstyle="round,pad=0.4",
                alpha=0.85,
            ),
        )

        # ---------------------------------------------------------
        # Map formatting
        # ---------------------------------------------------------

        ax.set_title(
            "SIH-2026 — Autonomous Driving Map View"
        )

        ax.set_xlabel("World X")
        ax.set_ylabel("World Y")

        ax.set_xlim(
            0,
            self.grid_width,
        )

        ax.set_ylim(
            0,
            self.grid_height,
        )

        ax.set_aspect(
            "equal",
            adjustable="box",
        )

        ax.grid(True)

        handles, labels = ax.get_legend_handles_labels()

        if handles:
            ax.legend(
                loc="lower right"
            )

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


def visualize_map(
    planning_output: Any = None,
    perception_output: Any = None,
    vehicle_position: Any = None,
    start: Any = None,
    goal: Any = None,
    show: bool = True,
    save_path: str | None = None,
):
    """Convenience function for rendering the project map."""

    view = MapView()

    return view.render(
        planning_output=planning_output,
        perception_output=perception_output,
        vehicle_position=vehicle_position,
        start=start,
        goal=goal,
        show=show,
        save_path=save_path,
    )