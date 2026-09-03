"""
SIH-2026 Combined Visualization Dashboard.

Combines:
    M2 Perception
    M1 Planning
    Optional CARLA camera frame

This module only visualizes data.
It does not modify M1, M2, or M4 interfaces.
"""

from __future__ import annotations

from typing import Any

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class Dashboard:
    """Combined M1 + M2 visualization dashboard."""

    def __init__(self, figsize=(14, 7)):
        self.figsize = figsize

    @staticmethod
    def _get(data: Any, key: str, default=None):
        """Read a value from either a dictionary or an object."""
        if data is None:
            return default

        if isinstance(data, dict):
            return data.get(key, default)

        return getattr(data, key, default)

    def _draw_perception(
        self,
        ax,
        perception_output: Any,
        camera_frame: Any = None,
    ):
        """Draw M2 perception data."""

        width = int(
            self._get(
                perception_output,
                "image_width",
                640,
            )
        )

        height = int(
            self._get(
                perception_output,
                "image_height",
                480,
            )
        )

        frame_id = self._get(
            perception_output,
            "frame_id",
            0,
        )

        source = self._get(
            perception_output,
            "source",
            "UNKNOWN",
        )

        objects = self._get(
            perception_output,
            "objects",
            [],
        ) or []

        # Camera image if available
        if camera_frame is not None:
            ax.imshow(camera_frame)

        ax.set_xlim(0, width)
        ax.set_ylim(height, 0)

        # Draw detected objects
        for obj in objects:

            bbox = self._get(
                obj,
                "bbox",
            )

            if bbox is None or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = map(
                float,
                bbox,
            )

            rectangle = Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                fill=False,
                linewidth=2,
            )

            ax.add_patch(rectangle)

            class_name = self._get(
                obj,
                "class_name",
                "unknown",
            )

            track_id = self._get(
                obj,
                "track_id",
                "N/A",
            )

            confidence = self._get(
                obj,
                "confidence",
                None,
            )

            distance = self._get(
                obj,
                "distance",
                None,
            )

            label = (
                f"{class_name} | "
                f"ID: {track_id}"
            )

            if confidence is not None:
                label += (
                    f" | Conf: "
                    f"{float(confidence):.2f}"
                )

            if distance is not None:
                label += (
                    f" | Dist: "
                    f"{float(distance):.2f} m"
                )

            ax.text(
                x1,
                max(0, y1 - 5),
                label,
                fontsize=8,
                bbox=dict(
                    boxstyle="round,pad=0.2",
                    alpha=0.8,
                ),
            )

        ax.set_title(
            f"M2 Perception | "
            f"Frame {frame_id} | "
            f"Objects: {len(objects)}"
        )

        ax.set_xlabel(
            "Image X (pixels)"
        )

        ax.set_ylabel(
            "Image Y (pixels)"
        )

    def _draw_planning(
        self,
        ax,
        planning_output: Any,
    ):
        """Draw M1 planning data."""

        waypoints = self._get(
            planning_output,
            "waypoints",
            [],
        ) or []

        action = self._get(
            planning_output,
            "action",
            "UNKNOWN",
        )

        speed = self._get(
            planning_output,
            "target_speed_mps",
            0.0,
        )

        algorithm = self._get(
            planning_output,
            "algorithm",
            "UNKNOWN",
        )

        hazard_count = self._get(
            planning_output,
            "hazard_count",
            0,
        )

        path_safe = self._get(
            planning_output,
            "path_safe",
            False,
        )

        safety_reason = self._get(
            planning_output,
            "safety_reason",
            "UNKNOWN",
        )

        # Convert waypoints into x/y values.
        points = []

        for waypoint in waypoints:

            if isinstance(waypoint, dict):

                if (
                    "x" in waypoint
                    and "y" in waypoint
                ):
                    x = waypoint["x"]
                    y = waypoint["y"]

                elif (
                    "X" in waypoint
                    and "Y" in waypoint
                ):
                    x = waypoint["X"]
                    y = waypoint["Y"]

                elif "position" in waypoint:

                    position = waypoint[
                        "position"
                    ]

                    if len(position) < 2:
                        continue

                    x = position[0]
                    y = position[1]

                else:
                    continue

            else:

                if len(waypoint) < 2:
                    continue

                x = waypoint[0]
                y = waypoint[1]

            points.append(
                (
                    float(x),
                    float(y),
                )
            )

        # Draw path
        if points:

            xs = [p[0] for p in points]
            ys = [p[1] for p in points]

            ax.plot(
                xs,
                ys,
                marker="o",
                linewidth=2,
                label="Planned Path",
            )

            ax.scatter(
                xs[0],
                ys[0],
                s=80,
                label="Start",
            )

            ax.scatter(
                xs[-1],
                ys[-1],
                s=80,
                label="Goal",
            )

            ax.legend()

        ax.set_title(
            "M1 Planning"
        )

        ax.set_xlabel(
            "Map X"
        )

        ax.set_ylabel(
            "Map Y"
        )

        ax.grid(True)

        status = (
            f"Action: {action}\n"
            f"Target speed: "
            f"{float(speed):.2f} m/s\n"
            f"Algorithm: {algorithm}\n"
            f"Hazards: {hazard_count}\n"
            f"Path safe: {path_safe}\n"
            f"Safety: {safety_reason}"
        )

        ax.text(
            0.02,
            0.98,
            status,
            transform=ax.transAxes,
            verticalalignment="top",
            fontsize=9,
            bbox=dict(
                boxstyle="round",
                alpha=0.8,
            ),
        )

    def render(
        self,
        perception_output: Any = None,
        planning_output: Any = None,
        camera_frame: Any = None,
        show: bool = True,
        save_path: str | None = None,
    ):
        """
        Render the combined M1 + M2 dashboard.
        """

        fig, axes = plt.subplots(
            1,
            2,
            figsize=self.figsize,
        )

        # M2 side
        if perception_output is not None:

            self._draw_perception(
                axes[0],
                perception_output,
                camera_frame,
            )

        else:

            axes[0].set_title(
                "M2 Perception — No Data"
            )

            axes[0].text(
                0.5,
                0.5,
                "Waiting for perception data...",
                ha="center",
                va="center",
                transform=axes[0].transAxes,
            )

            axes[0].set_axis_off()

        # M1 side
        if planning_output is not None:

            self._draw_planning(
                axes[1],
                planning_output,
            )

        else:

            axes[1].set_title(
                "M1 Planning — No Data"
            )

            axes[1].text(
                0.5,
                0.5,
                "Waiting for planning data...",
                ha="center",
                va="center",
                transform=axes[1].transAxes,
            )

            axes[1].set_axis_off()

        fig.suptitle(
            "SIH-2026 Autonomous Driving Dashboard",
            fontsize=16,
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
        else:
            # Prevent figures from accumulating during
            # the live CARLA processing loop.
            plt.close(fig)

        return fig


def create_dashboard(
    perception_output: Any = None,
    planning_output: Any = None,
    camera_frame: Any = None,
    show: bool = True,
    save_path: str | None = None,
):
    """Convenience function for creating the dashboard."""

    dashboard = Dashboard()

    return dashboard.render(
        perception_output=perception_output,
        planning_output=planning_output,
        camera_frame=camera_frame,
        show=show,
        save_path=save_path,
    )