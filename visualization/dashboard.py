"""
SIH-2026 Combined Visualization Dashboard.

Displays:
    M2 Perception
    M1 Planning
    M4 Simulation State
    M5 Vehicle Control

This module only visualizes data.
It does not modify M1, M2, M4, or M5 interfaces.
"""

from __future__ import annotations

from typing import Any

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class Dashboard:
    """Combined autonomous driving visualization dashboard."""

    def __init__(self, figsize=(14, 10)):
        self.figsize = figsize

    @staticmethod
    def _get(data: Any, key: str, default=None):
        """Read value from dictionary or object."""

        if data is None:
            return default

        if isinstance(data, dict):
            return data.get(key, default)

        return getattr(data, key, default)


    # --------------------------------------------------
    # M2 PERCEPTION
    # --------------------------------------------------

    def _draw_perception(
        self,
        ax,
        perception_output,
        camera_frame=None,
    ):

        width = int(
            self._get(
                perception_output,
                "image_width",
                640
            )
        )

        height = int(
            self._get(
                perception_output,
                "image_height",
                480
            )
        )

        objects = self._get(
            perception_output,
            "objects",
            []
        ) or []


        if camera_frame is not None:
            ax.imshow(camera_frame)


        ax.set_xlim(0, width)
        ax.set_ylim(height, 0)


        for obj in objects:

            bbox = self._get(
                obj,
                "bbox",
                None
            )

            if bbox is None or len(bbox) != 4:
                continue


            x1, y1, x2, y2 = map(
                float,
                bbox
            )


            rectangle = Rectangle(
                (x1, y1),
                x2-x1,
                y2-y1,
                fill=False,
                linewidth=2
            )

            ax.add_patch(rectangle)


            label = (
                f"{self._get(obj,'class_name','unknown')}"
            )


            confidence = self._get(
                obj,
                "confidence",
                None
            )


            distance = self._get(
                obj,
                "distance",
                None
            )


            if confidence is not None:
                label += (
                    f"\nConf:{float(confidence):.2f}"
                )


            if distance is not None:
                label += (
                    f"\nDist:{float(distance):.1f}m"
                )


            ax.text(
                x1,
                y1,
                label,
                fontsize=8,
                bbox=dict(
                    boxstyle="round",
                    alpha=0.7
                )
            )


        ax.set_title(
            f"M2 Perception\nObjects: {len(objects)}"
        )


    # --------------------------------------------------
    # M1 PLANNING
    # --------------------------------------------------

    def _draw_planning(
        self,
        ax,
        planning_output
    ):

        waypoints = self._get(
            planning_output,
            "waypoints",
            []
        ) or []


        points = []


        for wp in waypoints:

            if isinstance(wp, dict):

                x = wp.get("x")
                y = wp.get("y")

                if x is None:
                    continue

            else:

                if len(wp) < 2:
                    continue

                x = wp[0]
                y = wp[1]


            points.append(
                (
                    float(x),
                    float(y)
                )
            )


        if points:

            xs = [
                p[0]
                for p in points
            ]

            ys = [
                p[1]
                for p in points
            ]


            ax.plot(
                xs,
                ys,
                marker="o",
                linewidth=2,
                label="Path"
            )


            ax.scatter(
                xs[0],
                ys[0],
                s=80,
                label="Start"
            )


            ax.scatter(
                xs[-1],
                ys[-1],
                s=80,
                label="Goal"
            )


            ax.legend()



        safe = self._get(
            planning_output,
            "path_safe",
            False
        )


        reason = self._get(
            planning_output,
            "safety_reason",
            "NONE"
        )


        text = (
            f"Path Safe: {safe}\n"
            f"Safety: {reason}\n"
            f"Hazards: "
            f"{self._get(planning_output,'hazard_count',0)}"
        )


        ax.text(
            0.02,
            0.95,
            text,
            transform=ax.transAxes,
            verticalalignment="top"
        )


        ax.set_title(
            "M1 Planning"
        )

        ax.grid(True)



    # --------------------------------------------------
    # M4 SIMULATION MAP
    # --------------------------------------------------

    def _draw_simulation(
        self,
        ax,
        simulation_output
    ):

        position = self._get(
            simulation_output,
            "vehicle_position",
            None
        )


        destination = self._get(
            simulation_output,
            "destination",
            None
        )


        if position:

            ax.scatter(
                position[0],
                position[1],
                s=120,
                label="Vehicle"
            )


        if destination:

            ax.scatter(
                destination[0],
                destination[1],
                s=120,
                label="Destination"
            )


        ax.legend()


        ax.set_title(
            "M4 Simulation"
        )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True)



    # --------------------------------------------------
    # M5 CONTROL + TELEMETRY
    # --------------------------------------------------

    def _draw_vehicle_status(
        self,
        ax,
        simulation_output,
        control_output
    ):

        speed = self._get(
            simulation_output,
            "speed",
            0
        )


        position = self._get(
            simulation_output,
            "vehicle_position",
            "UNKNOWN"
        )


        throttle = self._get(
            control_output,
            "throttle",
            0
        )


        brake = self._get(
            control_output,
            "brake",
            0
        )


        steering = self._get(
            control_output,
            "steering",
            0
        )


        risk = self._get(
            simulation_output,
            "risk_level",
            "UNKNOWN"
        )


        text = (
            "Vehicle Status\n\n"
            f"Position: {position}\n"
            f"Speed: {speed}\n\n"
            f"Throttle: {throttle}\n"
            f"Brake: {brake}\n"
            f"Steering: {steering}\n\n"
            f"Risk: {risk}"
        )


        ax.text(
            0.05,
            0.95,
            text,
            transform=ax.transAxes,
            verticalalignment="top"
        )


        ax.set_title(
            "M5 Control / Safety"
        )


        ax.axis("off")



    # --------------------------------------------------
    # MAIN RENDER
    # --------------------------------------------------

    def render(
        self,
        simulation_output=None,
        perception_output=None,
        planning_output=None,
        control_output=None,
        camera_frame=None,
        show=True,
        save_path=None
    ):


        fig, axes = plt.subplots(
            2,
            2,
            figsize=self.figsize
        )


        # M2

        if perception_output:

            self._draw_perception(
                axes[0,0],
                perception_output,
                camera_frame
            )


        else:

            axes[0,0].set_title(
                "M2 Perception Waiting"
            )



        # M1

        if planning_output:

            self._draw_planning(
                axes[0,1],
                planning_output
            )


        else:

            axes[0,1].set_title(
                "M1 Planning Waiting"
            )



        # M4

        if simulation_output:

            self._draw_simulation(
                axes[1,0],
                simulation_output
            )


        else:

            axes[1,0].set_title(
                "M4 Simulation Waiting"
            )



        # M5

        self._draw_vehicle_status(
            axes[1,1],
            simulation_output,
            control_output
        )



        fig.suptitle(
            "SIH-2026 Autonomous Driving Dashboard",
            fontsize=16
        )


        fig.tight_layout()


        if save_path:

            fig.savefig(
                save_path,
                dpi=150
            )


        if show:

            plt.show()

        else:

            plt.close(fig)


        return fig



def create_dashboard(
    simulation_output=None,
    perception_output=None,
    planning_output=None,
    control_output=None,
    camera_frame=None,
    show=True,
    save_path=None
):

    dashboard = Dashboard()


    return dashboard.render(
        simulation_output,
        perception_output,
        planning_output,
        control_output,
        camera_frame,
        show,
        save_path
    )