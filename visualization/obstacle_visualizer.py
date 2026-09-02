"""
Obstacle visualization for the SIH-2026 perception pipeline.

Consumes the standardized PerceptionOutput contract from M2.

Supported input:
    - PerceptionOutput dataclass
    - PerceptionOutput.to_dict() result

Visualizes:
    - Bounding boxes
    - Object class
    - Track ID
    - Confidence
    - Distance
    - Frame metadata
"""


from __future__ import annotations

from typing import Any, List, Tuple

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class ObstacleVisualizer:
    """
    Visualizes tracked obstacles detected by M2 perception.

    The visualizer does not modify the M2 perception contract.
    """

    def __init__(self, figsize: Tuple[int, int] = (12, 7)):
        self.figsize = figsize

    @staticmethod
    def _get_value(obj: Any, key: str, default: Any = None) -> Any:
        """Read a field from either a dataclass object or dictionary."""

        if isinstance(obj, dict):
            return obj.get(key, default)

        return getattr(obj, key, default)

    @staticmethod
    def _get_objects(perception_output: Any) -> List[Any]:
        """Extract objects from PerceptionOutput or dictionary."""

        objects = ObstacleVisualizer._get_value(
            perception_output,
            "objects",
            [],
        )

        if objects is None:
            return []

        return list(objects)

    @staticmethod
    def _get_bbox(obj: Any) -> Tuple[float, float, float, float]:
        """Extract and validate [x1, y1, x2, y2] bounding box."""

        bbox = ObstacleVisualizer._get_value(obj, "bbox")

        if bbox is None or len(bbox) != 4:
            raise ValueError(
                f"Invalid bounding box: {bbox}. "
                "Expected [x1, y1, x2, y2]."
            )

        x1, y1, x2, y2 = map(float, bbox)

        return x1, y1, x2, y2

    @staticmethod
    def _format_label(obj: Any) -> str:
        """Build the object annotation shown above the bounding box."""

        class_name = ObstacleVisualizer._get_value(
            obj,
            "class_name",
            "unknown",
        )

        track_id = ObstacleVisualizer._get_value(
            obj,
            "track_id",
            "N/A",
        )

        confidence = ObstacleVisualizer._get_value(
            obj,
            "confidence",
            None,
        )

        distance = ObstacleVisualizer._get_value(
            obj,
            "distance",
            None,
        )

        label = f"{class_name} | ID: {track_id}"

        if confidence is not None:
            label += f" | Conf: {float(confidence):.2f}"

        if distance is not None:
            label += f" | Dist: {float(distance):.2f} m"

        return label

    def plot(
        self,
        perception_output: Any,
        background_image: Any = None,
        show: bool = True,
        save_path: str | None = None,
    ):
        """
        Visualize M2 perception output.

        Args:
            perception_output:
                PerceptionOutput dataclass or dictionary.

            background_image:
                Optional camera image. If provided, bounding boxes are
                drawn over the image. If omitted, a coordinate-space
                visualization is created.

            show:
                Display the matplotlib window when True.

            save_path:
                Optional path for saving the visualization.

        Returns:
            matplotlib Figure object.
        """

        width = int(
            self._get_value(
                perception_output,
                "image_width",
                0,
            )
        )

        height = int(
            self._get_value(
                perception_output,
                "image_height",
                0,
            )
        )

        frame_id = self._get_value(
            perception_output,
            "frame_id",
            0,
        )

        source = self._get_value(
            perception_output,
            "source",
            "UNKNOWN",
        )

        objects = self._get_objects(perception_output)

        fig, ax = plt.subplots(figsize=self.figsize)

        # ---------------------------------------------------------
        # Background camera image
        # ---------------------------------------------------------
        if background_image is not None:
            ax.imshow(background_image)

            if width > 0 and height > 0:
                ax.set_xlim(0, width)
                ax.set_ylim(height, 0)

        else:
            # Coordinate-space mode.
            if width > 0:
                ax.set_xlim(0, width)

            if height > 0:
                ax.set_ylim(height, 0)

            if width == 0 or height == 0:
                ax.set_xlim(0, 640)
                ax.set_ylim(480, 0)

        # ---------------------------------------------------------
        # Draw detected objects
        # ---------------------------------------------------------
        for obj in objects:
            x1, y1, x2, y2 = self._get_bbox(obj)

            rectangle = Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                fill=False,
                linewidth=2,
            )

            ax.add_patch(rectangle)

            label = self._format_label(obj)

            ax.text(
                x1,
                max(0, y1 - 5),
                label,
                fontsize=8,
                verticalalignment="bottom",
                bbox=dict(
                    boxstyle="round,pad=0.2",
                    alpha=0.8,
                ),
            )

        # ---------------------------------------------------------
        # Frame information
        # ---------------------------------------------------------
        ax.set_title(
            f"SIH-2026 — M2 Perception | "
            f"Frame: {frame_id} | Objects: {len(objects)}"
        )

        ax.set_xlabel("Image X (pixels)")
        ax.set_ylabel("Image Y (pixels)")

        ax.text(
            0.02,
            0.02,
            f"Source: {source}",
            transform=ax.transAxes,
            verticalalignment="bottom",
            bbox=dict(
                boxstyle="round",
                alpha=0.8,
            ),
        )

        ax.grid(False)

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


def visualize_perception_output(
    perception_output: Any,
    background_image: Any = None,
    show: bool = True,
    save_path: str | None = None,
):
    """
    Convenience function for visualizing M2 PerceptionOutput.
    """

    visualizer = ObstacleVisualizer()

    return visualizer.plot(
        perception_output=perception_output,
        background_image=background_image,
        show=show,
        save_path=save_path,
    )