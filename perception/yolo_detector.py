import cv2
import numpy as np
from .detector import ObjectDetector

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YOLODetector(ObjectDetector):
    """
    YOLO-based object detector.

    Input:
        OpenCV BGR image.

    Output:
        List of dictionaries containing:
        - class_id
        - class_name
        - confidence
        - bbox
    """

    def __init__(
        self,
        model_path="models/yolo/yolov8n.pt",
        confidence_threshold=0.40,
        device=None,
    ):
        if YOLO is None:
            raise ImportError(
                "Ultralytics is not installed. "
                "Run: pip install ultralytics"
            )

        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.device = device

        self.model = YOLO(model_path)

        self.class_names = self.model.names

    def detect(self, frame):
        """
        Detect objects in one OpenCV frame.

        Args:
            frame: numpy.ndarray in BGR format.

        Returns:
            list[dict]
        """

        if frame is None:
            return []

        if not isinstance(frame, np.ndarray):
            raise TypeError("frame must be a numpy.ndarray")

        if frame.size == 0:
            return []

        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False,
        )

        detections = []

        if not results:
            return detections

        result = results[0]

        if result.boxes is None:
            return detections

        for box in result.boxes:

            xyxy = box.xyxy[0].cpu().numpy()

            x1, y1, x2, y2 = map(int, xyxy)

            confidence = float(box.conf[0].cpu().item())
            class_id = int(box.cls[0].cpu().item())

            class_name = self.class_names.get(
                class_id,
                str(class_id),
            )

            detections.append(
                {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "bbox": [x1, y1, x2, y2],
                }
            )

        return detections