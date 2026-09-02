from abc import ABC, abstractmethod


class ObjectDetector(ABC):
    """
    Base interface for all object detectors.

    M2 perception implementations such as YOLODetector
    should inherit from this class.
    """

    @abstractmethod
    def detect(self, frame):
        """
        Detect objects in a camera frame.

        Args:
            frame: OpenCV BGR image.

        Returns:
            List of detection dictionaries.
        """
        raise NotImplementedError