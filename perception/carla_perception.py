class CarlaPerception:
    """
    Connects M4 SensorManager camera output
    to the M2 perception pipeline.
    """

    def __init__(self, sensor_manager, perception_pipeline):
        if sensor_manager is None:
            raise ValueError(
                "sensor_manager is required"
            )

        if perception_pipeline is None:
            raise ValueError(
                "perception_pipeline is required"
            )

        self.sensor_manager = sensor_manager
        self.pipeline = perception_pipeline

    def process_latest_frame(
        self,
        sensor_name="rgb_camera",
    ):
        """
        Process the latest RGB frame provided by M4.
        """

        frame_data = self.sensor_manager.get_latest_data(
            sensor_name
        )

        if frame_data is None:
            return None

        frame = frame_data["data"]

        perception = self.pipeline.process_frame(
            frame
        )

        return perception