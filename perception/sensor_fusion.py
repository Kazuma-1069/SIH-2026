class SensorFusion:
    """
    Combines all M2 perception outputs
    into one environment representation.
    """

    def fuse(
        self,
        objects,
        hazards,
        lidar_obstacles,
        road_edges,
        drivable_mask=None,
    ):

        return {
            "objects": objects,
            "hazards": hazards,
            "lidar_obstacles": lidar_obstacles,
            "road_edges": road_edges,
            "drivable_mask": drivable_mask,
        }