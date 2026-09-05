from interfaces.perception_output import PerceptionOutput


def perception_to_planning_input(
    perception_output
):
    """
    M2 PerceptionOutput -> M1 Planning Input
    """


    # ==========================
    # Extract perception data
    # ==========================

    if isinstance(
        perception_output,
        PerceptionOutput
    ):

        objects = (
            perception_output.objects
        )

        hazards = (
            perception_output.hazards
        )

        lidar_obstacles = (
            perception_output.lidar_obstacles
        )


    elif isinstance(
        perception_output,
        dict
    ):

        objects = (
            perception_output.get(
                "objects",
                []
            )
        )

        hazards = (
            perception_output.get(
                "hazards",
                []
            )
        )

        lidar_obstacles = (
            perception_output.get(
                "lidar_obstacles",
                []
            )
        )


    else:

        raise TypeError(
            "perception_output must be PerceptionOutput or dict"
        )



    # ==========================
    # Objects
    # ==========================

    primary_objects = []
    obstacle_positions = []


    for obj in objects:

        if isinstance(
            obj,
            dict
        ):

            bbox = list(
                obj.get(
                    "bbox",
                    []
                )
            )

            position = obj.get(
                "position",
                obj.get(
                    "center"
                )
            )

            if position is None and len(bbox) == 4:

                position = [
                    (bbox[0] + bbox[2]) / 2.0,
                    (bbox[1] + bbox[3]) / 2.0,
                ]

            primary_objects.append(
                {
                    "track_id": int(
                        obj.get(
                            "track_id",
                            -1
                        )
                    ),

                    "class_id": int(
                        obj.get(
                            "class_id",
                            -1
                        )
                    ),

                    "class_name": str(
                        obj.get(
                            "class_name",
                            "unknown"
                        )
                    ),

                    "confidence": float(
                        obj.get(
                            "confidence",
                            0.0
                        )
                    ),

                    "bbox": list(
                        bbox
                    ),

                    "distance": obj.get(
                        "distance"
                    ),

                    "position": position,
                }
            )

            obstacle_positions.append(
                {
                    "track_id": int(
                        obj.get(
                            "track_id",
                            -1
                        )
                    ),
                    "class_name": str(
                        obj.get(
                            "class_name",
                            "unknown"
                        )
                    ),
                    "distance": obj.get(
                        "distance"
                    ),
                    "confidence": float(
                        obj.get(
                            "confidence",
                            0.0
                        )
                    ),
                    "position": position,
                }
            )


        else:

            position = getattr(
                obj,
                "position",
                None
            )

            if position is None:
                position = obj.center

            primary_objects.append(
                {
                    "track_id": obj.track_id,
                    "class_id": obj.class_id,
                    "class_name": obj.class_name,
                    "confidence": obj.confidence,
                    "bbox": list(obj.bbox),
                    "distance": obj.distance,
                    "position": position,
                }
            )

            obstacle_positions.append(
                {
                    "track_id": obj.track_id,
                    "class_name": obj.class_name,
                    "distance": obj.distance,
                    "confidence": obj.confidence,
                    "position": position,
                }
            )



    # ==========================
    # Hazards / potholes
    # ==========================

    fallback_anomalies = []


    for hazard in hazards:


        if isinstance(
            hazard,
            dict
        ):

            fallback_anomalies.append(
                hazard
            )


        else:

            fallback_anomalies.append(
                {
                    "class_name": hazard.hazard_type,
                    "confidence": hazard.confidence,
                    "bbox": list(hazard.bbox),
                    "distance": hazard.distance,
                }
            )



    # ==========================
    # Output for M1
    # ==========================

    return {

        "primary_objects":
            primary_objects,


        "fallback_anomalies":
            fallback_anomalies,


        "lidar_obstacles":
            lidar_obstacles,


        "obstacle_positions":
            obstacle_positions,


        "drivable_space":
            {
                "obstacle_occupied_cells":
                    0
            },


        "confidence_uncertainty":
            {},


        # M0 overwrites these
        "start":
            [0, 0],


        "goal":
            [10, 10],
    }