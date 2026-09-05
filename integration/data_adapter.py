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


    for obj in objects:

        if isinstance(
            obj,
            dict
        ):

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
                        obj.get(
                            "bbox",
                            []
                        )
                    ),

                    "distance": obj.get(
                        "distance"
                    ),
                }
            )


        else:

            primary_objects.append(
                {
                    "track_id": obj.track_id,
                    "class_id": obj.class_id,
                    "class_name": obj.class_name,
                    "confidence": obj.confidence,
                    "bbox": list(obj.bbox),
                    "distance": obj.distance,
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


        "drivable_space":
            {
                "obstacle_occupied_cells":
                    len(primary_objects)
                    +
                    len(fallback_anomalies)
            },


        "confidence_uncertainty":
            {},


        # M0 overwrites these
        "start":
            [0, 0],


        "goal":
            [10, 10],
    }