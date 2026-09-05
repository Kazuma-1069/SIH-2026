"""
SIH-2026 Integration Pipeline

Full autonomous driving data flow:

CARLA/M4
   ↓
M2 Perception
   ↓
M0 Navigation State
   ↓
M1 Planning
   ↓
M5 Vehicle Control
   ↓
CARLA Vehicle
   ↓
M3 Visualization
"""

from integration.data_adapter import (
    perception_to_planning_input
)

from planning.coordinate_adapter import (
    CoordinateAdapter
)


class IntegrationPipeline:


    def __init__(
        self,
        perception_pipeline,
        planner,
        controller=None,
        dashboard=None,
        vehicle=None,
        destination=None,
    ):

        # M2
        self.perception_pipeline = (
            perception_pipeline
        )

        # M1
        self.planner = planner

        # M5
        self.controller = controller

        # M3
        self.dashboard = dashboard

        # M4 VehicleManager
        self.vehicle = vehicle

        # M0 destination
        self.destination = destination

        # CARLA -> Planner grid
        self.coordinate_adapter = (
            CoordinateAdapter()
        )


    def process_frame(
        self,
        frame,
        show=False,
        save_path=None,
    ):


        # ==========================
        # M2 PERCEPTION
        # ==========================

        perception_output = (
            self.perception_pipeline
            .process_frame(frame)
        )


        # ==========================
        # M2 -> M1
        # ==========================

        planning_input = (
            perception_to_planning_input(
                perception_output
            )
        )


        # ==========================
        # M0 NAVIGATION
        # ==========================

        ego_position = None


        if self.vehicle is not None:

            location = (
                self.vehicle.get_location()
            )

            if location is not None:

                ego_position = (
                    self.coordinate_adapter
                    .world_to_grid(
                        [
                            location.x,
                            location.y,
                        ]
                    )
                )

                planning_input[
                    "ego_position"
                ] = ego_position



        # Destination -> planner goal

        if self.destination is not None:

            planning_input[
                "goal"
            ] = (
                self.coordinate_adapter
                .world_to_grid(
                    [
                        self.destination.x,
                        self.destination.y,
                    ]
                )
            )



        # ==========================
        # M0 DESTINATION CHECK
        # ==========================

        destination_reached = False


        if self.vehicle is not None:

            destination_reached = (
                self.vehicle
                .has_reached_destination()
            )



        # ==========================
        # M1 PLANNING
        # ==========================

        planning_output = (
            self.planner.plan(
                planning_input
            )
        )
        print(
            "\n========== PLANNER DEBUG =========="
        )

        print(
            "PLANNING INPUT:"
        )

        print(
            planning_input
        )

        print(
            "PLANNING OUTPUT:"
        )

        print(
            planning_output
        )

        print(
            "=================================="
        )

        planning_output[
            "destination_reached"
        ] = destination_reached



        # ==========================
        # M5 CONTROL
        # ==========================

        control_command = None


        if destination_reached:

            control_command = {

                "throttle": 0.0,

                "steer": 0.0,

                "brake": 1.0,

            }


        elif self.controller is not None:


            control_command = (
                self.controller
                .compute_control(
                    planning_output,
                    ego_position,
                )
            )



        # ==========================
        # M5 -> M4
        # ==========================

        if (
            self.vehicle is not None
            and control_command is not None
        ):


            self.vehicle.apply_control(

                throttle=(
                    control_command[
                        "throttle"
                    ]
                ),

                steer=(
                    control_command[
                        "steer"
                    ]
                ),

                brake=(
                    control_command[
                        "brake"
                    ]
                ),

            )



        # ==========================
        # M3 VISUALIZATION
        # ==========================

        if self.dashboard is not None:


            self.dashboard.render(

                perception_output=(
                    perception_output
                ),

                planning_output=(
                    planning_output
                ),

                control_output=(
                    control_command
                ),

                camera_frame=frame,

                show=show,

                save_path=save_path,

            )



        # ==========================
        # M6 OUTPUT
        # ==========================

        return (

            perception_output,

            planning_output,

            control_command,

        )