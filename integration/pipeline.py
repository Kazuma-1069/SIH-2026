"""
SIH-2026 Integration Pipeline

Full autonomous driving data flow:

CARLA/M4
   ↓
M2 Perception
   ↓
M1 Planning
   ↓
M5 Vehicle Control
   ↓
CARLA Vehicle
   ↓
M3 Visualization
"""

from integration.data_adapter import perception_to_planning_input


class IntegrationPipeline:

    def __init__(
        self,
        perception_pipeline,
        planner,
        controller=None,
        dashboard=None,
        vehicle=None,
    ):

        self.perception_pipeline = perception_pipeline
        self.planner = planner

        # M5
        self.controller = controller

        # M3
        self.dashboard = dashboard

        # M4 vehicle interface
        self.vehicle = vehicle


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
            self.perception_pipeline.process_frame(
                frame
            )
        )


        # ==========================
        # M2 -> M1 ADAPTER
        # ==========================

        planning_input = (
            perception_to_planning_input(
                perception_output
            )
        )


        # ==========================
        # M1 PLANNING
        # ==========================

        planning_output = (
            self.planner.plan(
                planning_input
            )
        )


        # ==========================
        # M5 VEHICLE CONTROL
        # ==========================

        control_command = None


        if self.controller is not None:

            control_command = (
                self.controller.compute_control(
                    planning_output
                )
            )


        # ==========================
        # M5 -> M4 CARLA VEHICLE
        # ==========================

        if (
            self.vehicle is not None
            and control_command is not None
        ):

            self.vehicle.apply_control(
                control_command
            )


        # ==========================
        # M3 VISUALIZATION
        # ==========================

        if self.dashboard is not None:

            self.dashboard.render(
                perception_output=perception_output,
                planning_output=planning_output,
                control_output=control_command,
                camera_frame=frame,
                show=show,
                save_path=save_path,
            )


        # ==========================
        # OUTPUT FOR M6 TESTING
        # ==========================

        return (
            perception_output,
            planning_output,
            control_command,
        )