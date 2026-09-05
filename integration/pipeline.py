from integration.data_adapter import perception_to_planning_input


from integration.data_adapter import perception_to_planning_input
from simulation.controller import VehicleController


class IntegrationPipeline:

    def __init__(
        self,
        perception_pipeline,
        planner,
        dashboard=None,
    ):
        self.perception_pipeline = perception_pipeline
        self.planner = planner
        self.dashboard = dashboard
        self.controller = VehicleController()

    def process_frame(
        self,
        frame,
        show=False,
        save_path=None,
    ):
        # M2
        perception_output = (
            self.perception_pipeline.process_frame(frame)
        )

        # M2 -> M1
        planning_input = (
            perception_to_planning_input(
                perception_output
            )
        )

        # M1
        planning_output = self.planner.plan(
            planning_input
        )

        # M5
        control_command = (
            self.controller.compute_control(
                planning_output
            )
        )

        # M3
        if self.dashboard is not None:
            self.dashboard.render(
                perception_output=perception_output,
                planning_output=planning_output,
                camera_frame=frame,
                show=show,
                save_path=save_path,
            )

        return (
            perception_output,
            planning_output,
            control_command,
        )