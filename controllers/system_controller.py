from controllers.conveyor_controller import ConveyorController
from controllers.robot_controller import RobotController

class SystemController:
    """
    Coordinates the robotic system.

    Responsibilities:
        - Initialize and manage device controllers.
        - Start and update the system.
        - Handle controller errors and recovery.
        - Clean up system resources.
    """

    def __init__(self, sim, sim_ik):
        self.robot = RobotController(sim, sim_ik)
        self.conveyor = ConveyorController(sim)

    def start(self):
        self.robot.start()
        self.conveyor.start()

    def update(self):
        self.robot.update()
        self.conveyor.update()

    def reset(self):
        self.robot.reset()
        self.robot.start()

    def cleanup(self, sim, sim_ik):
        self.robot.cleanup(sim, sim_ik)