from controllers.conveyor_controller import ConveyorController
from controllers.robot_controller import RobotController

class SystemController:
    # states
    WAITING = 0
    PROCESSING = 1
    ERROR = 2

    def __init__(self, sim, sim_ik):
        self.box_detected = False
        self.sim = sim

        self.box = sim.getObject('/Box1')
        self.table = sim.getObject('/highTable')

        self.robot = RobotController(sim, sim_ik, self.box, self.table)
        self.conveyor = ConveyorController(sim)

        self.state = self.WAITING

    def start(self):
        self.robot.start()
        self.conveyor.start()

    def update(self):
        if self.state == self.WAITING:
            if self.is_box_on_table():
                self.box_detected = True

                if self.robot.start_task():
                    self.conveyor.stop()
                    self.state = self.PROCESSING

        elif self.state == self.PROCESSING:
            if (
                    self.box_detected
                    and self.robot.state in (
                    self.robot.LIFTING,
                    self.robot.GRABBING,
                    self.robot.TRANSFERRING,
                    self.robot.DROPPING,
            )
                    and not self.is_box_on_table()
            ):
                self.box_detected = False
                self.conveyor.start()

            if self.robot.state == self.robot.WAITING:
                self.state = self.WAITING

        self.robot.update()
        self.conveyor.update()

    def reset(self):
        self.robot.reset()
        self.robot.start()

        self.box_detected = False
        self.conveyor.start()

        self.state = self.WAITING

    def cleanup(self, sim, sim_ik):
        self.robot.cleanup(sim, sim_ik)

    def is_box_on_table(self):
        """Check whether the box is touching the table."""

        result, _ = self.sim.checkCollision(
            self.box,
            self.table
        )

        return bool(result)
