from controllers.conveyor_controller import ConveyorController
from controllers.robot_controller import RobotController
from opt.error import RobotError, SystemControlError
from opt.event_logger import EventCategory, EventLogger

class SystemController:
    # states
    WAITING = 0
    PROCESSING = 1
    STOPPED = 2
    ERROR = 3

    def __init__(self, sim, sim_ik):
        self.logger = EventLogger('SystemController')
        self.box_detected = False
        self.sim = sim

        self.box = sim.getObject('/Box1')
        self.table = sim.getObject('/highTable')

        self.robot = RobotController(sim, sim_ik, self.box, self.table)
        self.conveyor = ConveyorController(sim)

        self.state = self.STOPPED
        self.logger.set_state('STOPPED')
        self.deadline = None

        self.wait_timeout = 30
        self.processing_timeout = 20

    def start(self):
        self.logger.log(EventCategory.COMMAND, 'Start command received')
        if self.state != self.STOPPED:
            self.logger.log(
                EventCategory.COMMAND,
                'Start command ignored: system is not stopped'
            )
            return

        self.robot.start()
        self.conveyor.start()
        self._set_state(self.WAITING, self.wait_timeout, reason='System started')

    def update(self):
        if self.state in (self.ERROR, self.STOPPED):
            return

        t = self.sim.getSimulationTime()

        if self.state == self.WAITING:
            if self.is_box_on_table():
                self.box_detected = True

                if self.robot.start_task():
                    self.conveyor.stop()
                    self._set_state(
                        self.PROCESSING,
                        self.processing_timeout,
                        reason='Box processing task started'
                    )

            if self.state == self.WAITING and self._timer_expired(t):
                self._handle_error('Box waiting timeout')

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
                self.box_detected = False

                self._set_state(
                    self.WAITING,
                    self.wait_timeout,
                    reason='Box processing completed'
                )
            elif self._timer_expired(t):
                self._handle_error('Box processing timeout')

        try:
            self.robot.update()
        except RobotError as e:
            self._error_for_robot_error(str(e))
            raise

        self.conveyor.update()

    def reset(self):
        self.robot.reset(force=self.state in (self.ERROR, self.STOPPED))
        self.robot.start()

        self.box_detected = False
        self.conveyor.start()

        self._set_state(self.WAITING, self.wait_timeout, reason='System reset')

    def _set_state(self, state, timeout=None, reason=''):
        """Change state, set its timer and log the transition."""

        previous_state = self.state
        self.state = state
        self.deadline = (
            self.sim.getSimulationTime() + timeout
            if timeout is not None else None
        )

        if previous_state != state:
            states = {
                self.WAITING: 'WAITING',
                self.PROCESSING: 'PROCESSING',
                self.STOPPED: 'STOPPED',
                self.ERROR: 'ERROR'
            }
            self.logger.set_state(states[state])
            self.logger.log(
                EventCategory.STATE_CHANGE,
                'System state changed',
                previous_state=states[previous_state],
                next_state=states[state],
                reason=reason or 'Not specified'
            )

    def _timer_expired(self, t):
        """Check whether current state timer expired."""

        return self.deadline is not None and t >= self.deadline

    def _handle_error(self, message):
        """Stop the system and raise its own error."""

        self.logger.log(EventCategory.ERROR, message)
        self._set_state(self.ERROR, reason=message)
        self.conveyor.stop()
        self.conveyor.update()

        raise SystemControlError(message)

    def _error_for_robot_error(self, message):
        """Stop the system while preserving the robot's own error."""

        self._set_state(self.ERROR, reason=message)
        self.conveyor.stop()
        self.conveyor.update()

    def cleanup(self, sim, sim_ik):
        self.robot.cleanup(sim, sim_ik)

    def is_box_on_table(self):
        """Check whether the box is touching the table."""

        result, _ = self.sim.checkCollision(
            self.box,
            self.table
        )

        return bool(result)
