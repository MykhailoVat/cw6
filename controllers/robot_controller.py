
import random

from opt.event_logger import EventCategory, EventLogger
from opt.error import RobotError


class RobotController:
    # States
    STOPPED = 0
    WAITING = 1
    SEARCHING = 2
    GRABBING = 3
    LIFTING = 4
    TRANSFERRING = 5
    DROPPING = 6
    ERROR = 7

    def __init__(self, sim, sim_ik):
        self.sim = sim
        self.sim_ik = sim_ik

        # Scene objects
        self.base = sim.getObject('/IRB140')
        self.target = sim.getObject('/IRB140/target')
        self.gripper = sim.getObject('/IRB140/suctionPad')

        self.box = sim.getObject('/Box1')
        self.table = sim.getObject('/highTable')

        # Suction cup
        self.suction = SuctionCup(sim)

        # IK environment
        self.ik_env = sim_ik.createEnvironment()

        # Primary IK group
        self.ik_primary = sim_ik.createGroup(self.ik_env)

        sim_ik.setGroupCalculation(
            self.ik_env,
            self.ik_primary,
            sim_ik.method_pseudo_inverse,
            0,
            10
        )

        sim_ik.addElementFromScene(
            self.ik_env,
            self.ik_primary,
            self.base,
            self.gripper,
            self.target,
            sim_ik.constraint_pose
        )

        # Secondary IK group
        self.ik_secondary = sim_ik.createGroup(self.ik_env)

        sim_ik.setGroupCalculation(
            self.ik_env,
            self.ik_secondary,
            sim_ik.method_damped_least_squares,
            0.3,
            99
        )

        sim_ik.addElementFromScene(
            self.ik_env,
            self.ik_secondary,
            self.base,
            self.gripper,
            self.target,
            sim_ik.constraint_pose
        )

        # State
        self.state = self.STOPPED
        self.deadline = None

        EventLogger.set_state(self._state_name(self.state))

        # Timeouts
        self.wait_timeout = 30
        self.search_timeout = 10
        self.grab_timeout = 10
        self.lift_timeout = 10
        self.transfer_timeout = 15

        # Parameters
        self.hover_height = 0.35
        self.grab_height = 0.025

        self.position_tolerance = 0.01
        self.velocity_tolerance = 0.01

        # Remembered box position
        self.point_box = None

        # Retrieve points
        self.retrieve_points = [
            [0.5, -0.3, self.hover_height],
            [0.7, -0.3, self.hover_height]
        ]

        EventLogger.log(
            EventCategory.SYSTEM,
            'Robot controller initialized'
        )

    def start(self):
        """Start robot operation."""

        EventLogger.log(
            EventCategory.COMMAND,
            'Start command received'
        )

        if self.state != self.STOPPED:
            EventLogger.log(
                EventCategory.COMMAND,
                'Start command ignored: robot is not stopped'
            )
            return

        if not self.is_robot_ready():
            EventLogger.log(
                EventCategory.ERROR,
                'Robot is not ready; start command rejected'
            )
            return

        self._set_state(
            self.WAITING,
            self.wait_timeout,
            reason='Start command'
        )

    def reset(self):
        """Reset robot after an error."""

        EventLogger.log(
            EventCategory.COMMAND,
            'Reset command received'
        )

        if self.state != self.ERROR:
            EventLogger.log(
                EventCategory.COMMAND,
                'Reset command ignored: robot is not in ERROR state'
            )
            return

        self.suction.off()
        self.suction.update()

        self.point_box = None

        self._set_state(
            self.STOPPED,
            reason='Reset command'
        )

    def update(self):
        """Main robot control loop."""

        # Update suction cup on every simulation step
        self.suction.update()

        t = self.sim.getSimulationTime()

        if self.state in (self.STOPPED, self.ERROR):
            return

        # WAITING
        if self.state == self.WAITING:

            if self.is_box_on_table():
                self.aim_box()

                self._set_state(
                    self.SEARCHING,
                    self.search_timeout,
                    reason='Box arrived and stopped on the table'
                )

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Box detected at destination, searching'
                )

            elif self._timer_expired(t):
                self._handle_error('Box did not arrive')

        # SEARCHING
        elif self.state == self.SEARCHING:

            if self.validate_pos() and self.is_box_on_table():
                self.grab_box()

                self._set_state(
                    self.GRABBING,
                    self.grab_timeout,
                    reason='Robot aimed at box, grabbing state activated'
                )

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Attempting to grab box'
                )

            elif self._timer_expired(t):
                self._handle_error('Box search timeout')

        # GRABBING
        elif self.state == self.GRABBING:

            if self.is_holding():
                self.lift_box()

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Box successfully grabbed'
                )

                self._set_state(
                    self.LIFTING,
                    self.lift_timeout,
                    reason='Box successfully grabbed'
                )

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Lifting box'
                )

            elif self._timer_expired(t):
                self._handle_error('Failed to grab box')

        # LIFTING
        elif self.state == self.LIFTING:

            if not self.is_holding():
                self._handle_error('Box lost during lifting')

            elif self.validate_pos():
                self.retrieve_box()

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Box lifted successfully'
                )

                self._set_state(
                    self.TRANSFERRING,
                    self.transfer_timeout,
                    reason='Box lifted successfully'
                )

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Transferring box to destination'
                )

            elif self._timer_expired(t):
                self._handle_error('Lifting timeout')

        # TRANSFERRING
        elif self.state == self.TRANSFERRING:

            if not self.is_holding():
                self._handle_error('Box lost during transfer')

            elif self.validate_pos():
                self.drop_box()

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Destination reached; releasing box'
                )

                self._set_state(
                    self.DROPPING,
                    reason='Robot reached the destination'
                )

            elif self._timer_expired(t):
                self._handle_error('Transfer timeout')

        # DROPPING
        elif self.state == self.DROPPING:

            if not self.is_holding():

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Box successfully dropped'
                )

                self._set_state(
                    self.WAITING,
                    self.wait_timeout,
                    reason='Box released successfully'
                )

                EventLogger.log(
                    EventCategory.NORMAL,
                    'Waiting for next box'
                )

    def _set_state(self, state, timeout=None, reason=''):
        """Change state, set its timer and log the transition."""

        previous_state = self.state
        self.state = state

        if timeout is not None:
            self.deadline = (
                self.sim.getSimulationTime() + timeout
            )
        else:
            self.deadline = None

        if previous_state != state:
            previous_name = self._state_name(previous_state)
            next_name = self._state_name(state)

            EventLogger.set_state(next_name)

            EventLogger.log(
                EventCategory.STATE_CHANGE,
                'Robot state changed',
                previous_state=previous_name,
                next_state=next_name,
                reason=reason or 'Not specified'
            )

    def _state_name(self, state):
        """Return readable state name."""

        states = {
            self.STOPPED: 'STOPPED',
            self.WAITING: 'WAITING',
            self.SEARCHING: 'SEARCHING',
            self.GRABBING: 'GRABBING',
            self.LIFTING: 'LIFTING',
            self.TRANSFERRING: 'TRANSFERRING',
            self.DROPPING: 'DROPPING',
            self.ERROR: 'ERROR'
        }

        return states.get(state, f'UNKNOWN({state})')

    def _timer_expired(self, t):
        """Check whether current state timer expired."""

        return (
            self.deadline is not None
            and t >= self.deadline
        )

    def _handle_error(self, message):
        """Stop robot operation and enter error state."""

        EventLogger.log(
            EventCategory.ERROR,
            message
        )

        self._set_state(
            self.ERROR,
            reason=message
        )

        raise RobotError(message)

    def is_robot_ready(self):
        """Check whether robot scene objects are available."""

        handles = [
            self.base,
            self.target,
            self.gripper,
            self.box,
            self.table
        ]

        return all(handle != -1 for handle in handles)

    def is_holding(self):
        """Check whether suction cup holds the box."""

        return (
            self.sim.getObjectParent(
                self.suction.loop_dummy_1
            ) == self.box
        )

    def is_box_on_table(self):
        """Check box contact with floor and its velocity."""

        result, _ = self.sim.checkCollision(
            self.box,
            self.table
        )

        if not result:
            return False

        velocity, _ = self.sim.getObjectVelocity(
            self.box
        )

        speed = sum(
            v ** 2 for v in velocity
        ) ** 0.5

        return speed < self.velocity_tolerance


    def _handle_ik(self):
        """Execute primary IK, fallback to secondary IK."""

        result = self.sim_ik.handleGroup(
            self.ik_env,
            self.ik_primary,
            {'syncWorlds': True}
        )

        if result != self.sim_ik.result_success:
            self.sim_ik.handleGroup(
                self.ik_env,
                self.ik_secondary,
                {
                    'syncWorlds': True,
                    'allowError': True
                }
            )

    def aim_box(self):
        """Move gripper above the box."""

        self.point_box = self.sim.getObjectPosition(
            self.box, -1
        )

        self.sim.setObjectPosition(
            self.target,
            [
                self.point_box[0],
                self.point_box[1],
                self.point_box[2] + self.hover_height
            ]
        )

        self._handle_ik()

    def grab_box(self):
        """Lower gripper and enable suction cup."""

        self.sim.setObjectPosition(
            self.target,
            [
                self.point_box[0],
                self.point_box[1],
                self.point_box[2] + self.grab_height
            ]
        )

        self._handle_ik()

        self.suction.on()

    def lift_box(self):
        """Lift the grabbed box."""

        self.sim.setObjectPosition(
            self.target,
            [
                self.point_box[0],
                self.point_box[1],
                self.point_box[2] + self.hover_height
            ]
        )

        self._handle_ik()

    def retrieve_box(self):
        """Move box to a random destination."""

        point = random.choice(self.retrieve_points)

        self.sim.setObjectPosition(
            self.target, point
        )

        self._handle_ik()

    def drop_box(self):
        """Disable suction cup."""

        self.suction.off()

    def validate_pos(self):
        """Check whether gripper reached target position."""

        actual_pos = self.sim.getObjectPosition(
            self.gripper, -1
        )

        target_pos = self.sim.getObjectPosition(
            self.target, -1
        )

        return all(
            abs(actual_pos[i] - target_pos[i])
            < self.position_tolerance
            for i in range(3)
        )

    def cleanup(self, sim, sim_ik):
        """Release resources."""

        EventLogger.log(
            EventCategory.SYSTEM,
            'Robot controller cleanup started'
        )

        self.suction.cleanup(sim)
        sim_ik.eraseEnvironment(self.ik_env)

        EventLogger.log(
            EventCategory.SYSTEM,
            'Robot controller cleanup completed'
        )

class SuctionCup:
    def __init__(self, sim):
        self.sim = sim

        # Отримуємо об'єкти сцени
        self.sensor = sim.getObject('/IRB140/suctionPad/Sensor')
        self.loop_dummy_1 = sim.getObject('/IRB140/suctionPad/LoopClosureDummy1')
        self.loop_dummy_2 = sim.getObject('/IRB140/suctionPad/LoopClosureDummy2')
        self.body = sim.getObject('/IRB140/suctionPad')
        self.link = sim.getObject('/IRB140/suctionPad/Link')

        # Параметри присоски
        self.infinite_strength = True
        self.max_pull_force = 3
        self.max_shear_force = 1
        self.max_peel_torque = 0.1

        self.enabled = True

        self._reset()

    def on(self):
        self.enabled = True

    def off(self):
        self.enabled = False

    def update(self):
        if not self.enabled:
            if self.sim.getObjectParent(self.loop_dummy_1) != self.body:
                self._reset()
        else:
            if self.sim.getObjectParent(self.loop_dummy_1) == self.body:
                self._detect_object()
            elif not self.infinite_strength:
                self._check_strength()

    def _reset(self):
        self.sim.setLinkDummy(self.loop_dummy_1, -1)
        self.sim.setObjectParent(self.loop_dummy_1, self.body, True)

        matrix = self.sim.getObjectMatrix(self.loop_dummy_2)
        self.sim.setObjectMatrix(self.loop_dummy_1, matrix)

    def _detect_object(self):
        index = 0

        while True:
            shape = self.sim.getObjects(index, self.sim.sceneobject_shape)

            if shape == -1:
                break

            if (
                shape != self.body
                and self.sim.getBoolProperty(shape, 'respondable')
            ):
                result, *_ = self.sim.checkProximitySensor(
                    self.sensor, shape
                )

                if result == 1:
                    self.sim.setObjectParent(
                        self.loop_dummy_1, self.body, True
                    )

                    matrix = self.sim.getObjectMatrix(self.loop_dummy_2)
                    self.sim.setObjectMatrix(self.loop_dummy_1, matrix)

                    self.sim.setObjectParent(
                        self.loop_dummy_1, shape, True
                    )

                    self.sim.setLinkDummy(
                        self.loop_dummy_1, self.loop_dummy_2
                    )
                    break

            index += 1

    def _check_strength(self):
        result, force, torque = self.sim.readForceSensor(self.link)

        if result <= 0:
            return

        shear_force = (force[0] ** 2 + force[1] ** 2) ** 0.5

        break_it = (
            force[2] > self.max_pull_force
            or shear_force > self.max_shear_force
            or torque[0] > self.max_peel_torque
            or torque[1] > self.max_peel_torque
        )

        if break_it:
            self._reset()

    def cleanup(self, sim):
        self.sim = sim
        self._reset()
