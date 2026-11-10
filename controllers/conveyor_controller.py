from opt.event_logger import EventCategory, EventLogger

class ConveyorController:
    def __init__(
        self,
        sim,
        velocity: float = 0.1
    ):
        self.logger = EventLogger('ConveyorController')
        self.sim = sim
        conveyor_paths = [
            "/conveyor1",
            "/conveyor2"
        ]
        self.handles = [
            sim.getObject(path) for path in conveyor_paths
        ]

        self.velocity = velocity
        self.running = True
        self.logger.set_state('RUNNING')

    def start(self):
        was_running = self.running
        self.running = True
        self.logger.set_state('RUNNING')
        if not was_running:
            self.logger.log(EventCategory.COMMAND, 'Conveyor started')
            self.logger.log(
                EventCategory.STATE_CHANGE,
                'Conveyor state changed',
                previous_state='STOPPED',
                next_state='RUNNING',
                reason='Start command'
            )

    def stop(self):
        was_running = self.running
        self.running = False
        self.logger.set_state('STOPPED')
        if was_running:
            self.logger.log(EventCategory.COMMAND, 'Conveyor stopped')
            self.logger.log(
                EventCategory.STATE_CHANGE,
                'Conveyor state changed',
                previous_state='RUNNING',
                next_state='STOPPED',
                reason='Stop command'
            )

    def update(self):
        velocity = self.velocity if self.running else 0.0
        data = self.sim.packTable({"vel": velocity})

        for handle in self.handles:
            self.sim.setBufferProperty(
                handle,
                "customData.__ctrl__",
                data
            )
