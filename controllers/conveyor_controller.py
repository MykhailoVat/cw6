
class ConveyorController:
    def __init__(
        self,
        sim,
        velocity: float = 0.1
    ):
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

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def update(self):
        velocity = self.velocity if self.running else 0.0
        data = self.sim.packTable({"vel": velocity})

        for handle in self.handles:
            self.sim.setBufferProperty(
                handle,
                "customData.__ctrl__",
                data
            )