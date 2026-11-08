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