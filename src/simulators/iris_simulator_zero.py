import iris_simulator as ism


class IrisSimulatorZero(ism.IrisSimulator):
    """Dumb simulator that leaves the quad where it is."""


    def __init__(self, time, pos, rot, vel):
        ism.IrisSimulator.__init__(self, time, pos, rot, vel)


    def update_state(self, control_input, final_time):
        self.time = final_time
        return self.time, self.pos, self.rot, self.vel
