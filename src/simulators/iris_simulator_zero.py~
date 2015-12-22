import iris_simulator as ism
import numpy as np

class IrisSimulatorZero(ism.IrisSimulator):
    """Dumb simulator that leaves the quad where it is."""


    @classmethod
    def _dynamics(self, pos, rot, vel, cmd):
        return np.zeros(3), np.zeros((3,3)), np.zeros(3)


    def __init__(self, time, pos, rot, vel):
        ism.IrisSimulator.__init__(self, time, pos, rot, vel)
        
        
"""Testing."""
sim = IrisSimulatorZero(0.0, np.zeros(3), np.eye(3), np.zeros(3))
print sim.get_time_state()
sim.update(np.zeros(4), 1.0)
print sim.get_time_state()
