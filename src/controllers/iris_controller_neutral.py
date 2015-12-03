import iris_controller as ic
import iris_controller_parameters as icp

class IrisControllerNeutral(ic.IrisController):
    """Dummy Controller.
    It always returns the neutral thrust.
    """

    def __init__(self):
        ic.IrisController.__init__(self)

    def control_law(self, pos, rot, vel, rp, rv, ra, rj, rs, rc):
        cmd = icp.NEUTRAL_CONTROL_INPUT
        return cmd
