from iris_controller import IrisController


class IrisControllerZero(IrisController):
    """Dummy Controller.
    It always returns zero.
    """

    def __init__(self):
        IrisController.__init__(self)

    def control_law(self, pos, rot, vel, rp, rv, ra, rj, rs, rc):
        cmd = [0.0, 0.0, 0.0, 0.0]
        return cmd
