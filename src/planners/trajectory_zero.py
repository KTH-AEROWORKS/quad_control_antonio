import trajectory as tj

class TrajectoryZero(tj.Trajectory):
    """A still trajectory (stays forever in the initial point)."""


    def __init__(self, offset, rotation, initial_time, final_time):
        tj.Trajectory.__init__(self, offset, rotation, initial_time, final_time)


    def _get_untransformed_point(self, time):
        """Always return all zeros.
        In this way, the actual trajectory will forever stay in the point given
        by the offset.
        """
    
        p = numpy.array(self.offset)
        v = numpy.zeros(4)
        a = numpy.zeros(4)
        j = numpy.zeros(4)
        s = numpy.zeros(4)
        c = numpy.zeros(4)
        
        return p, v, a, j, s, c
