import numpy
from numpy import cos as c
from numpy import sin as s

import planners.trajectory as tj



class TrajectoryCircle(tj.Trajectory):
    """Circle trajectory."""


    def __init__(self, offset, rotation, initial_time, final_time, radius, angular_velocity):
        """Arguments:
        - offset (numpy array, 4)
        - rotation (numpy array, 3-by-3)
        - radius (float)
        - angular_velocity (float)
        """
        
        tj.Trajectory.__init__(self, offset, rotation, initial_time, final_time)
        self.radius = radius
        self.angular_velocity = angular_velocity


    def _get_untransformed_point(self, time):

        t = time
        r = self.radius
        w = self.angular_velocity
        rot = self.rotation
        off = self.offset

        p = numpy.zeros(4)
        v = numpy.zeros(4)
        a = numpy.zeros(4)
        j = numpy.zeros(4)
        sn = numpy.zeros(4)
        cr = numpy.zeros(4)

        p[0:3] = r*(numpy.array([c(w * t), -s(w * t), 0.0])-numpy.array([1.0, 0.0, 0.0]))
        v[0:3] = r * w**1 * numpy.array([-s(w * t), -c(w * t), 0.0])
        a[0:3] = r * w**2 * numpy.array([-c(w * t), s(w * t), 0.0])
        j[0:3] = r * w**3 * numpy.array([s(w * t), c(w * t), 0.0])
        sn[0:3] = r * w**4 * numpy.array([c(w * t), -s(w * t), 0.0])
        cr[0:3] = r * w**5 * numpy.array([-s(w * t), -c(w * t), 0.0])

        p[3] = -numpy.arctan2(s(w*t),c(w*t))
        v[3] = -w
        a[3] = 0.0
        j[3] = 0.0
        sn[3] = 0.0
        cr[3] = 0.0

        return p, v, a, j, sn, cr
