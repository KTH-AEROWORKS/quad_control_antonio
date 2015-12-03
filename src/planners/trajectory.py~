"""This file defines the abstract trajectory class.
This class gives the interface of a general trajectory class,
and different trajectories can be defined as subclasses.
A 'Trajectory' class needs to have a '_get_untransformed_point' method that
takes the current time and returns the current value of the trajectory along
with the first five derivatives.
All trajectories are in 4D, that is x, y, z, and yaw.
Also note that Trajectory objects have no memory. The returned position and the
derivatives are computed analitically.
"""

import numpy


class Trajectory:
    """Abstract class for a generic trajectory"""


    def __init__(self, offset, rotation, initial_time, final_time):
        self.offset = numpy.array(offset)
        self.rotation = numpy.array(rotation)
        self.t0 = initial_time
        self.tf = final_time

    def _get_untransformed_point(self, time):
        """The actual trajectory is given here.
        This is an abstract function and each Trajectory object needs to
        reimplement it.
        """
    
        position = None
        velocity = None
        acceleration = None
        jerk = None
        snap = None
        crackle = None
        
        return position, velocity, acceleration, jerk, snap, crackle
        
        
    def _add_offset_and_rotation(self, p, v, a, j, sn, cr):
    
        off = self.offset
        rot = self.rotation
        
        new_p = numpy.zeros(4)
        new_v = numpy.zeros(4)
        new_a = numpy.zeros(4)
        new_j = numpy.zeros(4)
        new_sn = numpy.zeros(4)
        new_cr = numpy.zeros(4)
        
        new_p[0:3] = rot.dot(p[0:3]) + off[0:3]
        new_v[0:3] = rot.dot(v[0:3])
        new_a[0:3] = rot.dot(a[0:3])
        new_j[0:3] = rot.dot(j[0:3])
        new_sn[0:3] = rot.dot(sn[0:3])
        new_cr[0:3] = rot.dot(cr[0:3])

        new_p[3] = p[3] + off[3]
        
        return new_p, new_v, new_a, new_j, new_sn, new_cr
        
        
    def get_point(self, time):
        """This method is the only public method that a trajectory class should
        have.
        It returns the actual point in the trajectory after adding offset and
        rotation.
        This is the method that the planner nodes call.
        """
        
        if time < self.t0:
            p, v, a, j, sn, cr = self._get_untransformed_point(0.0)
        elif time > self.tf:
            p, v, a, j, sn, cr = self._get_untransformed_point(self.tf-self.t0)
        else:
            p, v, a, j, sn, cr = self._get_untransformed_point(time-self.t0)
        
        return self._add_offset_and_rotation(p, v, a, j, sn, cr)
        
