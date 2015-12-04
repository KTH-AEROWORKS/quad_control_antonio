import numpy


class PlannerToGoal:
    """A planner to drive the quad to a given goal point in 4D.
    It offers a function to set the reference acceleration of the quad according
    to its displacement with respect to the goal point.
    It also offers a function to set the velocity according to the
    displacement between the position and the desired position.
    In other words, it allows to control the quad both as a double or a single
    integrator.
    """


    def __init__(self, goal_point, gain_pos, gain_vel, max_acc, max_vel):
        """The constructor takes a goal point in 4D and saves it as a property.
        It also sets gains.
        """
        
        self._goal_point = goal_point
        self._gain_pos = gain_pos
        self._gain_vel = gain_vel
        self._max_acc = max_acc
        self._max_vel = max_vel
        
    
    def _saturate(self, acc, ths):
        """Saturates the acceleration if needed."""
        
        exp = 10.0
        acc_norm = numpy.linalg.norm(acc)
        acc_sat = acc/(1+(acc_norm/ths)**exp)**(1.0/exp)
        
        return acc_sat
        

    def get_acceleration(self, quad_pos, quad_vel):

        kp = self._gain_pos
        kv = self._gain_vel
        
        ps = self._goal_point
        p = quad_pos
        
        v = quad_vel
        v_des = self._saturate(kp*(ps-p), 0.1)

        acc = kp*(ps-p) + kv*(v_des-v)
        #acc = kp*(ps-p) + kv*(0.0-v)
        acc = self._saturate(acc, self._max_acc)

        return acc
        
        
    def get_velocity(self, quad_pos):
    
        kp = self._gain_pos
        
        ps = self._goal_point
        p = quad_pos
        
        vel = kp*(ps-p)
        vel = self._saturate(vel, self._max_vel)
        
        return vel
        
        
       
