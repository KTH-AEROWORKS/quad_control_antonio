import scipy.integrate as spi
import numpy as np
import abc
import utils.utility_functions as uf



class IrisSimulator():
    """Skeleton class for the quad simulator.
    You can think about it as an abstract class.
    The real simulators are obtained by inheritance.
    Each inherited simulator must implement the '_dynamics' function, which
    defines the dynamics of the simulator.
    """
        
        
    @classmethod
    def _vectorize_quad_state(self, p, rot, v):
        rot_vec = np.reshape(rot, 9)
        return np.concatenate([p, v, rot_vec])


    @classmethod
    def _unvectorize_quad_state(self, x):
        p = x[0:3]
        v = x[3:6]
        rot = np.reshape(x[6:], (3, 3))
        return p, rot, v


    @classmethod
    def _dynamics(self, p, rot, v, cmd):
        """Each simulator needs to reimplement this function."""
        raise NotImplementedError
        dp = None
        drot = None
        dv = None
        return dp, drot, dv
    

    @classmethod
    def _ode_function(self, t, y, cmd):
        p, rot, v = self._unvectorize_quad_state(y)
        dp, drot, dv = self._dynamics(p, rot, v, cmd)
        dy = self._vectorize_quad_state(dp, drot, dv)
        return dy
        

    def __init__(self, time, pos, rot, vel):
        """Constructor of the simulator.
        The initialization parameters are:
        - initial time t
        - initial position p
        - initial attitude in the form of a rotation matrix
        - initial velocity
        The state of the quad is represented by pos, rot, vel.
        This is a sufficient description of the state if we control the Iris
        quads in either Stabilize or Acro mode. Moreover, position and velocity
        can be reliably obtained by our mocap, while further derivatives tend to
        be unreliable.
        This function sets the interface between the quad and the controller.
        If you want to change this interface, you will need to change all the
        simulators and the controllers accordingly. 
        """
        
        self._time = time
        
        self._pos = pos
        self._rot = rot
        self._vel = vel
        
        self._solver = spi.ode(self._ode_function).set_integrator('dopri5')


    def get_time_state(self):
        return self._time, self._pos, self._rot, self._vel
        
        
    def print_time_state(self):
        print 'time: ' + str(self._time)
        print 'pos: ' + str(tuple(self._pos))
        print 'vel: ' + str(tuple(self._vel))
        print 'roll, pitch, yaw: ' + str(tuple(uf.rot_to_ea_deg(self._rot)))

    def update(self, control_input, final_time):
        
        y = self._vectorize_quad_state(self._pos, self._rot, self._vel)
        self._solver.set_initial_value(y, self._time)
        self._solver.set_f_params(control_input)
        new_y = self._solver.integrate(final_time)

        self._time = final_time
        self._pos, new_rot, self._vel = self._unvectorize_quad_state(new_y)
        
        #self._rot = uf.ea_to_rot(uf.rot_to_ea(new_rot))
        self._rot = new_rot
        
        return self.get_time_state()
        
        

"""Testing."""
#sim = IrisSimulator(0.0, np.zeros(3), np.eye(3), np.zeros(3))
#print(sim.get_time_state())
#sim.update(np.zeros(4), 1.0)
