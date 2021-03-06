import numpy
import random

class PlannerCollisionAvoidanceMultiple:
    """A planner to avoid collisions with multiple obstacles and with the
    ground.
    It offers a function to compute a 3D acceleration to drive the body away
    from the obstacles and from the ground.
    For convenience, the acceleration is returned as a 4D numpy array, but the
    yaw component is always set to zero, since we assume that the yaw is
    unrelated to collisions.
    """


    def __init__(self, gain, ths, max_acc, max_vel):
        """The constructor takes a gain and a threshold.
        The threshold is the distance where the collision avoidance starts to
        take effect.
        """
        
        self._gain = gain
        self._ths = ths
        self._max_acc = max_acc
        self._max_vel = max_vel
        self._shake = random.random()


    def _rotate_2d_array_by_pi_halves(self, array):
        aux = numpy.zeros(2)
        aux[0] = -array[1]
        aux[1] = array[0]
        return aux
        
    
    def _saturate(self, acc, ths):
        """Saturates the acceleration if needed."""
        
        exp = 10.0
        acc_norm = numpy.linalg.norm(acc)
        acc_sat = acc/(1+(acc_norm/ths)**exp)**(1.0/exp)
        
        return acc_sat
    
    
    def _get_vel_from_obstacle(self, quad_pos, obs_pos, time):

        k = self._gain
        s = self._ths
        
        cs = numpy.cos(time)
        sn = numpy.sin(self._shake*time)
        
        po = obs_pos[0:3]
        p = quad_pos[0:3]
        d = numpy.linalg.norm(po-p)
        
        if d < s:
            vel3 = -k*(po-p)/d*(1.0/d**2-1.0/s**2) 
            aux = self._rotate_2d_array_by_pi_halves(vel3[0:2])
            vel3[0:2] += (1+sn)*k*aux
            #acc[2] = 0.0
            vel4 = numpy.concatenate([vel3, numpy.array([0.0])])
            print vel4
        else:
            vel4 = numpy.zeros(4)
            
        return vel4     


    def _get_drive_from_obstacle(self, quad_pos, quad_vel, obs_pos):
        
        k = self._gain
        s = self._ths
        
        po = obs_pos[0:3]
        p = quad_pos[0:3]
        d = numpy.linalg.norm(po-p)
        
        if d < s and quad_vel.dot(po-p)>0.0:
            acc = -k*(po-p)/d*(1.0/d-1.0/s) 
            aux = self._rotate_2d_array_by_pi_halves(acc[0:2])
            acc[0:2] += 0.3*k*aux
            #acc[2] = 0.0
            acc4 = numpy.concatenate([acc, numpy.array([0.0])])
        else:
            acc4 = numpy.zeros(4)
            
        return acc4


    def _get_drive_from_ground(self, quad_pos, quad_vel):
    
        k = self._gain
        s = 0.5
        
        h = quad_pos[2]
        vh = quad_vel[2]
        
        if h < s and vh < 0.0:
            ah = k*(1.0/h-1.0/s)
            acc = numpy.array([0.0, 0.0, ah, 0.0])
        else:
            acc = numpy.zeros(4)
            
        return acc

    
    def _get_vel_from_ground(self, quad_pos):
    
        k = self._gain
        s = 0.0
        
        h = quad_pos[2]
        
        if h < s:
            vh = k*(1.0/h-1.0/s)
            vel = numpy.array([0.0, 0.0, vh, 0.0])
        else:
            vel = numpy.zeros(4)
            
        return vel
        

    def get_acceleration(self, quad_pos, quad_vel, obss_poss):
        
        acc = numpy.zeros(4)
        
        for obs_pos in obss_poss:
            acc += self._get_drive_from_obstacle(quad_pos, quad_vel, obs_pos)
            
        acc += self._get_drive_from_ground(quad_pos, quad_vel)
        #acc = self._saturate(acc, 10.0)

        return acc
        
    
    def get_velocity(self, quad_pos, obss_poss, time):
        
        vel = numpy.zeros(4)
        
        for obs_pos in obss_poss:
            vel += self._get_vel_from_obstacle(quad_pos, obs_pos, time)
            
        vel += self._get_vel_from_ground(quad_pos)
        vel = self._saturate(vel, self._max_vel)

        return vel
