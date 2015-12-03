import controllers.iris_controller as ic
import controllers.iris_controller_parameters as icp

import utils.utility_functions as uf
import utils.filters as fs

import numpy as np



class IrisPDStabilizeMode(ic.IrisController):
    """Controller for the Iris quad in the stabilize mode."""
    
    def __init__(self):
        ic.IrisController.__init__(self)


    
    def control_law(self, pos, rot, vel, rp, rv, ra, rj, rs, rc):
        """Returns the control input for the Iris quad in the stabilize mode.
        """
        
        """
        Something is wrong here.
        We will fix it.
        
        err_pos = rp[0:3] - pos
        err_vel = rv[0:3] - vel
        
        u = icp.g*icp.e3 + icp.kp*(rp[0:3]-pos) + icp.kv*(rv[0:3]-vel)
        ea = uf.rot_to_ea(rot)
        yaw = ea[2]
        
        yaw_rate = icp.k_yaw*(rp[3]-yaw)
        
        cmd = [None, None, None, None]
        
        cmd[0] = np.linalg.norm(u)*icp.quad_mass*icp.g/icp.neutral_thrust
        
        roll = np.arctan2(u[1],u[0])
        cmd[1] = 1500.0 + roll*500.0/icp.MAX_TILT
        
        pitch = np.arcsin(u[2]/np.linalg.norm(u))
        cmd[2] = 1500.0 + pitch*500.0/icp.MAX_TILT
        
        cmd[3] = 1500.0 + yaw_rate*500.0/icp.MAX_YAW_RATE
        """
        
        cmd = [None, None, None, None]
        
        return cmd
