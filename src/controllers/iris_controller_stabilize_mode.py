import controllers.iris_controller as ic
import controllers.iris_controller_parameters as icp

import utils.utility_functions as uf
import utils.filters as fs

import numpy as np



class IrisControllerStabilizeMode(ic.IrisController):
    """Controller for the Iris quad in the stabilize mode."""
    
    
    def __init__(self):
        ic.IrisController.__init__(self)

    
    def control_law(self, pos, rot, vel, rp, rv, ra, rj, rs, rc):
        """Returns the control input for the Iris quad in the stabilize mode."""
        
        err_pos = rp[0:3] - pos
        err_vel = rv[0:3] - vel
        
        current_yaw = uf.rot_to_ea(rot)[2]
        err_yaw = rp[3] - current_yaw
        err_yaw = np.arctan2(np.sin(err_yaw), np.cos(err_yaw))
        
        des_acc = icp.gravity * uf.e3 + icp.kp*(err_pos) + icp.kv*(err_vel)
        des_thrust = des_acc * icp.quad_mass
        des_throttle = np.linalg.norm(des_thrust)
        
        des_yaw_rate = icp.k_yaw*(err_yaw)
        
        if des_throttle < icp.min_throttle:
            des_roll = 0.0
            des_pitch = 0.0
        else:
            n_des = des_thrust/des_throttle
            n_des_rot = uf.rot_z(-current_yaw).dot(n_des)
            sin_roll = -n_des_rot[1]
            des_roll = np.arcsin(sin_roll)
            sin_pitch = n_des_rot[0]/np.cos(des_roll)
            cos_pitch = n_des_rot[2]/np.cos(des_roll)
            des_pitch = np.arctan2(sin_pitch,cos_pitch)
        
        cmd_throttle = des_throttle/icp.quad_mass/icp.gravity*icp.neutral_throttle
        cmd_throttle = int(self._saturation(cmd_throttle, 1000.0, 2000.0))
        
        max_tilt = icp.max_tilt_deg * np.pi/180.0
        
        cmd_roll = 1500.0 + des_roll*500.0/max_tilt
        cmd_roll = int(self._saturation(cmd_roll, 1000.0, 2000.0))
        
        cmd_pitch = int(1500.0 - des_pitch*500.0/max_tilt)
        cmd_pitch = int(self._saturation(cmd_pitch, 1000.0, 2000.0))
        
        max_yaw_rate = icp.max_yaw_rate_deg * np.pi / 180.0
        cmd_yaw = int(1500.0 - des_yaw_rate*500.0/max_yaw_rate)
        cmd_yaw = int(self._saturation(cmd_yaw, 1000.0, 2000.0))
        
        cmd = (cmd_roll, cmd_pitch, cmd_throttle, cmd_yaw)
        
        return cmd
        
    
    @classmethod
    def _saturation(self, x, min_, max_):
        if x < min_:
            return min_
        elif x > max_:
            return max_
        return x
       
        
        
"""Testing."""
#con = IrisControllerStabilizeMode()
