import simulators.iris_simulator as is_
import simulators.iris_simulator_parameters as isp

import scipy.integrate as spi
import utils.utility_functions as uf

import numpy as np



class IrisSimulatorStabilizeMode(is_.IrisSimulator):
    """Simulator corresponding to the stabilize mode of the Iris.
    This simulator simply implements double-integrator dynamics and does not
    reproduce the internal dynamics of the stabilize mode.
    This means that the quad will always have zero roll and pitch, and will
    simply get the thrust that is specified as a control input.
    The yaw will be controlled as a single integrator, since the control input
    includes the yaw rate.
    The control input corresponds to the four values on the RC.
    These values are, in this order, roll, pitch, throttle and yaw rate.
    Each of them is expressed as a value between 1000 and 2000.
    The throttle value must be converted into a force, the roll and pitch values
    must be converted into angles, and the yaw rate value must be converted into
    an angular speed.
    """


    def __init__(self, time, pos, rot, vel):

        is_.IrisSimulator.__init__(self, time, pos, rot, vel)


    @classmethod
    def _cmd_to_thrust_omega(self, cmd, rot):
        """Computes the thrust as a 3D numpy array from the RC command."""
        
        # unpacking the control input
        roll_cmd, pitch_cmd, throttle_cmd, yaw_rate_cmd = cmd
        
        # getting the current yaw from rot
        current_yaw = uf.rot_to_ea(rot)[2]

        # maximum yaw rate
        max_yaw_rate_deg = isp.max_yaw_rate_deg
        max_yaw_rate = max_yaw_rate_deg * np.pi / 180.0

        # angular speed
        omega = np.zeros(3)
        omega[2] = -(float(yaw_rate_cmd) - 1500.0) * max_yaw_rate / 500.0

        # maximum tilt
        max_tilt_deg = isp.max_tilt_deg
        max_tilt = max_tilt_deg * np.pi / 180.0

        # desired roll
        roll_des = (float(roll_cmd) - 1500.0) * max_tilt / 500.0

        # desired pitch
        pitch_des = -(float(pitch_cmd) - 1500.0) * max_tilt / 500.0

        # desired acceleration
        ea = roll_des, pitch_des, current_yaw
        rot = uf.ea_to_rot(ea)
        throttle_gain = isp.quad_mass * isp.gravity / isp.neutral_throttle
        thrust = throttle_gain * throttle_cmd * rot.dot(uf.e3)
        
        return thrust, omega
        

    @classmethod
    def _dynamics(self, pos, rot, vel, cmd):
        """Reimplements the abstract method of the IrisSimulator class."""

        thrust, omega = self._cmd_to_thrust_omega(cmd, rot)
        acc = thrust / isp.quad_mass - isp.gravity * uf.e3
        
        # derivatives
        dpos = vel
        dvel = acc
        drot = uf.skew(omega).dot(rot)

        return dpos, drot, dvel
        
        

"""Testing."""
#sim = IrisSimulatorStabilizeMode(0.0, np.zeros(3), np.eye(3), np.zeros(3))
#sim.print_time_state()
#sim.update(isp.neutral_cmd, 1.0)
#cmd = [1500.0, 1500.0, 1800.0, 2000.0]
#sim.update(cmd, 2.0)
#sim.print_time_state()
