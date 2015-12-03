"""Implementation of the simulator of the stabilize mode."""


# skeleton class for the simulators
import simulators.iris_simulator as iss

# for solving differential equations
import scipy.integrate as spi

# import the simulation parameters
import simulators.simulator_parameters as sp

# utility functions
import utils.utility_functions as uf


import numpy


# self-explanatory
def vectorize_quad_state(p, rot, v):
    rot_vec = numpy.reshape(rot, 9)
    return numpy.concatenate([p, v, rot_vec])


# self-explanatory
def unvectorize_quad_state(x):
    p = x[0:3]
    v = x[3:6]
    rot = numpy.reshape(x[6:], (3, 3))
    return p, rot, v


class IrisSimulatorStabilizeMode(iss.IrisSimulator):
    """Simulator corresponding to the stabilize mode of the Iris."""

    def __init__(self, time, pos, rot, vel):

        iss.IrisSimulator.__init__(self, time, pos, rot, vel)

        self.solver = spi.ode(self.ode_f).set_integrator('dopri5')
        y0 = vectorize_quad_state(self.pos, self.rot, self.vel)
        self.solver.set_initial_value(y0, self.time)

    def dynamics(self, pos, rot, vel, cmd):

        # unpacking the control input
        roll_cmd, pitch_cmd, thrust_cmd, yaw_rate_cmd = cmd

        ea = uf.rot_to_ea(rot)

        # current yaw
        yaw = ea[2]

        # maximum yaw rate
        max_yaw_rate_deg = sp.max_yaw_rate_deg
        max_yaw_rate = max_yaw_rate_deg * numpy.pi / 180

        # maximum tilt
        max_tilt_deg = sp.max_tilt_deg
        max_tilt = max_tilt_deg * numpy.pi / 180

        # desired roll
        roll_des = (roll_cmd - 1500) * max_tilt / 500

        # desired pitch
        pitch_des = -(pitch_cmd - 1500) * max_tilt / 500

        # gain of the inner loop for attitude control
        ktt = sp.inner_loop_gain

        # desired thrust versor
        e3 = numpy.array([0.0, 0.0, 1.0])
        dtv = uf.ea_to_rot((roll_des, pitch_des, yaw)).dot(e3)

        # actual thrust versor
        atv = rot.dot(e3)

        # angular velocity
        aux = ktt * uf.skew(atv).dot(dtv)
        rot_t = numpy.transpose(rot)
        omega = rot_t.dot(aux)

        # yaw rate
        omega[2] = -(yaw_rate_cmd - 1500) * max_yaw_rate / 500

        # neutral thrust
        nt = sp.neutral_thrust

        # thrust gain
        kt = sp.quad_mass * sp.g / nt

        # thrust command adjustment
        # The thrust sent to the motors is automatically adjusted according to
        # the tilt angle of the vehicle (bigger tilt leads to bigger thrust)
        # to reduce the thrust compensation that the pilot must give.
        thrust_cmd = thrust_cmd / numpy.dot(atv, e3)

        # dynamics
        dpos = vel
        dvel = kt * thrust_cmd * atv / sp.quad_mass - sp.g * e3
        drot = rot.dot(uf.skew(omega))

        return dpos, drot, dvel

    def ode_f(self, t, y, u):

        pos, rot, vel = unvectorize_quad_state(y)
        dpos, drot, dvel = self.dynamics(pos, rot, vel, u)
        dy = vectorize_quad_state(dpos, drot, dvel)
        return dy

    def update_state(self, cmd, tf):

        self.solver.set_f_params(cmd)
        new_y = self.solver.integrate(tf)
        self.solver.set_initial_value(self.solver.y, tf)

        self.time = tf
        self.pos, self.rot, self.vel = unvectorize_quad_state(new_y)

        return self.time, self.pos, self.rot, self.vel
