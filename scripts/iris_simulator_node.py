#!/usr/bin/env python
"""This file implements the quad simulator.
There are different dynamics that you can choose from.
The dynamics can be changed by setting the object 'simulator' to one of the
simulators at disposal.
Of course, more dynamics functions can be defined and added to the simulator.
The type of simulator can be changed dynamically using a ROS service.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""


import numpy
import rospy

# for solving differential equations
from scipy.integrate import ode

# import the ROS msgs used by the simulator
# for QuadPosition and OverrideRCIn
import mavros_msgs.msg
import mocap.msg

# iporting the actual simulators and their parameters
import simulators.iris_simulator_zero as isz
import simulators.iris_simulator_stabilize_mode as issm
import simulators.iris_simulator_parameters as ssp

import utils.utility_functions as uf

"""Do not initialize variables here.
Do that in the 'work' function.
"""


class IrisSimulatorNode():


    def __init__(self):
    
        # inizializing the state of the quad
        self.pos = numpy.array([0.0, 0.0, 1.0])
        self.rot = numpy.eye(3)
        self.vel = numpy.zeros(3)

        # initializing the input
        self.rc_input = ssp.NEUTRAL_RC_INPUT


    def write_quad_state_msg(self, pos, rot, vel):
        """Converts the local state of the quad into a 'QuadPositionDerived'
        message from the mocap package.
        Warning. 'QuadPositionDerived' was maybe a poorly chosen name, since
        that type of message contains:
            position as x,y,z
            attitude as euler angles in degrees
            velocity
            angular velocity
            acceleration
            angular acceleration
        However, I decided to stick with the original mocap settings inherited
        by Matteo, so I did not change the name or format of that message.
        Some field of the message will be left blank, since our simulators only
        use position, attitude and velocity. It does not matter what will be put
        in those fileds, since we will not use them.
        """

        msg = mocap.msg.QuadPositionDerived()

        msg.found_body = True
        msg.x, msg.y, msg.z = pos
        msg.roll, msg.pitch, msg.yaw = uf.rot_to_ea_deg(rot)
        msg.x_vel, msg.y_vel, msg.z_vel = vel
        
        return msg


    def get_cmd_msg(self, msg):
        """This is the callback function that is called when a new value of the
        control input is received from the controller.
        This function copies the new value that has been received in the return.
        Only the first four fields of the message are used, which corresponds to
        position of the sticks of the IRIS remote.
        """
        
        self.rc_input = msg.channels[0:4]


    def work(self):
        """This function is the worker of the ROS node corresponding to this object.
        """

        # initialize a ROS node corresponding to the simulator
        rospy.init_node('iris_simulator_node')

        # initialize the time
        initial_time = rospy.get_time()
        self.time = 0.0

        # initialize the simulator
        self.simulator = issm.IrisSimulatorStabilizeMode(self.time, self.pos, self.rot, self.vel)
        #self.simulator = isz.IrisSimulatorZero(self.time, self.pos, self.rot, self.vel)

        # subscriber for the control input
        quad_cmd_sub = rospy.Subscriber(
            'iris_cmd', mavros_msgs.msg.OverrideRCIn, self.get_cmd_msg)

        # the node publishes the quad state to the topic 'iris_state'
        quad_state_pub = rospy.Publisher(
            'iris_state', mocap.msg.QuadPositionDerived, queue_size=10)

        # define the frequency of execution of this node
        freq = 1e2
        rate = rospy.Rate(freq)

        while not rospy.is_shutdown():

            # give the new control input to the ODE solver
            # self.solver.set_f_params(self.cmd)

            # get the current time
            new_time = rospy.get_time() - initial_time

            # publish the quad state
            quad_state_msg = self.write_quad_state_msg(self.pos, self.rot, self.vel)
            quad_state_pub.publish(quad_state_msg)

            # update the quad state
            #self.p, self.v, self.rot = unvectorize_quad_state(self.solver.y)
            self.time, self.pos, self.rot, self.vel = self.simulator.update_state(self.rc_input, new_time)

            # let the node sleep
            rate.sleep()

        rospy.spin()


# executing the node
if __name__ == '__main__':
    node = IrisSimulatorNode()
    node.work()
