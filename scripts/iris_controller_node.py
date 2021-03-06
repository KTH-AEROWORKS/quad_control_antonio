#!/usr/bin/env python
"""This file implements the quad controller node.
There are different control laws that you can choose from.
The control law can be changed by setting the object 'controller' to one of
the given controllers.
Of course, more controllers can be defined and added to the module.
The controllers can be changed dynamically using a ROS service.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""


import rospy

import mocap.msg as mcp_msgs
import mavros_msgs.msg as mvs_msgs
import quad_control_antonio.msg as qca_msgs
import utils.utility_functions as uf

import controllers.iris_controller_zero as icz
import controllers.iris_controller_neutral as icn
import controllers.iris_controller_stabilize_mode as icsm

import controllers.iris_controller_parameters as icp

import numpy as np



class IrisControllerNode:

    def __init__(self):
        
        # reference trajectory
        self._rp = np.zeros(4)
        self._rv = np.zeros(4)
        self._ra = np.zeros(4)
        self._rj = np.zeros(4)
        self._rs = np.zeros(4)
        self._rc = np.zeros(4)

        # state of the quad
        self._quad_pos = icp.init_pos
        self._quad_rot = np.eye(3)
        self._quad_vel = np.zeros(3)

        # initialize the controller
        #self._controller = ipdsm.IrisPDStabilizeMode()
        #self._controller = icz.IrisControllerZero()
        #self._controller = icn.IrisControllerNeutral()
        self._controller = icsm.IrisControllerStabilizeMode()


    def _get_quad_state(self, mocap_msg):
        """This function is the callback that is called when the controller
        receives a position measurement from the quad.
        The argument 'mocap_msg' is a ROS message of type
        'QuadPositionDerived', which (in spite of the misleading name) contains
        both position, attitude and velocity of the quad.
        It also contains further derivatives which we will not use.
        """

        self._quad_pos = np.array([mocap_msg.x, mocap_msg.y, mocap_msg.z])
        self._quad_rot = uf.ea_deg_to_rot(np.array([mocap_msg.roll, mocap_msg.pitch, mocap_msg.yaw]))
        self._quad_vel = np.array([mocap_msg.x_vel, mocap_msg.y_vel, mocap_msg.z_vel])


    def _get_reference_trajectory(self, rtm):
        """This function is the callback that is called when the controller
        receives a time instance of the reference trajectory and its
        derivatives.
        The argument 'rtm' (reference trajectory message) is a ROS message of
        type 'ReferenceTrajectoryMsg', which contains a timestamp and the
        corresponding values of the reference trajectory and its derivatives.
        We will not use the time stamp here.
        """

        self._rp = np.array(rtm.position)
        self._rv = np.array(rtm.velocity)
        self._ra = np.array(rtm.acceleration)
        self._rj = np.array(rtm.jerk)
        self._rs = np.array(rtm.snap)
        self._rc = np.array(rtm.crackle)


    def _write_quad_cmd_msg(self, cmd):

        message = mvs_msgs.OverrideRCIn()
        message.channels[0:4] = cmd
        message.channels[4:8] = [1500, 1500, 1500, 1500]
        
        return message


    def work(self):
        """This function is the worker of the ROS node corresponding to this
        object.
        """

        # initialize the ROS node
        rospy.init_node('iris_controller')

        # publisher of the control input signal
        cmd_pub = rospy.Publisher('iris_cmd', mvs_msgs.OverrideRCIn, queue_size=10)

        # subscriber to the quad pose
        pose_sub = rospy.Subscriber(
            'iris_state', mcp_msgs.QuadPositionDerived, self._get_quad_state)

        # subscriber to the reference trajectory
        ref_sub = rospy.Subscriber(
            'iris_reference_trajectory',
            qca_msgs.ReferenceTrajectoryMsg,
            self._get_reference_trajectory)

        # setting the frequency of the execution
        freq = 1e2
        rate = rospy.Rate(freq)

        # do work
        while not rospy.is_shutdown():

            control_input = self._controller.control_law(self._quad_pos, self._quad_rot, self._quad_vel, self._rp, self._rv, self._ra, self._rj, self._rs, self._rc)
            cmd_msg = self._write_quad_cmd_msg(control_input)
            cmd_pub.publish(cmd_msg)

            rate.sleep()

        rospy.spin()


# run the node
if __name__ == '__main__':
    node = IrisControllerNode()
    node.work()
