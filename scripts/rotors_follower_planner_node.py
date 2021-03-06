#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is planned by copying the trajectory of a leader quad and adding
a constant offset.
The trajectory is published in the form of a ROS message of the type
'MultiDOFJointTrajectory', and a description can be found at this link.

http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html 

Such ROS message can be taken as a reference by the quad models in the RotorS
simulator.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""


import rospy

import numpy

import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm
import nav_msgs.msg as nm

import planners.trajectory_circle as ct
import planners.trajectory_cubic as cbt
import planners.trajectory_quintic as qt

import utils.message_conversions as umc




class RotorSFollowerPlannerNode():

    def __init__(self):
        pass
        
     
    def _get_leader_pos_vel(self, msg):
        self._lead_pos, self._lead_vel, aux3, aux4, aux5, aux6 = umc.odometry_msg_to_reference_point(msg)
        self._got_leader_pos_vel_flag = True
        
    
    def _get_quad_initial_pos(self, msg):
        """This is called only once to get the initial position of the quad, in
        case one wants to use it as the starting point for the reference
        trajectory.
        This callback kills the subcriber object herself at the end of the call.
        """
        
        self._quad_initial_pos, dummy, dummy, dummy, dummy, dummy = umc.odometry_msg_to_reference_point(msg)
        self._sub.unregister()
        self._got_quad_initial_pos_flag = True
    

    def work(self):

        # initialize node
        rospy.init_node('rotors_follower_planner_node')
        
        # instantiate the publisher
        topic = rospy.get_param('follower_reference_trajectory_topic', default='command/trajectory')
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # to get the initial position of the quad
        self._quad_initial_pos = None
        self._got_quad_initial_pos_flag = False
        topic = rospy.get_param('follower_pose_topic', default='ground_truth/odometry')
        self._sub = rospy.Subscriber(topic, nm.Odometry, self._get_quad_initial_pos)

        # subscriber to the position of the leader
        topic = rospy.get_param('leader_state_topic', default='/hummingbird_leader/ground_truth/odometry')
        self._got_leader_pos_vel_flag = False
        rospy.Subscriber(topic, nm.Odometry, self._get_leader_pos_vel)

        # setting the frequency of execution
        rate = rospy.Rate(1e1)

        # get initial quad position
        while not self._got_leader_pos_vel_flag and not rospy.is_shutdown():
            rate.sleep()

        # desired offset with respect to the leader
        self._offset = numpy.array([1.0, 0.0, 0.0, 0.0])
        #self._trajectory = ct.TrajectoryCircle(self._quad_initial_pos, numpy.eye(3), delay, delay+duration, radius, ang_vel)

        # do work
        while not rospy.is_shutdown():

            p = self._lead_pos + self._offset
            v = self._lead_vel
            a = numpy.zeros(4)
            j = numpy.zeros(4)
            sn = numpy.zeros(4)
            cr = numpy.zeros(4)
            #print(p)
            msg = umc.reference_point_to_multidofjointtrajectory_msg(p, v, a, j, sn, cr)
            pub.publish(msg)
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = RotorSFollowerPlannerNode()
    node.work()
    
