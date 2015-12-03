#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is published in the form of a ROS message of the type
'MultiDOFJointTrajectory', and a description can be found at this link.

http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html 

Such ROS message can be taken as a reference by the quad models in the RotorS
simulator.

The actual trajectory is generated as a sequence of waypoint.
Each navigation segment is generated with one of the functions in the module
'trajectories' of this package.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""


import rospy

import numpy
import random

import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm
import nav_msgs.msg as nm

import planners.trajectory_circle as ct
import planners.trajectory_cubic as cbt
import planners.trajectory_quintic as qt
import planners.planner_to_goal as ptg

import utils.message_conversions as umc




class AscTecPointSequencePlannerNode():

    def __init__(self):
        pass


    def _get_quad_pos_vel(self, msg):
        """Callback to save the position and velocity of the quad into
        object properties.
        """
        
        self._pos, self._vel, dummy, dummy, dummy, dummy = umc.odometry_msg_to_reference_point(msg)
        self._got_initial_pos_vel_flag = True
    
    
    def _generate_initial_waypoint(self):
        if self._pos[2] < 1.0:
            self._waypoint = numpy.array(self._pos)
            self._waypoint[2] = 1.0
            #duration = 2.5*numpy.linalg.norm(self.waypoint-self.quad_pose)
            #time = rospy.get_time() - self.initial_time
            #delay = rospy.get_param('delay', default=1.0)
            #self.trajectory = qt.TrajectoryQuintic(self.quad_pose, numpy.eye(3), delay+time, delay+time+duration, self.waypoint)
            gain_pos = 1.0
            gain_vel = 3.0
            self._planner = ptg.PlannerToGoal(self._waypoint, gain_pos, gain_vel)
        else:
            self._generate_waypoint()


    def _generate_waypoint(self):
        self._waypoint = numpy.array(self._pos)
        distance = numpy.linalg.norm(self._waypoint-self._pos)
        while distance < 0.1:
            x = random.uniform(-1.5, 1.5)
            y = random.uniform(-1.5, 1.5)
            z = random.uniform(0.5, 1.5)
            yaw = random.uniform(-numpy.pi, numpy.pi)
            self._waypoint = numpy.array([x, y, z, yaw])
            distance = numpy.linalg.norm(self._waypoint-self._pos)
        #duration = 2.0*numpy.linalg.norm(self.waypoint-self.quad_pose)
        #time = rospy.get_time() - self.initial_time
        #delay = rospy.get_param('delay', default=1.0)
        #self.trajectory = qt.TrajectoryQuintic(self.quad_pose, numpy.eye(3), delay+time, delay+time+duration, self.waypoint)
        gain_pos = 1.0
        gain_vel = 3.0
        self._planner = ptg.PlannerToGoal(self._waypoint, gain_pos, gain_vel)


    def work(self):

        # initialize node
        rospy.init_node('asctec_point_sequence_planner_node')

        # local time
        #initial_time = rospy.get_time()
        #self.time = rospy.get_time() - initial_time
        
        # instantiate the publisher
        topic = rospy.get_param('ref_traj_topic', default='command/trajectory')
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # to get the pose of the quad
        self._pos = None
        self._vel = None
        self._got_initial_pos_vel_flag = False
        topic = rospy.get_param('quad_pos_topic', default='msf_core/odometry')
        rospy.Subscriber(topic, nm.Odometry, self._get_quad_pos_vel)

        # setting the frequency of execution
        rate = rospy.Rate(1e1)

        # get initial time
        #aux = rospy.get_time()
        #while rospy.get_time() == aux:
        #    pass
        #self.initial_time = rospy.get_time()
        #self.time = rospy.get_time() - self.initial_time
        #print(self.time)
        #print(type(self.initial_time))

        # get initial quad position
        while not self._got_initial_pos_vel_flag and not rospy.is_shutdown():
            print("The planner is waiting for the initial quad position.")
            rate.sleep()

        # generate initial waypoint
        self._generate_initial_waypoint()

        # do work
        while not rospy.is_shutdown():

            # get current time
            #self.time = rospy.get_time() - self.initial_time
            #print(self.time)
            
            # see if the current waypoint is reached
            print self._pos
            print self._waypoint
            print numpy.linalg.norm(self._pos-self._waypoint)
            
            if numpy.linalg.norm(self._pos-self._waypoint) < 0.1:
                self._generate_waypoint()
                waypoint_counter += 1
                print("\nNew waypoint: " + str(waypoint_counter) +"\n")
                
            #p, v, a, j, s, c = self.trajectory.get_point(self.time)
            #a = self._planner.get_acceleration(self._pos, self._vel)
            v = self._planner.get_velocity(self._pos)
            #v = numpy.array(self._vel)
            #aux = self._planner.get_velocity(self._pos)
            #v[3] = aux[3]
            p = numpy.array(self._pos)
            a = numpy.zeros(4)
            j = numpy.zeros(4)
            sn = numpy.zeros(4)
            cr = numpy.zeros(4)
            msg = umc.reference_point_to_multidofjointtrajectory_msg(p, v, a, j, sn, cr)
            
            pub.publish(msg)
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = AscTecPointSequencePlannerNode()
    node.work()
    
