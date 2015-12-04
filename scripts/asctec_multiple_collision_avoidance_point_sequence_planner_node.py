#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is planned as a sequence of randomly selected waypoints, plus a
conrtibution on acceleration for collision avoidance.
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
import random

import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm
import nav_msgs.msg as nm

import planners.trajectory_circle as ct
import planners.trajectory_cubic as cbt
import planners.trajectory_quintic as qt
import planners.planner_collision_avoidance_multiple as pcam
import planners.planner_to_goal as ptg

import utils.message_conversions as umc



class PartnerQuad():

    def __init__(self):
    
        self.pos = None
        self.got_initial_pos_flag = False



class AscTecMultipleCollisionAvoidancePointSequencePlannerNode():

    def __init__(self):
        pass


    def _get_quad_pos_vel(self, msg):
        """Callback to save the position and velocity of the quad into
        object properties.
        """
        
        self._pos, self._vel, dummy, dummy, dummy, dummy = umc.odometry_msg_to_reference_point(msg)
        self._got_initial_pos_vel_flag = True  
    

    def _get_other_pos(self, msg, quad_name):
    
        self._other_quads[quad_name].pos, dummy, dummy, dummy, dummy, dummy = umc.odometry_msg_to_reference_point(msg)
        self._other_quads[quad_name].got_initial_pos_flag = True
        

    def _generate_initial_waypoint(self):
        if self._pos[2] < 1.0:
            self._waypoint = numpy.array(self._pos)
            self._waypoint[2] = 1.0
            #duration = 2.5*numpy.linalg.norm(self._waypoint-self._pos)
            #time = rospy.get_time() - self.initial_time
            #delay = rospy.get_param('delay', default=1.0)
            #self._roaming_planner = cbt.TrajectoryCubic(self._pos, numpy.eye(3), delay+time, delay+time+duration, self._waypoint)
            self._roaming_planner = ptg.PlannerToGoal(self._waypoint, 1.0, 5.0)
        else:
            self._generate_waypoint()


    def _generate_waypoint(self):
        self._waypoint = numpy.array(self._pos)
        distance = numpy.linalg.norm(self._waypoint-self._pos)
        while distance < 0.1:
            x = random.uniform(-2.0, 2.0)
            y = random.uniform(-2.0, 2.0)
            z = random.uniform(0.5, 1.8)
            yaw = random.uniform(-numpy.pi, numpy.pi)
            self._waypoint = numpy.array([x, y, z, yaw])
            distance = numpy.linalg.norm(self._waypoint-self._pos)
        #duration = 2.0*numpy.linalg.norm(self._waypoint-self._pos)
        #time = rospy.get_time() - self.initial_time
        #delay = rospy.get_param('delay', default=1.0)
        #self._roaming_planner = cbt.TrajectoryCubic(self._pos, numpy.eye(3), delay+time, delay+time+duration, self._waypoint)
        self._roaming_planner = ptg.PlannerToGoal(self._waypoint, 1.0, 5.0)



    def work(self):

        # initialize node
        rospy.init_node('asctec_multiple_collision_avoidance_point_sequence_planner_node')
        
        # planner to compute the collision avoidance contributions
        gain = 1.0
        ths = 0.5
        self._collision_avoidance_planner = pcam.PlannerCollisionAvoidanceMultiple(gain, ths)
        
        # planner to roam around
        # it will be set up in the callback
        self._roaming_planner = None
        
        # instantiate the publisher
        topic = rospy.get_param('ref_traj_topic', default='command/trajectory')
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # to get the initial position of the quad
        self._pos = None
        self._vel = None
        self._got_initial_pos_vel_flag = False
        topic = rospy.get_param('quad_pos_vel_topic', default='msf_core/odometry')
        self._pos_subscriber = rospy.Subscriber(topic, nm.Odometry, self._get_quad_pos_vel)

        # subscribers to the positions of the other quads
        self._other_quads = {}
        other_quads_names_string = rospy.get_param('other_quads_names_string', default="")
        other_quads_names_list = other_quads_names_string.split()
        for quad_name in other_quads_names_list:
            self._other_quads[quad_name] = PartnerQuad()
            rospy.Subscriber("/" + quad_name + "/msf_core/odometry", nm.Odometry, self._get_other_pos, quad_name)

        # setting the frequency of execution
        freq = 30.0
        rate = rospy.Rate(freq)

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
            print("The planner is waiting for the quad initial position.")
            rate.sleep()

        # get initial position of the other
        while not all([partner.got_initial_pos_flag for partner in self._other_quads.values()]) and not rospy.is_shutdown():
            print("The planner is waiting for the partners initial positions.")
            print self._other_quads.keys()
            rate.sleep()

        # generate initial waypoint
        self._generate_initial_waypoint()
        waypoint_counter = 0

        # do work
        while not rospy.is_shutdown():

            # compute collision avoidance contribution
            others_poss = [partner.pos for partner in self._other_quads.values()]
            #ca_acc = self._collision_avoidance_planner.get_acceleration(self._pos, self._vel, others_poss)
            ca_vel = self._collision_avoidance_planner.get_velocity(self._pos, others_poss)

            # get current time
            #self.time = rospy.get_time() - self.initial_time
            #print(self.time)
            
            # see if the current waypoint is reached
            #print self._pos
            #print self._waypoint
            #print numpy.linalg.norm(self._pos-self._waypoint)
            
            if numpy.linalg.norm(self._pos-self._waypoint) < 0.15:
                #print("\nNEW WAYPOINT!!!: " + str(waypoint_counter) + "\n")
                #print("\nTIME: " + str(rospy.get_time()) + "\n")
                waypoint_counter += 1
                self._generate_waypoint()
                
            #print("TO GOAL: " + str(numpy.linalg.norm(self._pos-self._waypoint)))
            print("GOAL: " + str(self._waypoint))
            print("CURRENT: " + str(self._pos))
            print("TIME: " + str(rospy.get_time()))
            print("WAYPOINTS REACHED: " + str(waypoint_counter))
            
            #r_acc = self._roaming_planner.get_acceleration(self._pos, self._vel)
            r_vel = self._roaming_planner.get_velocity(self._pos)

            #acc = ca_acc + r_acc
            vel = ca_vel + r_vel
            
            #aux = self._roaming_planner.get_velocity(self._pos)
            #vel = numpy.array(self._vel)
            #vel[3] = aux[3]
            
            pos = numpy.array(self._pos)
            
            acc = numpy.zeros(4)
            j = numpy.zeros(4)
            sn = numpy.zeros(4)
            cr = numpy.zeros(4)

            msg = umc.reference_point_to_multidofjointtrajectory_msg(pos, vel, acc, j, sn, cr)
            pub.publish(msg)
            
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = AscTecMultipleCollisionAvoidancePointSequencePlannerNode()
    node.work()
    
