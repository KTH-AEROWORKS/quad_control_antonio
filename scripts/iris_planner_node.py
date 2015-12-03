#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is published in the form of a ROS message of the type
'MultiDOFJointTrajectory', and a description can be found at this link.

http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html 

The actual trajectory is generated with one of the functions in the module
'trajectories' of this package.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""


import rospy
import numpy
import quad_control.msg as qcm
import planners.trajectory_circle as ct




def trajectory_to_referencetrajectorymsg(p, v, a, j, s, c):
    """Function to transform a 'Trajectory' object into a
    'ReferenceTrajectoryMsg object'.
    """

    msg = qcm.ReferenceTrajectoryMsg()

    msg.p = p
    msg.v = v
    msg.a = a
    msg.j = j
    msg.s = s
    msg.c = c

    return msg


def work():

    # initialize node
    rospy.init_node('iris_planner_node')

    # get initial time
    initial_time = rospy.get_time()

    # topic to publish on
    topic = 'iris_reference_trajectory'

    # instantiate the publisher
    pub = rospy.Publisher(topic, qcm.ReferenceTrajectoryMsg, queue_size=10)

    # setting the frequency of execution
    rate = rospy.Rate(1e2)

    # trajectory to be published
    trajectory = ct.TrajectoryCircle(
        [0.0, 0.0, 1.0, numpy.pi], numpy.eye(3), 2.0, 0.3)

    # do work
    while not rospy.is_shutdown():

        time = rospy.get_time() - initial_time
        #print time
        p, v, a, j, s, c = trajectory.get_point(time)
        msg = trajectory_to_referencetrajectorymsg(p, v, a, j, s, c)
        pub.publish(msg)
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    work()
    
