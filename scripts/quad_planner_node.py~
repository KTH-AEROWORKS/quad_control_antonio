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
import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm
import planners.trajectories as pt
import tf.transformations as tft

# trajectory to be published
trajectory = pt.CircleTrajectory(
    [0.0, 0.0, 1.0, numpy.pi], numpy.eye(3), 2.0, 0.3)

# convert the trajectory to a message


def trajectory_to_multidofjointtrajectory(trajectory_value):
    """Function to transform a 'Trajectory' object into a
    'MultiDOFJointTrajectoryPoint object'.
    """

    msg = tm.MultiDOFJointTrajectory()
    point = tm.MultiDOFJointTrajectoryPoint()
    msg.points.append(point)

    p, v, a, j, s, c = trajectory_value

    transform = gm.Transform()
    transform.translation.x = p[0]
    transform.translation.y = p[1]
    transform.translation.z = p[2]
    quaternion = tft.quaternion_from_euler(0.0, 0.0, p[3])
    transform.rotation.x = quaternion[0]
    transform.rotation.y = quaternion[1]
    transform.rotation.z = quaternion[2]
    transform.rotation.w = quaternion[3]
    point.transforms.append(transform)

    velocity = gm.Twist()
    velocity.linear.x = v[0]
    velocity.linear.y = v[1]
    velocity.linear.y = v[2]
    point.velocities.append(velocity)

    acceleration = gm.Twist()
    acceleration.linear.x = a[0]
    acceleration.linear.y = a[1]
    acceleration.linear.z = a[2]
    point.accelerations.append(acceleration)

    return msg


def work():

    # initialize node
    rospy.init_node('quad_planner_node')

    # get initial time
    initial_time = rospy.get_time()

    # topic to publish on
    topic = 'quad_reference_trajectory'

    # instantiate the publisher
    pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

    # setting the frequency of execution
    rate = rospy.Rate(1e2)

    # do work
    while not rospy.is_shutdown():

        time = rospy.get_time() - initial_time
        trajectory_value = trajectory.output(time)
        msg = trajectory_to_multidofjointtrajectory(trajectory_value)
        pub.publish(msg)
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    work()
    
