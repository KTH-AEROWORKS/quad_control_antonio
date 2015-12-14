"""This file implements the conversions from ROS messages to numpy arrays."""

import numpy
import utility_functions as uf

import geometry_msgs.msg as gm
import trajectory_msgs.msg as tm

def reference_point_to_multidofjointtrajectory_msg(p, v, a, j, sn, cr):

    msg = tm.MultiDOFJointTrajectory()
    point = tm.MultiDOFJointTrajectoryPoint()
    msg.points.append(point)

    #print(p)

    transform = gm.Transform()
    transform.translation.x = p[0]
    transform.translation.y = p[1]
    transform.translation.z = p[2]
    rot = uf.ea_to_rot([0.0, 0.0, p[3]])
    quaternion = uf.rot_to_quaternion(rot)
    transform.rotation.x = quaternion[0]
    transform.rotation.y = quaternion[1]
    transform.rotation.z = quaternion[2]
    transform.rotation.w = quaternion[3]
    point.transforms.append(transform)

    velocity = gm.Twist()
    velocity.linear.x = v[0]
    velocity.linear.y = v[1]
    velocity.linear.z = v[2]
    velocity.angular.z = v[3]
    point.velocities.append(velocity)

    acceleration = gm.Twist()
    acceleration.linear.x = a[0]
    acceleration.linear.y = a[1]
    acceleration.linear.z = a[2]
    point.accelerations.append(acceleration)

    return msg
    
    
def odometry_msg_to_reference_point(msg):
    """This function converts a message of type geomtry_msgs.Odometry into
    position and velocity of the quad as 4D numpy arrays.
    """

    aux1 = msg.pose.pose.position
    aux2 = msg.pose.pose.orientation
    quaternion = numpy.array([aux2.x, aux2.y, aux2.z, aux2.w])
    rot = uf.quaternion_to_rot(quaternion)
    ea = uf.rot_to_ea(rot)
    yaw = ea[2]
    pos = numpy.array([aux1.x, aux1.y, aux1.z, yaw])
    aux3 = msg.twist.twist.linear
    vel_body = numpy.array([aux3.x, aux3.y, aux3.z])
    vel_world = rot.dot(vel_body)
    aux4 = msg.twist.twist.angular
    vel = numpy.concatenate([vel_world, numpy.array([aux4.z])])

    acc = numpy.zeros(4)
    jerk = numpy.zeros(4)
    snap = numpy.zeros(4)
    crackle = numpy.zeros(4)

    return pos, vel, acc, jerk, snap, crackle
