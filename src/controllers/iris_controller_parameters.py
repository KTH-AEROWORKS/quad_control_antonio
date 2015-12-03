"""This file collects some parameters for the controller of the quad.
The parameters are not defined directly here, but they are defined in a launch
file.
This file takes out the parameters from the launch file, but also gives a
default value for each parameter.
In this way, ROS is able to change the values of the parameters online, if
desired.
For example, we do such changes in the GUI.
"""

import rospy

import numpy as np


# gravity
g = rospy.get_param("g_ctr", 9.81)

# quad mass
quad_mass = rospy.get_param("quad_mass_ctr", 1.442)

# vertical
e3 = np.array([0.0, 0.0, 1.0])

# controller gains
omega_n = 1.0
xi = np.sqrt(2) / 2
kv = rospy.get_param("kv", 2.0 * xi * omega_n)
kp = rospy.get_param("kp", omega_n**2)

# controller thresholds
sp = rospy.get_param("sp", 1.0)
sv = rospy.get_param("sv", 1.0)

# thrust that cancels the gravity
neutral_thrust = rospy.get_param("neutral_thrust_ctr", 1430.0)

# angular velocity sensitivity for the acro mode
acro_rpp = rospy.get_param("acro_rpp_ctr", 4.5)

# yaw gain
k_yaw = rospy.get_param("k_yaw_ctr", 3.0)

# desired yaw
psi_des = 0.0

# maximum roll and pitch angle
max_tilt_deg = rospy.get_param("max_tilt_deg_ctr", 45.0)

# maximum yaw rate
max_yaw_rate_deg = rospy.get_param("max_yaw_rate_deg_ctr", 200.0)

# neutral control
NEUTRAL_CONTROL_INPUT = [1500.0, 1500.0, 1500.0, 1500.0]

# conventional initial position of a quad
QUAD_INITIAL_POSITION = np.array([0.0, 0.0, 1.0])

# thresholds
MAX_TILT = np.pi/4
MAX_YAW_RATE = np.pi/10
