#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from robot import Robot

def updateWorld(msg):
    joint_states = np.array(msg.position[0:4])
    soft_snake.setJointStates(joint_states)

def cb_velocity(msg):
    J = soft_snake.jacobianComputation()
    q = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]).T
    w = np.dot(np.linalg.pinv(J), q) *0.001
    joint_cmd = Float64MultiArray()
    joint_cmd.data = [0, 0, w[0], w[1], w[2], w[3]]
    pub.publish(joint_cmd)

soft_snake = Robot([0, 0, 0, 0, 0, 0], [1, 1, 1, 1], 0, 0, 0)

rospy.init_node('compliant_control')

rospy.Subscriber('/joint_states', JointState, updateWorld)

rospy.Subscriber('/cmd_vel', Twist, cb_velocity)

pub = rospy.Publisher('/manipulator_velocity_controller/command', Float64MultiArray, queue_size=10)

while not rospy.is_shutdown():
    pass