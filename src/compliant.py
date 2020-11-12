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
    soft_snake.setJointStates(msg.position[0:3])

def cb_velocity(msg):
    J = soft_snake.jacobianComputation()
    w = np.linalg.inv(J)@[msg.linear msg.angular]
    #pub([0 0 w])

soft_snake = Robot([0 0 0 0 0 0], [0 0 0 0], 0, 0, 0)

rospy.init_node('compliant_control')

rospy.Subscriber('/joint_states', JointState, updateWorld)

rospy.Subscriber('/cmd_vel', Twist, cb_velocity)

pub = rospy.Publisher('/manipulator_velocity_controller/command', Float64MultiArray, queue_size=10)