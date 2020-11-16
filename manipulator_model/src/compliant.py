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
    global q
    q = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]).T

soft_snake = Robot([0, 0, 0, 0, 0, 0], [1, 1, 1, 1], 0, 0, 0)
q = np.array([0, 0, 0, 0, 0, 0])
Ts = 0.1

rospy.init_node('compliant_control')

rospy.Subscriber('/joint_states', JointState, updateWorld)

rospy.Subscriber('/cmd_vel', Twist, cb_velocity)

pub = rospy.Publisher('/manipulator_pos_group_controller/command', Float64MultiArray, queue_size=10)

i = 0
while not rospy.is_shutdown():
    J = soft_snake.jacobianComputation()
    w = np.dot(np.linalg.pinv(J), q)*0.0001
    joint_cmd = Float64MultiArray()
    joint_cmd.data = [0, 0, soft_snake.getJointStates()[0] + w[0]*Ts, soft_snake.getJointStates()[1] + w[1]*Ts,
                        soft_snake.getJointStates()[2] + w[2]*Ts, soft_snake.getJointStates()[3] + w[3]*Ts]
    pub.publish(joint_cmd)

    if i > 10:
        print(q, joint_cmd)
        i = 0
    else:
        i+=1
    pass

    rospy.sleep(Ts)