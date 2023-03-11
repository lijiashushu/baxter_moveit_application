#!/usr/bin/python
# -*- coding:utf-8 -*-
'''
@Time : 2019/05/27 21:16
@Author :Li Jiashu
@E-mail :lijiashushu@163com
@QQ :1738496395
@Modified:
@E-mail :
@QQ :
只在关节空间进行控制
'''
import math
import numpy as np
import baxter_interface
import sys
import rospy
import matplotlib.pyplot as plt
import time
from multiprocessing import Process, Queue
import multiprocessing as mp
import time
import random
import PyKDL
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from mpl_toolkits.mplot3d import Axes3D
import baxter_pykdl
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.init_node('publish_node')
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()

    left_joint_name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0','left_w1','left_w2']
    right_joint_name = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0','right_w1','right_w2']
    hello_str.name = left_joint_name + right_joint_name

    left_position = [0.514998,-0.487572,-1.79923,1.6679,-0.28682,0.603706,2.86722]
    right_position = [-0.517204,-0.49348,1.79396,1.6679,0.302716,0.602833,-2.87906]
    hello_str.position = left_position + right_position

    hello_str.velocity = []
    hello_str.effort = []

    while not rospy.is_shutdown():
        # 发布消息
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    talker()

    # a = [1, 2, 3]
    # b = [4, 5, 6]
    # print np.r_[a, b]
