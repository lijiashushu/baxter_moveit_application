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
import os
from matplotlib import rcParams
rcParams['font.family']='Times New Roman'



def main():
    size1 = 17
    size2 = 12
    linesize = 2
    # robot = URDF.from_xml_file("/home/lijiashushu/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")
    # num = 10
    # dual_data = np.loadtxt("/home/lijiashushu/ros_ws/src/dual_rrt_star/result/dual/" + str(num) + ".txt")
    # dual_star_data = np.loadtxt("/home/lijiashushu/ros_ws/src/dual_rrt_star/result/dual_star/" + str(num) + ".txt")
    # dual_informed_data = np.loadtxt("/home/lijiashushu/ros_ws/src/dual_rrt_star/result/dual_informed_star/" + str(num) + ".txt")
    # fig0 = plt.figure()
    # ax = fig0.add_subplot(111)
    # # plt.title("PD Tracking Performance", font 1)
    # ax.tick_params(labelsize=size2)
    # plt.plot(dual_data[:, 0], dual_data[:, 1], linewidth=linesize, color='red', label='dual')
    # plt.plot(dual_star_data[:, 0], dual_star_data[:, 1], linewidth=linesize, color='blue', label='dual_rrt')
    # plt.plot(dual_informed_data[:, 0], dual_informed_data[:, 1], linewidth=linesize, color='green', label='dual_informed_rrt')
    # ax.set_xlim(0.0, 5)
    # # ax.set_ylim(0.0, 0.05)
    # plt.legend(loc='best')
    # plt.show()
    experiment_str = "experiment2"

    time_draw = 0.01 * np.asarray(range(500))
    dual_data_2 = np.zeros(500, dtype=float)
    dual_star_data_2 = np.zeros(500, dtype=float)
    dual_informed_data_2 = np.zeros(500, dtype=float)
    joint_data_2 = np.zeros(500, dtype=float)

    root_dir = "/home/lijiashushu/ros_ws/src/dual_rrt_star/result/" + experiment_str + "/dual"
    cat = os.listdir(root_dir)
    iteration_num = 100

    for num in range(iteration_num):
        # print num
        dual_data = np.loadtxt("/home/lijiashushu/ros_ws/src/dual_rrt_star/result/" + experiment_str + "//dual/" + str(num) + ".txt")
        dual_star_data = np.loadtxt("/home/lijiashushu/ros_ws/src/dual_rrt_star/result/" + experiment_str + "//dual_star/" + str(num) + ".txt")
        dual_informed_data = np.loadtxt("/home/lijiashushu/ros_ws/src/dual_rrt_star/result/" + experiment_str + "//dual_informed_star/" + str(num) + ".txt")
        joint_data = np.loadtxt("/home/lijiashushu/ros_ws/src/dual_rrt_star/result/" + experiment_str + "//joint_space/" + str(num) + ".txt")

        a_index = 0
        b_index = 0
        c_index = 0
        d_index = 0
        for i in range(500):
            print i
            required_t = i * 0.01
            while(required_t > dual_data[a_index, 0]):
                if not a_index == dual_data.shape[0] - 1:
                    a_index = a_index + 1
                else:
                    break

            while (required_t > dual_star_data[b_index, 0]):
                if not b_index == dual_star_data.shape[0] - 1:
                    b_index = b_index + 1
                else:
                    break

            while (required_t > dual_informed_data[c_index, 0]):
                if not c_index == dual_informed_data.shape[0] - 1:
                    c_index = c_index + 1
                else:
                    break

            while (required_t > joint_data[d_index, 0]):
                if not d_index == joint_data.shape[0] - 1:
                    d_index = d_index + 1
                else:
                    break

            if  dual_data[a_index, 1] == 3:
                dual_data[a_index, 1] = 5
            if dual_star_data[b_index, 1] ==3:
                dual_star_data[b_index, 1] =5
            if dual_informed_data[c_index, 1] == 3:
                dual_informed_data[c_index, 1] = 5
            if joint_data[d_index, 1] == 3:
                joint_data[d_index, 1] = 5

            dual_data_2[i] = dual_data_2[i] + dual_data[a_index, 1]
            dual_star_data_2[i] = dual_star_data_2[i] + dual_star_data[b_index, 1]
            dual_informed_data_2[i] = dual_informed_data_2[i] + dual_informed_data[c_index, 1]
            joint_data_2[i] = joint_data_2[i] + joint_data[d_index, 1]

    dual_data_2 = dual_data_2 / iteration_num
    dual_star_data_2 = dual_star_data_2 / iteration_num
    dual_informed_data_2 = dual_informed_data_2 / iteration_num
    joint_data_2 = joint_data_2 / iteration_num

    size1 = 17
    size2 = 18
    linesize = 2
    font2 = {'family': 'Times New Roman',
             'weight': 'normal',
             'size': 22,
             }

    fig1 = plt.figure(figsize=(8.0, 6.5))
    ax = fig1.add_subplot(111)
    # plt.title("PD Tracking Performance", font 1)
    ax.tick_params(labelsize=size2)
    plt.plot(time_draw, joint_data_2, linewidth=linesize, color='black', label=r'B$\mathrm{I}^2$RRT*')
    plt.plot(time_draw, dual_data_2, linewidth=linesize, color='red', label='TSGD-BIRRT')
    plt.plot(time_draw, dual_star_data_2, linewidth=linesize, color='blue', label='TSGD-BIRRT*')
    plt.plot(time_draw, dual_informed_data_2, linewidth=linesize, color='green', label=r'TSGD-B$\mathrm{I}^2$RRT*')
    ax.set_xlim(0.0, 5)
    # ax.set_ylim(0.0, 0.05)
    plt.xlabel("Planning Time [s]", font2)
    plt.ylabel("Path Cost [m]", font2)
    plt.legend(loc='best', fontsize=16)

    print experiment_str
    print "joint space: ", joint_data_2[-1]
    print "dual rrt: ", dual_data_2[-1]
    print "dual rrt*: ", dual_star_data_2[-1]
    print "informed dual rrt*: ", dual_informed_data_2[-1]
    plt.show()


if __name__ == '__main__':
    main()

    # a = [1, 2, 3]
    # b = [4, 5, 6]
    # print np.r_[a, b]
