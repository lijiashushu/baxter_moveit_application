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
import seaborn

def main():
    rospy.init_node("draw_traj")
    robot = URDF.from_parameter_server("robot_description")

    print " "
    experiment_str = "experiment2"
    print experiment_str

    left_right_distance_z = [0.2 * math.cos(0.0), 0.2 * math.cos(0.52), 0.2 * math.cos(1.05), 0.2 * math.cos(1.57)]
    left_right_distance_x = [-0.2 * math.sin(0.0), -0.2 * math.sin(0.52), -0.2 * math.sin(1.05), -0.2 * math.sin(1.57)];
    left_right_euler_distance = [0, 2 * 0.52, 2 * 1.05, 2 * 1.57]
    constraint_index = 1

    tree = kdl_tree_from_urdf_model(robot)
    leftChain = tree.getChain("base", "left_gripper")
    rightChain = tree.getChain("base", "right_gripper")
    leftFKSolver = PyKDL.ChainFkSolverPos_recursive(leftChain)
    rightFKSolver = PyKDL.ChainFkSolverPos_recursive(rightChain)
    leftJacSolver = PyKDL.ChainJntToJacSolver(leftChain)
    rightJacSolver = PyKDL.ChainJntToJacSolver(rightChain)
    leftFrame = PyKDL.Frame()
    rightFrame = PyKDL.Frame()
    leftPos = PyKDL.Vector()
    rightPos = PyKDL.Vector()
    leftRot = PyKDL.Rotation()
    rightRot = PyKDL.Rotation()
    leftJointArr = PyKDL.JntArray(7)
    rightJointArr = PyKDL.JntArray(7)
    leftJacobian = PyKDL.Jacobian(7)
    rightJacobian = PyKDL.Jacobian(7)



    leftRotMeanErr = []
    rightPosMeanErr = []
    rightRotMeanErr = []

    leftRotSdUpErr = []
    rightPosSdUpErr = []
    rightRotSdUpErr = []

    leftRotSdLowErr = []
    rightPosSdLowErr = []
    rightRotSdLowErr = []

    trajectory_total_time = []
    root_dir = '/home/lijiashushu/ros_ws/src/dual_rrt_star/result/' + experiment_str + '/traj/weight2/data/'
    planning_time = np.loadtxt('/home/lijiashushu/ros_ws/src/dual_rrt_star/result/' + experiment_str + '/traj/weight2/planning_time.txt')
    waypoint_num = np.loadtxt('/home/lijiashushu/ros_ws/src/dual_rrt_star/result/' + experiment_str + '/traj/weight2/waypoints_num.txt')
    print " "
    print "waypoint_num: ", np.mean(waypoint_num)
    print "with the close chain constraint "
    print "avg plan time: ", np.mean(planning_time)
    for num in range(planning_time.shape[0]):
        time = np.loadtxt(root_dir + str(num) + "_time.txt")
        position = np.loadtxt(root_dir + str(num) + "_angle.txt")
        velocity = np.loadtxt(root_dir + str(num) + "_velocity.txt")
        acceleration = np.loadtxt(root_dir + str(num) + "_acceleration.txt")

        trajectory_total_time.append(time[-1])

        timeDraw = []
        leftRotErrDraw = []
        rightPosErrDraw = []
        rightRotErrDraw = []

        for i in range(position.shape[0]):
            for j in range(7):
                leftJointArr[j] = position[i, j]
                rightJointArr[j] = position[i, j + 7]

            leftFKSolver.JntToCart(leftJointArr, leftFrame)
            rightFKSolver.JntToCart(rightJointArr, rightFrame)

            leftPos = leftFrame.p
            rightPos = rightFrame.p
            leftRot = leftFrame.M
            rightRot = rightFrame.M

            leftEulerVec = np.zeros(3)
            leftEulerVec[0], leftEulerVec[1], leftEulerVec[2] = leftRot.GetEulerZYX()
            leftGoalEulerVec = np.array([leftEulerVec[0], 0, 1.57])
            leftGoalRot = PyKDL.Rotation.EulerZYX(leftEulerVec[0], 0, 1.57)
            leftEulerErr = leftEulerVec - leftGoalEulerVec

            distance = PyKDL.Vector(left_right_distance_x[constraint_index], 0, left_right_distance_z[constraint_index])
            rightGoalPos = leftGoalRot * distance + leftPos
            rightPosErr = rightPos - rightGoalPos
            rightGoalEulerVec = np.array([leftEulerVec[0]-left_right_euler_distance[constraint_index], 0, -1.57])

            rightEulerVec = np.zeros(3)
            rightEulerVec[0], rightEulerVec[1], rightEulerVec[2] = rightRot.GetEulerZYX()
            rightEulerErr = rightEulerVec - rightGoalEulerVec

            timeDraw.append(time[i])
            leftRotErrDraw.append(np.linalg.norm(leftEulerErr))
            rightPosErrDraw.append(rightPosErr.Norm())
            rightRotErrDraw.append(np.linalg.norm(rightEulerErr))

        # fig0 = plt.figure()
        # ax = fig0.add_subplot(111)
        # # plt.title("PD Tracking Performance", font 1)
        # ax.tick_params(labelsize=size2)
        # plt.plot(timeDraw, leftRotErrDraw, linewidth=linesize, color='red', label='left_rot_err')
        # plt.plot(timeDraw, rightPosErrDraw, linewidth=linesize, color='blue', label='right_pos_err')
        # plt.plot(timeDraw, rightRotErrDraw, linewidth=linesize, color='green', label='right_rot_err')
        # ax.set_xlim(0.0, timeDraw[-1])
        # # ax.set_ylim(0.0, 0.05)
        # plt.legend(loc='best')
        # plt.show()
        # if np.mean(leftRotErrDraw) + np.std(leftRotErrDraw) > 0.04 or np.mean(rightPosErrDraw) - np.std(rightPosErrDraw) > 0.04 or\
        #     np.mean(rightRotErrDraw) + np.std(rightRotErrDraw) > 0.04:
        #     print num
        #     continue
        if num == 88 or num == 91:
            continue

        leftRotErrDraw = np.asarray(leftRotErrDraw)
        rightPosErrDraw = np.asarray(rightPosErrDraw)
        rightRotErrDraw = np.asarray(rightRotErrDraw)
        leftRotMeanErr.append(np.mean(leftRotErrDraw))
        rightPosMeanErr.append(np.mean(rightPosErrDraw))
        rightRotMeanErr.append(np.mean(rightRotErrDraw))

        leftRotSdUpErr.append(np.mean(leftRotErrDraw) + np.std(leftRotErrDraw))
        rightPosSdUpErr.append(np.mean(rightPosErrDraw) + np.std(rightPosErrDraw))
        rightRotSdUpErr.append(np.mean(rightRotErrDraw) + np.std(rightRotErrDraw))

        leftRotSdLowErr.append(np.mean(leftRotErrDraw) - np.std(leftRotErrDraw))
        rightPosSdLowErr.append(np.mean(rightPosErrDraw) - np.std(rightPosErrDraw))
        rightRotSdLowErr.append(np.mean(rightRotErrDraw) - np.std(rightRotErrDraw))
    print "avg trajectory time: ", np.asarray(trajectory_total_time).mean()
    print "avg left rot deviation: ", np.asarray(leftRotMeanErr).mean()
    print "avg right pos deviation: ", np.asarray(rightPosMeanErr).mean()
    print "avg right rot deviation: ", np.asarray(rightRotMeanErr).mean()

    root_dir = '/home/lijiashushu/ros_ws/src/dual_rrt_star/result/' + experiment_str + '/traj/weight0/data/'
    planning_time_no_soft = np.loadtxt('/home/lijiashushu/ros_ws/src/dual_rrt_star/result/' + experiment_str + '/traj/weight0/planning_time.txt')
    print "\n\nwithout the close chain constraint"
    print "avg plan time no soft: ", np.mean(planning_time_no_soft)
    leftRotMeanErrNoSoft = []
    rightPosMeanErrNoSoft = []
    rightRotMeanErrNoSoft = []

    leftRotSdUpErrNoSoft = []
    rightPosSdUpErrNoSoft = []
    rightRotSdUpErrNoSoft = []

    leftRotSdLowErrNoSoft = []
    rightPosSdLowErrNoSoft = []
    rightRotSdLowErrNoSoft = []

    trajectory_total_time = []
    for num in range(planning_time.shape[0]):
        time = np.loadtxt(root_dir + str(num) + "_time.txt")
        position = np.loadtxt(root_dir + str(num) + "_angle.txt")
        velocity = np.loadtxt(root_dir + str(num) + "_velocity.txt")
        acceleration = np.loadtxt(root_dir + str(num) + "_acceleration.txt")
        trajectory_total_time.append(time[-1])

        timeDraw = []
        leftRotErrDraw = []
        rightPosErrDraw = []
        rightRotErrDraw = []

        for i in range(position.shape[0]):
            for j in range(7):
                leftJointArr[j] = position[i, j]
                rightJointArr[j] = position[i, j + 7]

            leftFKSolver.JntToCart(leftJointArr, leftFrame)
            rightFKSolver.JntToCart(rightJointArr, rightFrame)

            leftPos = leftFrame.p
            rightPos = rightFrame.p
            leftRot = leftFrame.M
            rightRot = rightFrame.M

            leftEulerVec = np.zeros(3)
            leftEulerVec[0], leftEulerVec[1], leftEulerVec[2] = leftRot.GetEulerZYX()
            leftGoalEulerVec = np.array([leftEulerVec[0], 0, 1.57])
            leftGoalRot = PyKDL.Rotation.EulerZYX(leftEulerVec[0], 0, 1.57)
            leftEulerErr = leftEulerVec - leftGoalEulerVec

            distance = PyKDL.Vector(left_right_distance_x[constraint_index], 0, left_right_distance_z[constraint_index])
            rightGoalPos = leftGoalRot * distance + leftPos
            rightPosErr = rightPos - rightGoalPos
            rightGoalEulerVec = np.array([leftEulerVec[0] - left_right_euler_distance[constraint_index], 0, -1.57])

            rightEulerVec = np.zeros(3)
            rightEulerVec[0], rightEulerVec[1], rightEulerVec[2] = rightRot.GetEulerZYX()
            rightEulerErr = rightEulerVec - rightGoalEulerVec

            timeDraw.append(time[i])
            leftRotErrDraw.append(np.linalg.norm(leftEulerErr))
            rightPosErrDraw.append(rightPosErr.Norm())
            rightRotErrDraw.append(np.linalg.norm(rightEulerErr))

        # if np.mean(leftRotErrDraw) + np.std(leftRotErrDraw) > 0.04 or np.mean(rightPosErrDraw) - np.std(rightPosErrDraw) > 0.04 or \
        #         np.mean(rightRotErrDraw) + np.std(rightRotErrDraw) > 0.04:
        #     print num
        #     continue
        if num == 88 or num == 91:
            continue


        leftRotErrDraw = np.asarray(leftRotErrDraw)
        rightPosErrDraw = np.asarray(rightPosErrDraw)
        rightRotErrDraw = np.asarray(rightRotErrDraw)
        leftRotMeanErrNoSoft.append(np.mean(leftRotErrDraw))
        rightPosMeanErrNoSoft.append(np.mean(rightPosErrDraw))
        rightRotMeanErrNoSoft.append(np.mean(rightRotErrDraw))

        leftRotSdUpErrNoSoft.append(np.mean(leftRotErrDraw) + np.std(leftRotErrDraw))
        rightPosSdUpErrNoSoft.append(np.mean(rightPosErrDraw) + np.std(rightPosErrDraw))
        rightRotSdUpErrNoSoft.append(np.mean(rightRotErrDraw) + np.std(rightRotErrDraw))

        leftRotSdLowErrNoSoft.append(np.mean(leftRotErrDraw) - np.std(leftRotErrDraw))
        rightPosSdLowErrNoSoft.append(np.mean(rightPosErrDraw) - np.std(rightPosErrDraw))
        rightRotSdLowErrNoSoft.append(np.mean(rightRotErrDraw) - np.std(rightRotErrDraw))


    print "avg trajectory time: ", np.asarray(trajectory_total_time).mean()
    print "avg left rot deviation: ", np.asarray(leftRotMeanErrNoSoft).mean()
    print "avg right pos deviation: ", np.asarray(rightPosMeanErrNoSoft).mean()
    print "avg right rot deviation: ", np.asarray(rightRotMeanErrNoSoft).mean()

    size1 = 17
    size2 = 25
    linesize = 2
    font2 = {'family': 'Times New Roman',
                  'weight': 'normal',
                  'size': 40,
                  }
    draw_x = range(len(leftRotMeanErrNoSoft))
    fig0 = plt.figure(figsize=(25, 14))
    ax = fig0.add_subplot(311)
    # plt.title("PD Tracking Performance", font 1)
    ax.tick_params(labelsize=size2)
    plt.plot(draw_x, leftRotMeanErrNoSoft, linewidth=linesize, linestyle='-', color='blue')
    plt.fill_between(draw_x, leftRotSdUpErrNoSoft, leftRotSdLowErrNoSoft, edgecolor='lightskyblue', facecolor='lightskyblue')
    plt.plot(draw_x, leftRotMeanErr, linewidth=linesize, color='red')
    plt.fill_between(draw_x, leftRotSdUpErr, leftRotSdLowErr, edgecolor='lightcoral', facecolor='lightcoral')
    ax.set_xlim(0.0, draw_x[-1])
    plt.ylabel("Master Rot [rad]", font2)
    # ax.set_ylim()
    plt.legend(loc='best')

    # fig1 = plt.figure()
    ax = fig0.add_subplot(312)
    # plt.title("PD Tracking Performance", font 1)
    ax.tick_params(labelsize=size2, )
    plt.plot(draw_x, rightPosMeanErrNoSoft, linewidth=linesize, color='blue')
    plt.fill_between(draw_x, rightPosSdUpErrNoSoft, rightPosSdLowErrNoSoft, edgecolor='lightskyblue', facecolor='lightskyblue')
    plt.plot(draw_x, rightPosMeanErr, linewidth=linesize, color='red')
    plt.fill_between(draw_x, rightPosSdUpErr, rightPosSdLowErr, edgecolor='lightcoral', facecolor='lightcoral')
    plt.ylabel("Slave Pos [m]", font2)
    ax.set_xlim(0.0, draw_x[-1])
    # ax.set_ylim(0.0, 0.025)
    plt.legend(loc='best')

    # fig2 = plt.figure()
    ax = fig0.add_subplot(313)
    # plt.title("PD Tracking Performance", font 1)
    ax.tick_params(labelsize=size2)
    plt.plot(draw_x, rightRotMeanErrNoSoft, linewidth=linesize, color='blue')
    plt.fill_between(draw_x, rightRotSdUpErrNoSoft, rightRotSdLowErrNoSoft, edgecolor='lightskyblue', facecolor='lightskyblue')
    plt.plot(draw_x, rightRotMeanErr, linewidth=linesize, color='red')
    plt.fill_between(draw_x, rightRotSdUpErr, rightRotSdLowErr, edgecolor='lightcoral', facecolor='lightcoral')
    plt.ylabel("Slave Rot [rad]", font2)
    ax.set_xlim(0.0, draw_x[-1])
    # ax.set_ylim(0.0, 0.025)
    plt.legend(loc='best')
    plt.savefig('/home/lijiashushu/scene2_traj')
    plt.show()


if __name__ == '__main__':
    main()

    # a = [1, 2, 3]
    # b = [4, 5, 6]
    # print np.r_[a, b]
