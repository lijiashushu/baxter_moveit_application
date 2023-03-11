#!/usr/bin/python
#-*- coding:utf-8 -*-
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

def left_NN(input, output):


    joint_pos_max_limit = np.array([1.7016, 1.047, 3.0541, 2.618, 3.059, 2.094, 3.059])
    joint_pos_min_limit = np.array([-1.7016, -2.147, -3.0541, -0.05, -3.059, -1.5707, -3.059])
    joint_vel_max_limit = np.array([2.0, 2.0, 2.0, 2.0, 4.0, 4.0, 4.0])
    joint_vel_min_limit = -1 * joint_vel_max_limit


    node = 70
    pos_dis = (joint_pos_max_limit - joint_pos_min_limit) / node
    vel_dis = (joint_vel_max_limit - joint_vel_min_limit) / node
    alpha_dis = vel_dis
    dot_alpha_dis = alpha_dis * 10 #大概估计的值，正确的应该使用不带alpha参数的神经网络

    mu = np.zeros((node, 28))  # 各个神经元的中心向量
    mu[0] = np.r_[joint_pos_min_limit, joint_vel_min_limit, joint_vel_min_limit, 10*joint_vel_min_limit]
    for i in range(1, (node-1)):
        mu[i] = mu[i-1] + np.r_[pos_dis, vel_dis, alpha_dis, dot_alpha_dis]
    mu[-1] = np.r_[joint_pos_max_limit, joint_vel_max_limit, joint_vel_max_limit, 10 * joint_vel_max_limit]


    itea = 1.2 # 方差
    sigma = 0.25 #小常数

    s = np.zeros(node)
    x = np.zeros(4*7) #Z向量
    w = np.zeros((node, 7)) #神经元的权值
    d_w = np.zeros((node, 7)) #神经元权值的更新
    for i in range(node):
        for j in range(7):
            w[i, j] = random.random()
    # tmp_gamma = np.ones(node)
    # gamma = np.diag(tmp_gamma)
    gamma = 1 # 先所有权值用一个更新率


    while (input[35]):

        # print "!!!!!!!!!!!!!", input[35]

        test_start_time = time.time()
        if not rospy.is_shutdown():
            # print list(input)
            x = np.array(input[0:28])
            # print x
            "gaussian"

            for i in range(0, node):
                s[i] = math.exp(-np.linalg.norm(x - mu[i]) / itea)


            tmp_output = np.dot(w.T, s)
            for i in range(7):
                output[i] = tmp_output[i]
            for i in range(0, node):
                d_w[i] = -gamma * (s[i] * np.array(input[28:35]) + sigma * w[i])
            w = w + d_w

            time.sleep(0.001)

        # print time.time() - test_start_time
    # print "???????????????????/"

def right_NN(input, output):


    joint_pos_max_limit = np.array([1.7016, 1.047, 3.0541, 2.618, 3.059, 2.094, 3.059])
    joint_pos_min_limit = np.array([-1.7016, -2.147, -3.0541, -0.05, -3.059, -1.5707, -3.059])
    joint_vel_max_limit = np.array([2.0, 2.0, 2.0, 2.0, 4.0, 4.0, 4.0])
    joint_vel_min_limit = -1 * joint_vel_max_limit


    node = 120
    pos_dis = (joint_pos_max_limit - joint_pos_min_limit) / node
    vel_dis = (joint_vel_max_limit - joint_vel_min_limit) / node
    alpha_dis = vel_dis
    dot_alpha_dis = alpha_dis * 10 #大概估计的值，正确的应该使用不带alpha参数的神经网络

    mu = np.zeros((node, 28))  # 各个神经元的中心向量
    mu[0] = np.r_[joint_pos_min_limit, joint_vel_min_limit, joint_vel_min_limit, 10*joint_vel_min_limit]
    for i in range(1, (node-1)):
        mu[i] = mu[i-1] + np.r_[pos_dis, vel_dis, alpha_dis, dot_alpha_dis]
    mu[-1] = np.r_[joint_pos_max_limit, joint_vel_max_limit, joint_vel_max_limit, 10 * joint_vel_max_limit]


    itea = 1.8 # 方差
    sigma = 0.25 #小常数

    s = np.zeros(node)
    x = np.zeros(4*7) #Z向量
    w = np.zeros((node, 7)) #神经元的权值
    d_w = np.zeros((node, 7)) #神经元权值的更新
    for i in range(node):
        for j in range(7):
            w[i, j] = random.random()
    # tmp_gamma = np.ones(node)
    # gamma = np.diag(tmp_gamma)
    gamma = 1 # 先所有权值用一个更新率


    while (input[35]):

        # print "!!!!!!!!!!!!!", input[35]

        test_start_time = time.time()
        if not rospy.is_shutdown():
            # print list(input)
            x = np.array(input[0:28])
            # print x
            "gaussian"

            for i in range(0, node):
                s[i] = math.exp(-np.linalg.norm(x - mu[i]) / itea)


            tmp_output = np.dot(w.T, s)
            for i in range(7):
                output[i] = tmp_output[i]
            for i in range(0, node):
                d_w[i] = -gamma * (s[i] * np.array(input[28:35]) + sigma * w[i])
            w = w + d_w

            time.sleep(0.001)

        # print time.time() - test_start_time
    

class NNController(object):
    def __init__(self):
        self.rs = baxter_interface.RobotEnable()
        self.rs.enable()
        self.left_limb = baxter_interface.Limb('left')
        self.right_limb = baxter_interface.Limb('right')
        # self.left_limb.move_to_neutral()
        # self.right_limb.move_to_neutral()

        goal_q_pos = np.loadtxt("/home/lijiashushu/minimum_jerk_pos.txt")
        goal_q_vel = np.loadtxt("/home/lijiashushu/minimum_jerk_vel.txt")
        goal_q_acc = np.loadtxt("/home/lijiashushu/minimum_jerk_acc.txt")

        print goal_q_pos.shape
        self.left_goal_q_pos = goal_q_pos[:, 0:7]
        self.left_goal_q_vel = goal_q_vel[:, 0:7]
        self.left_goal_q_acc = goal_q_acc[:, 0:7]
        self.right_goal_q_pos = goal_q_pos[:, 7:]
        self.right_goal_q_vel = goal_q_vel[:, 7:]
        self.right_goal_q_acc = goal_q_acc[:, 7:]

        self.left_joint_names = self.left_limb.joint_names()
        self.right_joint_names = self.right_limb.joint_names()


        self.left_K1 = np.array([25, 30, 25.0, 25.3, 20.7, 20.0, 15.7])
        self.left_K2 = np.array([4.5, 4.5, 3, 3.1, 2.6, 2.2, 2.2])


        # '''基础'''
        # self.right_K1 = np.array([20, 30, 30.0, 20.3, 25.7, 25.0, 10.7])
        # self.right_K2 = np.array([4.5, 4.5, 2.5, 2.1, 2.5, 2.2, 1.2])

        self.right_K1 = np.array([20, 30, 20.0, 20.3, 25.7, 20.0, 10.7])
        self.right_K2 = np.array([4.5, 4.5, 3.5, 2.1, 2.8, 2.2, 1.2])

        self.left_init_angles = self.left_goal_q_pos[0, :]
        self.left_limb.move_to_joint_positions(dict(zip(self.left_joint_names, self.left_init_angles)))
        self.right_init_angles = self.right_goal_q_pos[0, :]
        self.right_limb.move_to_joint_positions(dict(zip(self.right_joint_names, self.right_init_angles)))

        self.left_now_q_pos = np.zeros(7)
        self.left_now_q_vel = np.zeros(7)
        self.right_now_q_pos = np.zeros(7)
        self.right_now_q_vel = np.zeros(7)

        self.left_pos_error = np.zeros(7)
        self.left_vel_error = np.zeros(7)
        self.right_pos_error = np.zeros(7)
        self.right_vel_error = np.zeros(7)

        self.left_z1 = np.zeros(7)
        self.left_alpha = np.zeros(7)
        self.left_z2 = np.zeros(7)
        self.left_dot_alpha = np.zeros(7)
        self.left_tau = np.zeros(7)

        self.right_z1 = np.zeros(7)
        self.right_alpha = np.zeros(7)
        self.right_z2 = np.zeros(7)
        self.right_dot_alpha = np.zeros(7)
        self.right_tau = np.zeros(7)

        '''画图用'''
        self.size1 = 17
        self.size2 = 12
        self.linesize = 1.5
        # legend 字体
        # self.legend_size = 1000
        self.font1 = {'family': 'Times New Roman',
                      'weight': 'normal',
                      'size': self.size1,
                      }
        # 坐标轴刻度和坐标轴名称
        self.font2 = {'family': 'Times New Roman',
                      'weight': 'normal',
                      'size': self.size2,
                      }

        self.draw_time_step = 0.5

        self.left_draw_now_pos = list()
        self.left_draw_now_vel = list()
        self.left_draw_goal_pos = list()
        self.left_draw_goal_vel = list()
        self.left_draw_err_pos = list()
        self.left_draw_err_vel = list()
        self.left_draw_time = list()
        self.left_draw_tau = list()

        self.left_Cartesian_goal_draw_x = list()
        self.left_Cartesian_goal_draw_y = list()
        self.left_Cartesian_goal_draw_z = list()
        self.left_Cartesian_now_draw_x = list()
        self.left_Cartesian_now_draw_y = list()
        self.left_Cartesian_now_draw_z = list()

        self.right_draw_now_pos = list()
        self.right_draw_now_vel = list()
        self.right_draw_goal_pos = list()
        self.right_draw_goal_vel = list()
        self.right_draw_err_pos = list()
        self.right_draw_err_vel = list()
        self.right_draw_time = list()
        self.right_draw_tau = list()

        self.right_Cartesian_goal_draw_x = list()
        self.right_Cartesian_goal_draw_y = list()
        self.right_Cartesian_goal_draw_z = list()
        self.right_Cartesian_now_draw_x = list()
        self.right_Cartesian_now_draw_y = list()
        self.right_Cartesian_now_draw_z = list()

        self.left_jacobian_sigularity_value = list()

        self.control_Rate_value = 100
        self.control_Rate = rospy.Rate(self.control_Rate_value)

        ''' Baxter Kinematics with PyKDL '''
        self._baxter = URDF.from_parameter_server(key='robot_description')
        # self._baxter = URDF.from_xml_file('/home/lijiashushu/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf.xacro')  # get a tree
        self.kdl_tree = kdl_tree_from_urdf_model(self._baxter)
        self.left_base_link = 'base'
        self.left_tip_link = 'left_gripper'
        self.left_arm_chain = self.kdl_tree.getChain(self.left_base_link, self.left_tip_link)
        self.left_fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self.left_arm_chain)
        self.left_jac_kdl = PyKDL.ChainJntToJacSolver(self.left_arm_chain)
        self.left_tip_goal_frame = PyKDL.Frame()
        self.left_tip_now_frame = PyKDL.Frame()
        self.left_joint_kdl_goal_array = PyKDL.JntArray(7)
        self.left_joint_kdl_now_array = PyKDL.JntArray(7)
        self.left_baxter_kdl = baxter_pykdl.baxter_kinematics('left')

        self.right_base_link = 'base'
        self.right_tip_link = 'right_gripper'
        self.right_arm_chain = self.kdl_tree.getChain(self.right_base_link, self.right_tip_link)
        self.right_fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self.right_arm_chain)
        self.right_jac_kdl = PyKDL.ChainJntToJacSolver(self.right_arm_chain)
        self.right_tip_goal_frame = PyKDL.Frame()
        self.right_tip_now_frame = PyKDL.Frame()
        self.right_joint_kdl_goal_array = PyKDL.JntArray(7)
        self.right_joint_kdl_now_array = PyKDL.JntArray(7)
        self.right_baxter_kdl = baxter_pykdl.baxter_kinematics('right')

        '''进程通信'''
        self.left_nn_input = mp.Array('f',
                                 [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, #q
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #dot_q
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #alpha
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #dot_alpha
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #z2
                                  1.0])#标志
        self.left_nn_output = mp.Array('f', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.left_nn_process = Process(target=left_NN, args=(self.left_nn_input, self.left_nn_output))
        self.left_nn_process.start()

        self.right_nn_input = mp.Array('f',
                                      [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # q
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # dot_q
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # alpha
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # dot_alpha
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # z2
                                       1.0])  # 标志
        self.right_nn_output = mp.Array('f', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.right_nn_process = Process(target=right_NN, args=(self.right_nn_input, self.right_nn_output))
        self.right_nn_process.start()

    def control_loop(self):
        goal_num = -1
        draw_num = -1
        loop_count = 0
        goal_length = len(self.left_goal_q_pos)
        left_force_sensor_weight = 0.2 * 9.8
        right_force_sensor_weight = 0.8 * 9.8
        left_compensate = np.array([0, 0, left_force_sensor_weight, 0, 0, 0])
        right_compensate = np.array([0, 0, right_force_sensor_weight, 0, 0, 0])
        while goal_num < goal_length - 1:


            goal_num = goal_num + 1
            '''左臂'''
            '''更新当前关节角度'''
            left_temp_pos = self.left_limb.joint_angles()
            left_temp_vel = self.left_limb.joint_velocities()
            count = 0
            for i in self.left_joint_names:
                self.left_now_q_pos[count] = left_temp_pos[i]
                self.left_now_q_vel[count] = left_temp_vel[i]
                count = count + 1
            '''计算误差'''
            self.left_z1 = self.left_now_q_pos - self.left_goal_q_pos[goal_num]
            self.left_alpha = -self.left_K1 * self.left_z1 + self.left_goal_q_vel[goal_num]
            self.left_z2 = self.left_now_q_vel - self.left_alpha
            self.left_dot_alpha = -self.left_K1*(self.left_z2-self.left_K1*self.left_z1)+self.left_goal_q_acc[goal_num]

            '''计算力矩'''
            left_torque_compensate_tmp = self.left_baxter_kdl.jacobian_transpose() * np.mat(left_compensate).T
            left_torque_compensate = left_torque_compensate_tmp.T.getA()[0]
            # print "np.array(self.nn_output)", np.array(self.left_nn_output)
            self.left_tau = -self.left_z1 - self.left_K2 * self.left_z2 + np.array(self.left_nn_output) + left_torque_compensate
            self.tmp_nn_input = np.r_[self.left_now_q_pos, self.left_now_q_vel, self.left_alpha, self.left_dot_alpha, self.left_z2, 1.0]
            for i in range(len(self.tmp_nn_input)):
                self.left_nn_input[i] = self.tmp_nn_input[i]
            print "self.left_tau", self.left_tau
            print "self.right_tau", self.right_tau


            '''右臂'''
            '''更新当前关节角度'''
            right_temp_pos = self.right_limb.joint_angles()
            right_temp_vel = self.right_limb.joint_velocities()
            count = 0
            for i in self.right_joint_names:
                self.right_now_q_pos[count] = right_temp_pos[i]
                self.right_now_q_vel[count] = right_temp_vel[i]
                count = count + 1
            '''计算误差'''
            self.right_z1 = self.right_now_q_pos - self.right_goal_q_pos[goal_num]
            self.right_alpha = -self.right_K1 * self.right_z1 + self.right_goal_q_vel[goal_num]
            self.right_z2 = self.right_now_q_vel - self.right_alpha
            self.right_dot_alpha = -self.right_K1 * (self.right_z2 - self.right_K1 * self.right_z1) + self.right_goal_q_acc[goal_num]

            '''计算力矩'''
            right_torque_compensate_tmp = self.right_baxter_kdl.jacobian_transpose() * np.mat(right_compensate).T
            right_torque_compensate = right_torque_compensate_tmp.T.getA()[0]
            print 'torque_compensate', right_torque_compensate
            # print "np.array(self.nn_output)", np.array(self.right_nn_output)
            self.right_tau = -self.right_z1 - self.right_K2 * self.right_z2 + np.array(self.right_nn_output) + right_torque_compensate
            self.tmp_nn_input = np.r_[
                self.right_now_q_pos, self.right_now_q_vel, self.right_alpha, self.right_dot_alpha, self.right_z2, 1.0]
            for i in range(len(self.tmp_nn_input)):
                self.right_nn_input[i] = self.tmp_nn_input[i]

            # print "endpoint_velocity"
            # print self.left_limb.endpoint_velocity()


            if loop_count % (self.draw_time_step * self.control_Rate_value) == 0:
                draw_num = draw_num + 1

                '''左臂'''
                for i in range(7):
                    self.left_joint_kdl_goal_array[i] = self.left_goal_q_pos[goal_num, i]
                    self.left_joint_kdl_now_array[i] = self.left_now_q_pos[i]
                self.left_fk_p_kdl.JntToCart(self.left_joint_kdl_goal_array, self.left_tip_goal_frame)
                self.left_fk_p_kdl.JntToCart(self.left_joint_kdl_now_array, self.left_tip_now_frame)
                self.left_Cartesian_goal_draw_x.append(self.left_tip_goal_frame.p[0])
                self.left_Cartesian_goal_draw_y.append(self.left_tip_goal_frame.p[1])
                self.left_Cartesian_goal_draw_z.append(self.left_tip_goal_frame.p[2])
                self.left_Cartesian_now_draw_x.append(self.left_tip_now_frame.p[0])
                self.left_Cartesian_now_draw_y.append(self.left_tip_now_frame.p[1])
                self.left_Cartesian_now_draw_z.append(self.left_tip_now_frame.p[2])

                self.left_draw_now_pos.append(self.left_now_q_pos.copy())
                self.left_draw_now_vel.append(self.left_now_q_vel.copy())
                self.left_draw_goal_pos.append(self.left_goal_q_pos[goal_num].copy())
                self.left_draw_goal_vel.append(self.left_goal_q_vel[goal_num].copy())
                self.left_draw_err_pos.append((self.left_now_q_pos - self.left_goal_q_pos[goal_num]).copy())
                self.left_draw_err_vel.append((self.left_now_q_vel - self.left_goal_q_vel[goal_num]).copy())
                self.left_draw_tau.append(self.left_tau.copy())

                if draw_num == 0:
                    start_time = rospy.get_time()
                    self.left_draw_time.append(0)
                else:
                    self.left_draw_time.append(rospy.get_time() - start_time)

                '''右臂'''
                for i in range(7):
                    self.right_joint_kdl_goal_array[i] = self.right_goal_q_pos[goal_num, i]
                    self.right_joint_kdl_now_array[i] = self.right_now_q_pos[i]
                self.right_fk_p_kdl.JntToCart(self.right_joint_kdl_goal_array, self.right_tip_goal_frame)
                self.right_fk_p_kdl.JntToCart(self.right_joint_kdl_now_array, self.right_tip_now_frame)
                self.right_Cartesian_goal_draw_x.append(self.right_tip_goal_frame.p[0])
                self.right_Cartesian_goal_draw_y.append(self.right_tip_goal_frame.p[1])
                self.right_Cartesian_goal_draw_z.append(self.right_tip_goal_frame.p[2])
                self.right_Cartesian_now_draw_x.append(self.right_tip_now_frame.p[0])
                self.right_Cartesian_now_draw_y.append(self.right_tip_now_frame.p[1])
                self.right_Cartesian_now_draw_z.append(self.right_tip_now_frame.p[2])

                self.right_draw_now_pos.append(self.right_now_q_pos.copy())
                self.right_draw_now_vel.append(self.right_now_q_vel.copy())
                self.right_draw_goal_pos.append(self.right_goal_q_pos[goal_num].copy())
                self.right_draw_goal_vel.append(self.right_goal_q_vel[goal_num].copy())
                self.right_draw_err_pos.append((self.right_now_q_pos - self.right_goal_q_pos[goal_num]).copy())
                self.right_draw_err_vel.append((self.right_now_q_vel - self.right_goal_q_vel[goal_num]).copy())
                self.right_draw_tau.append(self.right_tau.copy())
                if draw_num == 0:
                    start_time = rospy.get_time()
                    self.right_draw_time.append(0)
                else:
                    self.right_draw_time.append(rospy.get_time() - start_time)

            self.left_limb.set_joint_torques(dict(zip(self.left_joint_names, self.left_tau.tolist())))
            self.right_limb.set_joint_torques(dict(zip(self.right_joint_names, self.right_tau.tolist())))

            loop_count = loop_count + 1
            self.control_Rate.sleep()

        '''保存数据'''
        # np.savetxt("data/left_draw_time.txt", self.left_draw_time)
        # np.savetxt("data/left_Cartesian_goal_draw_x.txt", self.left_Cartesian_goal_draw_x)
        # np.savetxt("data/left_Cartesian_goal_draw_y.txt", self.left_Cartesian_goal_draw_y)
        # np.savetxt("data/left_Cartesian_goal_draw_z.txt", self.left_Cartesian_goal_draw_z)
        # np.savetxt("data/left_Cartesian_now_draw_x.txt", self.left_Cartesian_now_draw_x)
        # np.savetxt("data/left_Cartesian_now_draw_y.txt", self.left_Cartesian_now_draw_y)
        # np.savetxt("data/left_Cartesian_now_draw_z.txt", self.left_Cartesian_now_draw_z)
        # np.savetxt("data/left_draw_now_pos.txt", self.left_draw_now_pos)
        # np.savetxt("data/left_draw_now_vel.txt", self.left_draw_now_vel)
        # np.savetxt("data/left_draw_goal_pos.txt", self.left_draw_goal_pos)
        # np.savetxt("data/left_draw_goal_vel.txt", self.left_draw_goal_vel)
        # np.savetxt("data/left_draw_err_pos.txt", self.left_draw_err_pos)
        # np.savetxt("data/left_draw_err_vel.txt", self.left_draw_err_vel)
        # np.savetxt("data/left_draw_tau.txt", self.left_draw_tau)
        #
        # np.savetxt("data/right_draw_time.txt", self.right_draw_time)
        # np.savetxt("data/right_Cartesian_goal_draw_x.txt", self.right_Cartesian_goal_draw_x)
        # np.savetxt("data/right_Cartesian_goal_draw_y.txt", self.right_Cartesian_goal_draw_y)
        # np.savetxt("data/right_Cartesian_goal_draw_z.txt", self.right_Cartesian_goal_draw_z)
        # np.savetxt("data/right_Cartesian_now_draw_x.txt", self.right_Cartesian_now_draw_x)
        # np.savetxt("data/right_Cartesian_now_draw_y.txt", self.right_Cartesian_now_draw_y)
        # np.savetxt("data/right_Cartesian_now_draw_z.txt", self.right_Cartesian_now_draw_z)
        # np.savetxt("data/right_draw_now_pos.txt", self.right_draw_now_pos)
        # np.savetxt("data/right_draw_now_vel.txt", self.right_draw_now_vel)
        # np.savetxt("data/right_draw_goal_pos.txt", self.right_draw_goal_pos)
        # np.savetxt("data/right_draw_goal_vel.txt", self.right_draw_goal_vel)
        # np.savetxt("data/right_draw_err_pos.txt", self.right_draw_err_pos)
        # np.savetxt("data/right_draw_err_vel.txt", self.right_draw_err_vel)
        # np.savetxt("data/right_draw_tau.txt", self.right_draw_tau)


        self.left_limb.exit_control_mode()
        self.right_limb.exit_control_mode()
        for i in range(len(self.tmp_nn_input)):
            self.left_nn_input[i] = 0
            self.right_nn_input[i] = 0
        
        
        "右臂"
        fig0 = plt.figure(0)
        ax = fig0.add_subplot(111)
        plt.title("Right Arm Torques", self.font1)
        ax.tick_params(labelsize=self.size2)
        plt.plot(self.right_draw_time, np.array(self.right_draw_tau)[:, 0], '-*', linewidth=self.linesize,
                 label='Joint S0')
        plt.plot(self.right_draw_time, np.array(self.right_draw_tau)[:, 1], '-o', linewidth=self.linesize,
                 label='Joint S1')
        plt.plot(self.right_draw_time, np.array(self.right_draw_tau)[:, 2], '-v', linewidth=self.linesize,
                 label='Joint E0')
        plt.plot(self.right_draw_time, np.array(self.right_draw_tau)[:, 3], '-s', linewidth=self.linesize,
                 label='Joint E1')
        plt.plot(self.right_draw_time, np.array(self.right_draw_tau)[:, 4], '-p', linewidth=self.linesize,
                 label='Joint W0')
        plt.plot(self.right_draw_time, np.array(self.right_draw_tau)[:, 5], '-h', linewidth=self.linesize,
                 label='Joint W1')
        plt.plot(self.right_draw_time, np.array(self.right_draw_tau)[:, 6], '-d', linewidth=self.linesize,
                 label='Joint W2')
        plt.xlabel("Time/s", self.font2)
        plt.ylabel("Torque/$\mathrm{N\cdot m}$", self.font2)
        ax.grid()
        ax.set_xlim(0.0, self.right_draw_time[-1])
        # plt.ylim(-2.5, 2.5)
        # plt.legend(bbox_to_anchor=(1.105, 0.6), fontsize=self.legend_size)
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.26), ncol=4, prop=self.font2)
        fig0.tight_layout()
        plt.subplots_adjust(left=0.10, right=0.97, wspace=0.20, hspace=0.20, bottom=0.21, top=0.94)
        plt.savefig('right/right_torque.eps', format='eps')

        '''位置跟踪误差FJ'''
        fig1 = plt.figure(1)
        ax = fig1.add_subplot(111)
        plt.title("Right Joint $\mathrm{S_0}$", self.font1)
        lns1 = ax.plot(self.right_draw_time, np.array(self.right_draw_goal_pos)[:, 0], '--', linewidth=self.linesize,
                       color="green", label="Desired trajectory")
        lns2 = ax.plot(self.right_draw_time, np.array(self.right_draw_now_pos)[:, 0], '-', linewidth=self.linesize,
                       color='red', label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.right_draw_time, np.array(self.right_draw_err_pos)[:, 0], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.right_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig1.tight_layout()
        plt.savefig('right/right_joint_S0.eps', format='eps')

        fig2 = plt.figure(2)
        ax = fig2.add_subplot(111)
        plt.title("Right Joint $\mathrm{S_1}$", self.font1)
        lns1 = ax.plot(self.right_draw_time, np.array(self.right_draw_goal_pos)[:, 1], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.right_draw_time, np.array(self.right_draw_now_pos)[:, 1], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.right_draw_time, np.array(self.right_draw_err_pos)[:, 1], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.right_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig2.tight_layout()
        plt.savefig('right/right_joint_S1.eps', format='eps')

        fig3 = plt.figure(3)
        ax = fig3.add_subplot(111)
        plt.title("Right Joint $\mathrm{E_0}$", self.font1)
        lns1 = ax.plot(self.right_draw_time, np.array(self.right_draw_goal_pos)[:, 2], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.right_draw_time, np.array(self.right_draw_now_pos)[:, 2], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.right_draw_time, np.array(self.right_draw_err_pos)[:, 2], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.right_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig3.tight_layout()
        plt.savefig('right/right_joint_E0.eps', format='eps')

        fig4 = plt.figure(4)
        ax = fig4.add_subplot(111)
        plt.title("Right Joint $\mathrm{E_1}$", self.font1)
        lns1 = ax.plot(self.right_draw_time, np.array(self.right_draw_goal_pos)[:, 3], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.right_draw_time, np.array(self.right_draw_now_pos)[:, 3], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.right_draw_time, np.array(self.right_draw_err_pos)[:, 3], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.right_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig4.tight_layout()
        plt.savefig('right/right_joint_E1.eps', format='eps')

        fig5 = plt.figure(5)
        ax = fig5.add_subplot(111)
        plt.title("Right Joint $\mathrm{W_0}$", self.font1)
        lns1 = ax.plot(self.right_draw_time, np.array(self.right_draw_goal_pos)[:, 4], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.right_draw_time, np.array(self.right_draw_now_pos)[:, 4], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.right_draw_time, np.array(self.right_draw_err_pos)[:, 4], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.right_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig5.tight_layout()
        plt.savefig('right/right_joint_W0.eps', format='eps')

        fig6 = plt.figure(6)
        ax = fig6.add_subplot(111)
        plt.title("Right Joint $\mathrm{W_1}$", self.font1)
        lns1 = ax.plot(self.right_draw_time, np.array(self.right_draw_goal_pos)[:, 5], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.right_draw_time, np.array(self.right_draw_now_pos)[:, 5], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.right_draw_time, np.array(self.right_draw_err_pos)[:, 5], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.right_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig6.tight_layout()
        plt.savefig('right/right_joint_W1.eps', format='eps')

        fig7 = plt.figure(7)
        ax = fig7.add_subplot(111)
        plt.title("Right Joint $\mathrm{W_2}$", self.font1)
        lns1 = ax.plot(self.right_draw_time, np.array(self.right_draw_goal_pos)[:, 6], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.right_draw_time, np.array(self.right_draw_now_pos)[:, 6], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.right_draw_time, np.array(self.right_draw_err_pos)[:, 6], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.right_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig6.tight_layout()
        plt.savefig('right/right_joint_W2.eps', format='eps')

        '''左臂'''
        fig10 = plt.figure(10)
        ax = fig10.add_subplot(111)
        plt.title("Left Arm Torques", self.font1)
        ax.tick_params(labelsize=self.size2)
        plt.plot(self.left_draw_time, np.array(self.left_draw_tau)[:, 0], '-*', linewidth=self.linesize,
                 label='Joint S0')
        plt.plot(self.left_draw_time, np.array(self.left_draw_tau)[:, 1], '-o', linewidth=self.linesize,
                 label='Joint S1')
        plt.plot(self.left_draw_time, np.array(self.left_draw_tau)[:, 2], '-v', linewidth=self.linesize,
                 label='Joint E0')
        plt.plot(self.left_draw_time, np.array(self.left_draw_tau)[:, 3], '-s', linewidth=self.linesize,
                 label='Joint E1')
        plt.plot(self.left_draw_time, np.array(self.left_draw_tau)[:, 4], '-p', linewidth=self.linesize,
                 label='Joint W0')
        plt.plot(self.left_draw_time, np.array(self.left_draw_tau)[:, 5], '-h', linewidth=self.linesize,
                 label='Joint W1')
        plt.plot(self.left_draw_time, np.array(self.left_draw_tau)[:, 6], '-d', linewidth=self.linesize,
                 label='Joint W2')
        plt.xlabel("Time/s", self.font2)
        plt.ylabel("Torque/$\mathrm{N\cdot m}$", self.font2)
        ax.grid()
        ax.set_xlim(0.0, self.left_draw_time[-1])
        # plt.ylim(-2.5, 2.5)
        # plt.legend(bbox_to_anchor=(1.105, 0.6), fontsize=self.legend_size)
        plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.26), ncol=4, prop=self.font2)
        fig10.tight_layout()
        plt.subplots_adjust(left=0.10, right=0.97, wspace=0.20, hspace=0.20, bottom=0.21, top=0.94)
        plt.savefig('left/left_torque.eps', format='eps')

        '''位置跟踪误差FJ'''
        fig11 = plt.figure(11)
        ax = fig11.add_subplot(111)
        plt.title("Left Joint $\mathrm{S_0}$", self.font1)
        lns1 = ax.plot(self.left_draw_time, np.array(self.left_draw_goal_pos)[:, 0], '--', linewidth=self.linesize,
                       color="green", label="Desired trajectory")
        lns2 = ax.plot(self.left_draw_time, np.array(self.left_draw_now_pos)[:, 0], '-', linewidth=self.linesize,
                       color='red', label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.left_draw_time, np.array(self.left_draw_err_pos)[:, 0], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.left_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig11.tight_layout()
        plt.savefig('left/left_joint_S0.eps', format='eps')

        fig12 = plt.figure(12)
        ax = fig12.add_subplot(111)
        plt.title("Left Joint $\mathrm{S_1}$", self.font1)
        lns1 = ax.plot(self.left_draw_time, np.array(self.left_draw_goal_pos)[:, 1], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.left_draw_time, np.array(self.left_draw_now_pos)[:, 1], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.left_draw_time, np.array(self.left_draw_err_pos)[:, 1], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.left_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig12.tight_layout()
        plt.savefig('left/left_joint_S1.eps', format='eps')

        fig13 = plt.figure(13)
        ax = fig13.add_subplot(111)
        plt.title("Left Joint $\mathrm{E_0}$", self.font1)
        lns1 = ax.plot(self.left_draw_time, np.array(self.left_draw_goal_pos)[:, 2], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.left_draw_time, np.array(self.left_draw_now_pos)[:, 2], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.left_draw_time, np.array(self.left_draw_err_pos)[:, 2], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.left_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig13.tight_layout()
        plt.savefig('left/left_joint_E0.eps', format='eps')

        fig14 = plt.figure(14)
        ax = fig14.add_subplot(111)
        plt.title("Left Joint $\mathrm{E_1}$", self.font1)
        lns1 = ax.plot(self.left_draw_time, np.array(self.left_draw_goal_pos)[:, 3], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.left_draw_time, np.array(self.left_draw_now_pos)[:, 3], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.left_draw_time, np.array(self.left_draw_err_pos)[:, 3], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.left_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig14.tight_layout()
        plt.savefig('left/left_joint_E1.eps', format='eps')

        fig15 = plt.figure(15)
        ax = fig15.add_subplot(111)
        plt.title("Left Joint $\mathrm{W_0}$", self.font1)
        lns1 = ax.plot(self.left_draw_time, np.array(self.left_draw_goal_pos)[:, 4], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.left_draw_time, np.array(self.left_draw_now_pos)[:, 4], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.left_draw_time, np.array(self.left_draw_err_pos)[:, 4], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.left_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig15.tight_layout()
        plt.savefig('left/left_joint_W0.eps', format='eps')

        fig16 = plt.figure(16)
        ax = fig16.add_subplot(111)
        plt.title("Left Joint $\mathrm{W_1}$", self.font1)
        lns1 = ax.plot(self.left_draw_time, np.array(self.left_draw_goal_pos)[:, 5], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.left_draw_time, np.array(self.left_draw_now_pos)[:, 5], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.left_draw_time, np.array(self.left_draw_err_pos)[:, 5], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.left_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig16.tight_layout()
        plt.savefig('left/left_joint_W1.eps', format='eps')

        fig17 = plt.figure(17)
        ax = fig17.add_subplot(111)
        plt.title("Left Joint $\mathrm{W_2}$", self.font1)
        lns1 = ax.plot(self.left_draw_time, np.array(self.left_draw_goal_pos)[:, 6], '--', linewidth=self.linesize,
                       color="green",
                       label="Desired trajectory")
        lns2 = ax.plot(self.left_draw_time, np.array(self.left_draw_now_pos)[:, 6], '-', linewidth=self.linesize,
                       color='red',
                       label="Actual trajectory")
        ax2 = ax.twinx()
        lns5 = ax2.plot(self.left_draw_time, np.array(self.left_draw_err_pos)[:, 6], '-*', linewidth=self.linesize,
                        color=(0.0, 0.5, 1.0),
                        label="Tracking error")
        lns = lns1 + lns2 + lns5
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.grid()
        ax.set_xlabel("Time/s", self.font2)
        ax.set_ylabel("Angle/rad", self.font2)
        ax2.set_ylabel("Angle error/rad", self.font2)
        ax.set_xlim(0.0, self.left_draw_time[-1])
        ax.tick_params(labelsize=self.size2)
        ax2.tick_params(labelsize=self.size2)
        fig16.tight_layout()
        plt.savefig('left/left_joint_W2.eps', format='eps')
        
        
        '''空间'''
        fig9 = plt.figure(9)
        ax = fig9.add_subplot(111, projection='3d')
        ax.grid()
        lns1 = ax.plot(self.left_Cartesian_goal_draw_x, self.left_Cartesian_goal_draw_y,
                       self.left_Cartesian_goal_draw_z, '--', label="Left desired trajectory", color='peru',
                       linewidth=self.linesize)
        lns2 = ax.plot(self.left_Cartesian_now_draw_x, self.left_Cartesian_now_draw_y,
                       self.left_Cartesian_now_draw_z, label="Left actual trajectory", color='blue',
                       linewidth=self.linesize)
        plt.title("End Effector Position", self.font1)
        lns3 = ax.plot(self.right_Cartesian_goal_draw_x, self.right_Cartesian_goal_draw_y,
                       self.right_Cartesian_goal_draw_z, '--', label="Right desired trajectory", color='green',
                       linewidth=self.linesize)
        lns4 = ax.plot(self.right_Cartesian_now_draw_x, self.right_Cartesian_now_draw_y,
                       self.right_Cartesian_now_draw_z, label="Right actual trajectory", color='red',
                       linewidth=self.linesize)

        lns = lns1 + lns2 + lns3 + lns4
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='best', prop=self.font2)
        ax.set_xlabel("X/m", self.font2)
        ax.set_ylabel("Y/m", self.font2)
        ax.set_zlabel("Z/m", self.font2)
        # plt.ylim(0, 1)
        # ax.set_zlim(0, 0.5)
        # ax.set_ylim(-0.5, 0)
        # ax.set_xlim(0.2, 0.7)
        fig9.tight_layout()
        plt.savefig('right/end_effector_position.eps', format='eps')
        # plt.show()

    def fastest_rate_test(self):
        '''更新当前关节角度'''
        left_temp_pos = self.left_limb.joint_angles()
        left_temp_vel = self.left_limb.joint_velocities()
        count = 0
        for i in self.left_joint_names:
            self.left_now_q_pos[count] = left_temp_pos[i]
            self.left_now_q_vel[count] = left_temp_vel[i]
            count = count + 1

        last_angle = self.left_now_q_pos.copy()
        last_velocity = self.left_now_q_vel.copy()
        while True:

            left_temp_pos = self.left_limb.joint_angles()
            left_temp_vel = self.left_limb.joint_velocities()
            count = 0
            for i in self.left_joint_names:
                self.left_now_q_pos[count] = left_temp_pos[i]
                self.left_now_q_vel[count] = left_temp_vel[i]
                count = count + 1

            print "angle!!!"
            print self.left_now_q_pos - last_angle
            print "velocity!!!!"
            print self.left_now_q_vel - last_velocity

            last_angle = self.left_now_q_pos.copy()
            last_velocity = self.left_now_q_vel.copy()

            self.control_Rate.sleep()




def main():
    # robot = URDF.from_xml_file("/home/lijiashushu/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")
    rospy.init_node("draw_traj")
    robot = URDF.from_parameter_server("robot_description")
    tree = kdl_tree_from_urdf_model(robot)
    leftChain = tree.getChain("base", "left_gripper")
    rightChain = tree.getChain("base", "right_gripper")

    time = np.loadtxt("/home/lijiashushu/traj_time.txt")
    position = np.loadtxt("/home/lijiashushu/traj_angle.txt")
    velocity = np.loadtxt("/home/lijiashushu/traj_velocity.txt")
    acceleration = np.loadtxt("/home/lijiashushu/traj_acceleration.txt")

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

    timeDraw = []
    leftRotErrDraw = []
    rightPosErrDraw = []
    rightRotErrDraw = []

    leftXDraw = []
    leftYDraw = []
    leftZDraw = []

    for i in range(position.shape[0]):
        for j in range(7):
            leftJointArr[j] = position[i, j]
            rightJointArr[j] = position[i, j+7]

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

        distance = PyKDL.Vector(0, 0, 0.2)
        rightGoalPos = leftGoalRot * distance + leftPos
        rightPosErr = rightPos - rightGoalPos
        rightGoalEulerVec = np.array([leftEulerVec[0], 0, -1.57])

        rightEulerVec = np.zeros(3)
        rightEulerVec[0], rightEulerVec[1], rightEulerVec[2] = rightRot.GetEulerZYX()
        rightEulerErr = rightEulerVec - rightGoalEulerVec

        timeDraw.append(time[i])
        leftRotErrDraw.append(np.linalg.norm(leftEulerErr))
        rightPosErrDraw.append(rightPosErr.Norm())
        rightRotErrDraw.append(np.linalg.norm(rightEulerErr))
        leftXDraw.append(leftPos[0])
        leftYDraw.append(leftPos[1])
        leftZDraw.append(leftPos[2])

    size1 = 17
    size2 = 12
    linesize = 2
    fig0 = plt.figure()
    ax = fig0.add_subplot(111)
    # plt.title("PD Tracking Performance", font 1)
    ax.tick_params(labelsize=size2)
    plt.plot(timeDraw, leftRotErrDraw, linewidth=linesize, color='red', label='left_rot_err')
    plt.plot(timeDraw, rightPosErrDraw, linewidth=linesize, color='blue', label='right_pos_err')
    plt.plot(timeDraw, rightRotErrDraw, linewidth=linesize, color='green', label='right_rot_err')
    ax.set_xlim(0.0, timeDraw[-1])
    # ax.set_ylim(0.0, 0.05)
    plt.legend(loc='best')


    fig1 = plt.figure()
    ax = fig1.add_subplot(111)
    plt.title("x")
    ax.tick_params(labelsize=size2)
    plt.plot(timeDraw, leftXDraw, linewidth=linesize, color='red', label='angle')
    plt.plot(timeDraw, leftYDraw, linewidth=linesize, color='green', label='angle')
    plt.plot(timeDraw, leftZDraw, linewidth=linesize, color='blue', label='angle')
    ax.set_xlim(0.0, timeDraw[-1])
    plt.show()



if __name__ == '__main__':
    main()

    # a = [1, 2, 3]
    # b = [4, 5, 6]
    # print np.r_[a, b]
