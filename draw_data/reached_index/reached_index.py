#!/usr/bin/python
#-*- coding:utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt


# pos_error_draw = np.loadtxt(filenum + "pos_error_draw.txt")
# euler_error_draw = np.loadtxt(filenum + "euler_error_draw.txt")
# joint_angles_draw = np.loadtxt(filenum + "joint_angles_draw.txt")
# joint_angles_bounds = np.loadtxt(filenum + "joint_bounds.txt")
test_num = 1

for i in range(test_num):
    pose = []
    pose.append(np.loadtxt("pose" + str(i) + "0.txt"))
    pose.append(np.loadtxt("pose" + str(i) + "1.txt"))
    pose.append(np.loadtxt("pose" + str(i) + "2.txt"))
    pose.append(np.loadtxt("pose" + str(i) + "3.txt"))

    datalength = len(pose[0][:, 0])
    data_index = 0

    for j in range(4):
        if len(pose[j]) < datalength:
            datalength = len(pose[j][:, 0])
            data_index = j

    print "datalength", datalength

    names = ["a_tree_reached", "b_tree_reached", "a_tree_nearest", "b_tree_nearest"]
    for k in range(4):
        plt.figure(i)
        plt.subplot(2, 2, k+1)
        plt.title("test"+str(i) + "pose" + str(data_index) + names[k])
        x = np.linspace(0, datalength, datalength)
        for j in range(4):
            plt.plot(x, pose[j][0: datalength, k], label="pose"+str(j))
        plt.legend(loc="best")
plt.show()
# x = np.linspace(0, data_length, data_length)
# plt.figure()
# plt.subplot(4, 1, 1)
# plt.title("angle_error")
# plt.plot(x, angle_error_draw[0:data_length])
#
# plt.subplot(4, 1, 2)
# plt.title("euler_r_error")
# plt.plot(x, euler_error_draw[0:data_length, 2])
#
# plt.subplot(4, 1, 3)
# plt.title("euler_p_error")
# plt.plot(x, euler_error_draw[0:data_length, 1])
#
# plt.subplot(4, 1, 4)
# plt.title("euler_y_error")
# plt.plot(x, euler_error_draw[0:data_length, 0])

# x = np.linspace(0, data_length, data_length)
# plt.figure()
# plt.subplot(6, 1, 1)
# plt.title("pos_x_error")
# plt.plot(x, pos_error_draw[0:data_length, 0])
#
# plt.subplot(6, 1, 2)
# plt.title("pos_y_error")
# plt.plot(x, pos_error_draw[0:data_length, 1])
#
# plt.subplot(6, 1, 3)
# plt.title("pos_z_error")
# plt.plot(x, pos_error_draw[0:data_length, 2])
#
# plt.subplot(6, 1, 4)
# plt.title("euler_y_error")
# plt.plot(x, euler_error_draw[0:data_length, 0])
#
# plt.subplot(6, 1, 5)
# plt.title("euler_p_error")
# plt.plot(x, euler_error_draw[0:data_length, 1])
#
# plt.subplot(6, 1, 6)
# plt.title("euler_r_error")
# plt.plot(x, euler_error_draw[0:data_length, 2])

#
# plt.figure()
# plt.subplot(7, 1, 1)
# plt.title("s0")
# plt.plot(x, joint_angles_bounds[0, 0] * np.ones(data_length))
# plt.plot(x, joint_angles_bounds[1, 0] * np.ones(data_length))
# plt.plot(x, joint_angles_draw[0:data_length, 0])
#
# plt.subplot(7, 1, 2)
# plt.title("s1")
# plt.plot(x, joint_angles_bounds[0, 1] * np.ones(data_length))
# plt.plot(x, joint_angles_bounds[1, 1] * np.ones(data_length))
# plt.plot(x, joint_angles_draw[0:data_length, 1])
#
# plt.subplot(7, 1, 3)
# plt.title("e0")
# plt.plot(x, joint_angles_bounds[0, 2] * np.ones(data_length))
# plt.plot(x, joint_angles_bounds[1, 2] * np.ones(data_length))
# plt.plot(x, joint_angles_draw[0:data_length, 2])
#
# plt.subplot(7, 1, 4)
# plt.title("e1")
# plt.plot(x, joint_angles_bounds[0, 3] * np.ones(data_length))
# plt.plot(x, joint_angles_bounds[1, 3] * np.ones(data_length))
# plt.plot(x, joint_angles_draw[0:data_length, 3])
#
# plt.subplot(7, 1, 5)
# plt.title("w0")
# plt.plot(x, joint_angles_bounds[0, 4] * np.ones(data_length))
# plt.plot(x, joint_angles_bounds[1, 4] * np.ones(data_length))
# plt.plot(x, joint_angles_draw[0:data_length, 4])
#
# plt.subplot(7, 1, 6)
# plt.title("w1")
# plt.plot(x, joint_angles_bounds[0, 5] * np.ones(data_length))
# plt.plot(x, joint_angles_bounds[1, 5] * np.ones(data_length))
# plt.plot(x, joint_angles_draw[0:data_length, 5])
#
# plt.subplot(7, 1, 7)
# plt.title("w2")
# plt.plot(x, joint_angles_bounds[0, 6] * np.ones(data_length))
# plt.plot(x, joint_angles_bounds[1, 6] * np.ones(data_length))
# plt.plot(x, joint_angles_draw[0:data_length, 6])

# plt.show()