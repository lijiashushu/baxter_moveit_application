#!/usr/bin/python
#-*- coding:utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

filenum = "0"
# pos_error_draw = np.loadtxt(filenum + "pos_error_draw.txt")
# euler_error_draw = np.loadtxt(filenum + "euler_error_draw.txt")
# joint_angles_draw = np.loadtxt(filenum + "joint_angles_draw.txt")
# joint_angles_bounds = np.loadtxt(filenum + "joint_bounds.txt")

euler_error_draw = np.loadtxt(filenum + "euler_error_draw.txt")
angle_error_draw = np.loadtxt(filenum + "rot_error_angle_draw.txt")

data_length = len(euler_error_draw)

x = np.linspace(0, data_length, data_length)
plt.figure()
plt.subplot(4, 1, 1)
plt.title("angle_error")
plt.plot(x, angle_error_draw[0:data_length])

plt.subplot(4, 1, 2)
plt.title("euler_r_error")
plt.plot(x, euler_error_draw[0:data_length, 2])

plt.subplot(4, 1, 3)
plt.title("euler_p_error")
plt.plot(x, euler_error_draw[0:data_length, 1])

plt.subplot(4, 1, 4)
plt.title("euler_y_error")
plt.plot(x, euler_error_draw[0:data_length, 0])

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

plt.show()