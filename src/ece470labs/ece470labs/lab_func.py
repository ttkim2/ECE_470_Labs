#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
# Link Lengths
L1 = 0.152
L2 = 0.120
L3 = 0.244
L4 = 0.093
L5 = 0.213
L6 = 0.104
L7 = 0.085
L8 = 0.092

# Offset of the UR3 base frame in the world frame
x_0 = 0.150
y_0 = 0.150
z_0 = 0.010

# Offset of the end-effector frame in the gripper frame
x_E = 0.535
z_E = 0.059

def VectoSE3(S):
	omega = S[0:3]
	v = S[3:6]

	# Create 3x3 skew-symmetric matrix for omega
	omega_hat = np.array([
		[0, -omega[2], omega[1]],
		[omega[2], 0, -omega[0]],
		[-omega[1], omega[0], 0]
	])

	# Construct the 4x4 matrix
	se3_matrix = np.zeros((4, 4))
	se3_matrix[0:3, 0:3] = omega_hat
	se3_matrix[0:3, 3] = v
	return se3_matrix

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	x = L3 + L5 + L7
	y = L2 - L4 + L6
	z = L1
	
	M = [[0.0, -1.0, 0.0, x],
	      [0.0, 0.0, -1.0, y],
	      [1.0, 0.0, 0.0, z],	
	      [0.0, 0.0, 0.0, 1]]

	S = np.zeros((6,6))
	S[0] = [0, 0, 1, 0, 0, 0]
	S[1] = [0, 1, 0, -L1, 0, 0]
	S[2] = [0, 1, 0, -L1, 0, L3]
	S[3] = [0, 1, 0, -L1, 0, L3 + L5]
	S[4] = [1, 0, 0, 0, -L1, -L2 + L4 - L6]
	S[5] = [0, 1, 0, -L1, 0, L3 + L5 + L7]
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	T = np.eye(4)
	M, S = Get_MS()
	theta = [theta1, theta2, theta3, theta4, theta5, theta6]
	for i in range(6):
		S_ = VectoSE3(S[i])
		T = T @ expm(S_ * theta[i])
	T = T @ M

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
