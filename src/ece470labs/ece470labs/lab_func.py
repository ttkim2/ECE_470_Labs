#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import atan2, pi
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
x_E = 0.0535
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
	x = L3 + L5 + L7 - x_0 
	y = L2 - L4 + L6 + L8 + y_0 + z_E
	z = L1 + z_0 + x_E
	
	M = [[0.0, -1.0, 0.0, x],
	      [0.0, 0.0, -1.0, y],
	      [1.0, 0.0, 0.0, z],	
	      [0.0, 0.0, 0.0, 1]]

	S = np.zeros((6,6))
	w = np.zeros((6,3))
	q = np.zeros((6,3))
	v = np.zeros((6,3))
	# S[0] = [0, 0, 1, 0, 0, 0]
	# S[1] = [0, 1, 0, -L1, 0, 0]
	# S[2] = [0, 1, 0, -L1, 0, L3]
	# S[3] = [0, 1, 0, -L1, 0, L3 + L5]
	# S[4] = [1, 0, 0, 0, -L1, -L2 + L4 - L6]
	# S[5] = [0, 1, 0, -L1, 0, L3 + L5 + L7]
	w[0] = [0, 0, 1]
	w[1] = [0, 1, 0]
	w[2] = [0, 1, 0]
	w[3] = [0, 1, 0]
	w[4] = [1, 0, 0]
	w[5] = [0, 1, 0]
	
	q[0] = [0, 0, L1]
	q[1] = [0, L2, L1]
	q[2] = [L3, L2, L1]
	q[3] = [L3 + L5, L2 - L4, L1]
	q[4] = [L3 + L5, L2 - L4 + L6, L1]
	q[5] = [L3 + L5 + L7, L2 - L4 + L6, L1]

	for i in range(6):
		q[i] = q[i] + np.array([-x_0, y_0, z_0])
		v[i] = np.cross(-w[i], q[i])
		S[i] = np.append(w[i], v[i])

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
	yaw = np.radians(yaw_WgripDegree)
	x_cen = xWgrip - np.cos(yaw) * x_E + x_0
	y_cen = yWgrip - np.sin(yaw) * x_E - y_0
	z_cen = zWgrip - z_0
	
	arcsin_ = (L2 - L4 + L6) / np.sqrt(x_cen**2 + y_cen**2)

	theta1 = np.arctan2(y_cen, x_cen) - np.arcsin(arcsin_)
	theta6 = np.pi/2 + theta1 - yaw

	translation_cen_to_base = np.array([
		[np.cos(theta1), -np.sin(theta1), 0, x_cen],
		[np.sin(theta1), np.cos(theta1)  , 0, y_cen],
		[0,            0,               1,    z_cen],
		[0,            0,               0,     1]
		])
	vec_cen_to_3end = np.array([-L7, -L6 - 0.027, L8 + z_E, 1])

	end3 = vec_cen_to_3end @ translation_cen_to_base.T
	
	x3end = end3[0]
	y3end = end3[1]
	z3end = z_cen + L8 + z_E

	horizontal_distance = np.sqrt(x3end**2 + y3end**2)
	hypotenuse = np.sqrt(horizontal_distance**2 + (z3end - L1)**2)

	theta2 = -(np.arctan2(z3end - L1, horizontal_distance) - np.arccos((L3**2 + hypotenuse**2 - L5**2) / (2 * L3 * hypotenuse)))
	theta3 = np.pi - np.arccos((L3**2 + L5**2 - hypotenuse**2) / (2 * L3 * L5))

	theta4 = theta2 - theta3
	theta5 = -np.pi/2
	
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
