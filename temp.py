import math

import numpy as np


def rotation_matrix_from_body_to_ned(roll, pitch, yaw):
    """
    计算从体坐标系到东北天坐标系的旋转矩阵。

    参数:
        roll (float): 滚转角（绕X轴），单位为弧度。
        pitch (float): 俯仰角（绕Y轴），单位为弧度。
        yaw (float): 偏航角（绕Z轴），单位为弧度。

    返回:
        np.ndarray: 3x3的旋转矩阵。
    """
    # 构建旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # 总旋转矩阵：R = R_z * R_y * R_x
    R = R_z @ R_y @ R_x
    return R


roll = math.radians(45)
pitch = math.radians(45)
yaw = math.radians(0)
R = rotation_matrix_from_body_to_ned(roll, pitch, yaw)
#print(R)
Gravity = np.array([[0], [0], [1]])
#print(Gravity)
Gravity_Subtraction_Vector = np.dot(R, Gravity)
print(Gravity_Subtraction_Vector)
print(Gravity_Subtraction_Vector[0,0])
print(Gravity_Subtraction_Vector[1,0])
print(Gravity_Subtraction_Vector[2,0])