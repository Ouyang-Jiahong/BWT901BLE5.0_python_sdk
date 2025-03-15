import asyncio
import math

import bleak
from datetime import datetime
import time
import numpy as np

import device_model

# 扫描到的设备
devices = []
# 蓝牙设备
BLEDevice = None

# 初始化状态变量 Initialize state variables
last_time = None  # 上次更新时间 Last update time
velocity_x = 0.0  # X轴速度 X-axis velocity
velocity_y = 0.0  # Y轴速度 Y-axis velocity
velocity_z = 0.0  # Z轴速度 Z-axis velocity
speed_drift_correction_factor = 0.1  # 速度漂移校正系数 Speed drift correction factor, adjust as needed


# 指定MAC地址搜索并连接设备
async def scanByMac(device_mac):
    global BLEDevice
    print("Searching for Bluetooth devices......")
    BLEDevice = await bleak.BleakScanner.find_device_by_address(device_mac, timeout=20)


# 数据更新时会调用此方法
def updateData(DeviceModel):
    global last_time, velocity_x, velocity_y, velocity_z

    # 获取当前时间戳 Get current timestamp
    current_datetime = datetime.now()
    current_time = current_datetime.timestamp()

    if last_time is not None:
        dt = current_time - last_time
    else:
        dt = 0.0
    last_time = current_time  # 更新最后时间戳

    # 获得加速度数据
    acc_x = DeviceModel.get("AccX")
    acc_y = DeviceModel.get("AccY")
    acc_z = DeviceModel.get("AccZ")

    # 获得欧拉角数据
    roll = math.radians(DeviceModel.get("AngX"))
    pitch = math.radians(DeviceModel.get("AngY"))
    yaw = math.radians(DeviceModel.get("AngZ"))
    # 打印加速度数据
    # print(f"加速度数据: AccX={acc_x}, AccY={acc_y}, AccZ={acc_z}")
    # 打印欧拉角数据
    # print(f"欧拉角数据: Roll={roll:.2f} rad, Pitch={pitch:.2f} rad, Yaw={yaw:.2f} rad")

    # 更新速度并进行速度漂移校正
    if all([acc_x, acc_y, acc_z, roll, pitch, yaw]):
        velocity_x, velocity_y, velocity_z = integrate_and_correct_velocity(acc_x, acc_y, acc_z, roll, pitch, yaw, dt)
        # 输出X, Y, Z轴的速度 Print X, Y, Z axis velocities
        # print(f"Velocity X: {velocity_x:.2f} m/s, Y: {velocity_y:.2f} m/s, Z: {velocity_z:.2f} m/s")
        # 计算并输出合速度 Calculate and print the resultant speed
        resultant_speed = np.sqrt(velocity_x ** 2 + velocity_y ** 2 + velocity_z ** 2)
        print(f"Resultant Speed: {resultant_speed:.2f} m/s")
    else:
        print("Warning: Missing or invalid data for updating velocity. Skipping this update.")


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
    r_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    r_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    r_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # 总旋转矩阵：r = r_z * r_y * r_x
    r = r_z @ r_y @ r_x
    return r


def integrate_and_correct_velocity(acc_x, acc_y, acc_z, roll, pitch, yaw, dt):
    """
    对加速度进行重力校正并进行数值积分，返回更新后的速度。
    :param acc_x: X轴加速度
    :param acc_y: Y轴加速度
    :param acc_z: Z轴加速度
    :param roll: 滚转角（绕X轴），单位为弧度。
    :param pitch: 俯仰角（绕Y轴），单位为弧度。
    :param yaw: 偏航角（绕Z轴），单位为弧度。
    :param dt: 时间差
    :return: 更新后的速度 (vx, vy, vz)
    """
    global velocity_x, velocity_y, velocity_z
    # 生成校正矩阵，需要生成重力扣除向量
    r = rotation_matrix_from_body_to_ned(roll, pitch, yaw)
    gravity = np.array([[0], [0], [1]])
    gravity_subtraction_vector = np.dot(r, gravity)

    # 使用梯形法则进行数值积分
    if dt > 0:
        velocity_x += (acc_x + gravity_subtraction_vector[0, 0]) * dt
        velocity_y += (acc_y + gravity_subtraction_vector[1, 0]) * dt
        velocity_z += (acc_z + gravity_subtraction_vector[2, 0]) * dt

    return velocity_x, velocity_y, velocity_z


if __name__ == '__main__':
    # 指定MAC地址搜索并连接设备
    asyncio.run(scanByMac("F0:FC:E2:AD:C1:7E"))

    if BLEDevice is not None:
        # 创建设备
        device = device_model.DeviceModel("MyBle5.0", BLEDevice, updateData)
        # 开始连接设备
        asyncio.run(device.openDevice())
    else:
        print("This BLEDevice was not found!!")
