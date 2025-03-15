import asyncio
import math

import bleak
from datetime import datetime
import time
import numpy as np
import threading

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

import device_model

# 扫描到的设备
devices = []
# 蓝牙设备
BLEDevice = None

# 初始化状态变量 Initialize state variables
last_time = None  # 上次更新时间 Last update time
acc_x = 0.0  # X轴加速度
acc_y = 0.0  # Y轴加速度
acc_z = 0.0  # Z轴加速度
roll = 0.0
pitch = 0.0
yaw = 0.0
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
    global last_time, velocity_x, velocity_y, velocity_z, acc_x, acc_y, acc_z, roll, pitch, yaw

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

    # 更新速度并进行速度漂移校正
    velocity_x, velocity_y, velocity_z = integrate_and_correct_velocity(dt)


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


def integrate_and_correct_velocity(dt):
    """
    对加速度进行重力校正并进行数值积分，返回更新后的速度。
    :param dt: 时间差
    :return: 更新后的速度 (vx, vy, vz)
    """
    global velocity_x, velocity_y, velocity_z, acc_x, acc_y, acc_z, roll, pitch, yaw
    # 生成校正矩阵，需要生成重力扣除向量
    r = rotation_matrix_from_body_to_ned(roll, pitch, yaw)
    gravity = np.array([[0], [0], [1]])
    gravity_subtraction_vector = np.dot(r, gravity)
    # 定义静止状态的加速度阈值
    STOP_THRESHOLD = 0.1
    ax = acc_x - gravity_subtraction_vector[0, 0]
    ay = acc_y - gravity_subtraction_vector[1, 0]
    az = acc_z - gravity_subtraction_vector[2, 0]

    # 判断是否停止
    isStop = (abs(ax) < STOP_THRESHOLD and abs(ay) < STOP_THRESHOLD and abs(az) < STOP_THRESHOLD)
    if not isStop:
        # 使用梯形法则进行数值积分
        if dt > 0:
            velocity_x += ax * dt
            velocity_y += ay * dt
            velocity_z += az * dt
    else:  # 如果设备停止
        velocity_x = 0
        velocity_y = 0
        velocity_z = 0
    return velocity_x, velocity_y, velocity_z


def print_velocity_data():
    while True:
        # 打印分割行
        print("**************************************************")
        # 打印速度数据
        print(f"速度数据：velocity_x: {velocity_x:.2f}, velocity_y={velocity_y:.2f}, velocity_z={velocity_z:.2f}")
        # 打印加速度数据
        print(f"加速度数据: AccX={acc_x}, AccY={acc_y}, AccZ={acc_z}")
        # 打印欧拉角数据
        print(f"欧拉角数据: Roll={roll:.2f} rad, Pitch={pitch:.2f} rad, Yaw={yaw:.2f} rad")
        # 打印分割行
        print("**************************************************")
        time.sleep(3)


def draw_acc_data():
    # 全局变量
    global acc_x, acc_y, acc_z

    acc_x_list = []
    acc_y_list = []
    acc_z_list = []

    # 更新加速度数据列表的函数
    def update_data_lists():
        acc_x_list.append(acc_x)
        acc_y_list.append(acc_y)
        acc_z_list.append(acc_z)

        # 保持列表只包含最近的600个数据点
        if len(acc_x_list) > 600:
            acc_x_list.pop(0)
            acc_y_list.pop(0)
            acc_z_list.pop(0)

    # 创建图形和轴
    fig, ax = plt.subplots()
    line_x, = ax.plot([], [], label='X Acc')
    line_y, = ax.plot([], [], label='Y Acc')
    line_z, = ax.plot([], [], label='Z Acc')
    ax.legend()

    # 设置标题和坐标轴标签
    ax.set_title('Acceleration Data Real-time Display')
    ax.set_xlabel('Time (samples)')
    ax.set_ylabel('Acceleration (g)')

    # 动画初始化函数
    def init():
        ax.set_xlim(0, 600)
        ax.set_ylim(-2, 2)
        return line_x, line_y, line_z  # 返回所有需要更新的对象

    # 动画更新函数
    def update(frame):
        update_data_lists()  # 更新数据
        print(acc_x_list)
        line_x.set_data(range(len(acc_x_list)), acc_x) # 还需要往里面添加数据，添加数据的功能还没实现成功。
        line_y.set_data(range(len(acc_y_list)), acc_y)
        line_z.set_data(range(len(acc_z_list)), acc_z)
        return line_x, line_y, line_z  # 同样返回所有更新过的对象

    # 创建动画
    while True:
        print(acc_x)
        print(acc_y)
        print(acc_z)
        time.sleep(0.1)

    ani = FuncAnimation(fig, update, frames=600, init_func=init, blit=True)

    plt.show()


if __name__ == '__main__':

    # 开启数据打印线程
    t1 = threading.Thread(target=print_velocity_data)
    t1.start()

    t2 = threading.Thread(target=draw_acc_data)
    t2.start()

    # 指定MAC地址搜索并连接设备
    asyncio.run(scanByMac("F0:FC:E2:AD:C1:7E"))

    if BLEDevice is not None:
        # 创建设备
        device = device_model.DeviceModel("MyBle5.0", BLEDevice, updateData)
        # 开始连接设备
        asyncio.run(device.openDevice())
    else:
        while True:
            print("This BLEDevice was not found!!")
