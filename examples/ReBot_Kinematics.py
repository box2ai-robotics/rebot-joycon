# code by Boxjod 2025.12.01 Box2AI-Robotics copyright 盒桥智能 版权所有
"""reBot B601-DM：base_link（底座根部）到夹爪尖端中心 TCP 的六轴运动学。

长度 m，关节/姿态 rad；q=0 为 URDF 六轴零位，不附加工作姿态或电机标定偏置。
参数来源：3_模型参数/rebot_b601_dm/dh_parameters.json（标准 DH）。
TCP 沿 end_link 的局部 X 轴 -0.105 m，采用同目录 MuJoCo 模型的 tcp。
这是模型中的夹爪尖端中心，不随夹爪开合变化，也不是单侧指尖或实测 TCP。
"""

import numpy as np
import math
from math import sqrt as sqrt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from importlib import import_module
ReBotCalibration = import_module("0_calibrate_rebot").ReBotCalibration
from scipy.spatial.transform import Rotation as R

# 每行 [a, alpha, d, theta_offset]；A_i = Rz(q_i+offset) Tz(d) Tx(a) Rx(alpha)。
# 保留原 URDF 的角度精度；joint2 的轴方向已包含在 DH 中，不再对 q2 取反。
REBOT_DH = np.array([
    [0.020084, 1.5707926535897934, 0.05555011616511138, -3.454936219830327e-16],
    [0.26399999999999996, 3.141592653589793, -0.03162500000021327, -3.141592653589793],
    [0.24853724067028665, 2.4609998553816117e-22, -0.0016250000000000014, 2.922574576194731],
    [0.07830799999999999, -1.5708, -0.030000137745191377, 0.21901807739506274],
    [2.117582368135751e-22, 1.5708000000000002, 0.002500102878875504, 1.5707963267948966],
    [-2.117582368135751e-22, 3.088570086371268e-22, 0.0, -1.0364362966181977e-22],
], dtype=float)
REBOT_JOINT_LIMITS = np.array([
    [-2.8, 2.8],
    [-3.14, 0.0],
    [-3.14, 0.0],
    [-1.87, 1.57],
    [-1.57, 1.57],
    [-3.14, 3.14],
], dtype=float)
REBOT_BASE_TRANSFORM = np.array([
    [1.0, 0.0, 0.0, -8.416e-05],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.08465],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)
REBOT_END_TRANSFORM = np.array([
    [-3.4033563838089864e-10, -0.9999999957076559, -9.265358965986517e-05, -2.117582368135751e-22],
    [-3.673205087492488e-06, 9.265358966049024e-05, -0.99999999570091, 2.7755575615628914e-17],
    [0.9999999999932538, -3.0354278852653465e-26, -3.6732051033524324e-06, 0.18339800000018897],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)
REBOT_TCP_OFFSET = np.array([-0.105, 0.0, 0.0])


def create_rebot_b601_dm(calibration=None):
    """标准 DH 模型；六个输入依次为 joint1…joint6，不包含夹爪电机。"""
    if calibration is None:
        calibration = ReBotCalibration()
    calibrated_limits = calibration.control_limits(REBOT_JOINT_LIMITS.T)[:, :6].T
    links = [
        RevoluteDH(a=a, alpha=alpha, d=d, offset=offset, qlim=limits)
        for (a, alpha, d, offset), limits in zip(REBOT_DH, calibrated_limits)
    ]
    return DHRobot(
        links,
        name="reBot_B601_DM_TCP",
        base=SE3(REBOT_BASE_TRANSFORM),
        tool=SE3(REBOT_END_TRANSFORM) * SE3.Trans(REBOT_TCP_OFFSET),
    )


DEFAULT_HOME_Z = 0.25  # 底座坐标系下的 TCP 高度，单位 m


def get_home_q(robot=None, *, z=DEFAULT_HOME_Z):
    """保留六轴零位的 TCP X/Y/姿态，仅把 Z 设为指定初始高度。"""
    if robot is None:
        robot = create_rebot_b601_dm()
    target = robot.fkine(np.zeros(6)).A.copy()
    target[2, 3] = z
    solution = robot.ikine_LM(
        target, q0=np.zeros(6), tol=1e-18, ilimit=100, slimit=1)
    if (not solution.success
            or np.any(solution.q < robot.qlim[0])
            or np.any(solution.q > robot.qlim[1])
            or not np.allclose(robot.fkine(solution.q).A, target, atol=1e-6, rtol=0)):
        raise RuntimeError(f"reBot 初始 Z={z} m 位姿求解失败")
    return solution.q.copy()


# 保留旧导入入口；这些名称现在均返回六轴 reBot 运动学，不再表示旧 BoxArm。
def create_boxarm_b0():
    return create_rebot_b601_dm()


def create_boxarm_b1():
    return create_rebot_b601_dm()


def create_boxarm_b2():
    return create_rebot_b601_dm()


def get_robot(robot="rebot_b601_dm"):
    if robot in ("rebot_b601_dm", "boxarm_b0", "boxarm_b1", "boxarm_b2"):
        return create_rebot_b601_dm()
    raise ValueError(f"Unsupported robot: {robot}")


def manual_gravity_compensation(robot, q):
    # 旧 BoxArm 的质量/质心不适用于 reBot；本文件只提供运动学。
    raise NotImplementedError("reBot 重力补偿请使用官方 SDK 动力学，不能使用旧 BoxArm 参数")


# Retain 15 decimal places and round off after the 15th place
def atan2(first, second):
    return round(math.atan2(first, second), 3)

def sin(radians_angle):
    return round(math.sin(radians_angle), 3)

def cos(radians_angle):
    return round(math.cos(radians_angle), 3)

def acos(value):
    return round(math.acos(value), 3)

def round_value(value):
    return round(value, 3)

def rebot_FK(qpos_data, robot):
    if len(qpos_data) != len(robot.qlim[0]):
        raise Exception("The dimensions of qpose_data are not the same as the robot joint dimensions")
    # Get the end effector's homogeneous transformation matrix (T is an SE3 object)
    T = robot.fkine(qpos_data)
    
    # Extract position (X, Y, Z) — use SE3 object's attribute
    X, Y, Z = T.t  # Directly use t attribute to get position (X, Y, Z)
    
    # Extract rotation matrix (T.A) and calculate Euler angles (alpha, beta, gamma)
    R = T.R  # Get the rotation part (3x3 matrix)

    # Calculate Euler angles
    beta = atan2(-R[2, 0], sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    if cos(beta) != 0:  # Ensure no division by zero
        alpha = atan2(R[1, 0] / cos(beta), R[0, 0] / cos(beta))
        gamma = atan2(R[2, 1] / cos(beta), R[2, 2] / cos(beta))
    else:  # When cos(beta) is zero, singularity occurs
        alpha = 0
        gamma = atan2(R[0, 1], R[1, 1])
    
    return np.array([X, Y, Z, gamma, beta, alpha])
    
def rebot_IK(q_now, target_pose, robot, *, tol=1e-9, ilimit=10, slimit=2, quiet=False):
    # 所有 Gpos 共用：平方误差阈值 1e-9，避免 0.8 mm 小步被视作已到达。
    if len(q_now) != len(robot.qlim[0]):
        raise Exception("The dimensions of qpose_data are not the same as the robot joint dimensions")
    # R = SE3.RPY(target_pose[3:])
    # T = SE3(target_pose[:3]) * R
    
    x, y, z, roll, pitch, yaw = target_pose
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)  # 欧拉角的顺序是 XYZ
    R_mat = r.as_matrix()  # 获取旋转矩阵
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = [x, y, z]
    
    sol = robot.ikine_LM(
            Tep=T, 
            q0=q_now,
            ilimit=ilimit,
            slimit=slimit,
            tol=tol)  # 阈值为 0.5 * e.T @ e，不是距离本身
    
    if sol.success:
        # If IK solution is successful, 
        q = sol.q
        q = smooth_joint_motion(q_now, q, robot)
        # print(f'{q=}')
        return q, True
    else:
        # If the target position is unreachable, IK fails
        if not quiet:
            print('IK fails')
        return -1 * np.ones(len(q_now)), False
    
def smooth_joint_motion(q_now, q_new, robot):
    q_current = q_now
    max_joint_change = 0.1 
    
    for i in range(len(q_new)):
        delta = q_new[i] - q_current[i]
        if abs(delta) > max_joint_change:
            delta = np.sign(delta) * max_joint_change
        q_new[i] = q_current[i] + delta
    
    robot.q = q_new
    return q_new
