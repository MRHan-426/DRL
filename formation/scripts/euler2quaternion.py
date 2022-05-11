import time
import math
from geometry_msgs.msg import *
import rospy
import tf
import numpy as np
x = 0.0
y = 0.0
agent_x = -0.503
agent_y = 0.135
print((agent_y-y)/(agent_x-x))
yaw = math.atan2((agent_y-y),(agent_x-x))
print(yaw)

pos = Pose()
q = tf.transformations.quaternion_from_euler(0, 0, yaw)
pos.orientation.x = q[0]
pos.orientation.y = q[1]
pos.orientation.z = q[2]
pos.orientation.w = q[3]

print(pos)




# def euler_from_quaternion(self, x, y, z, w):
#     euler = [0, 0, 0]
#     Epsilon = 0.0009765625
#     Threshold = 0.5 - Epsilon
#     TEST = w * y - x * z
#     if TEST < -Threshold or TEST > Threshold:
#         if TEST > 0:
#             sign = 1
#         elif TEST < 0:
#             sign = -1
#         euler[2] = -2 * sign * math.atan2(x, w)
#         euler[1] = sign * (math.pi / 2.0)
#         euler[0] = 0
#     else:
#         euler[0] = math.atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z)
#         euler[1] = math.asin(-2 * (x * z - w * y))
#         euler[2] = math.atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)
#     return euler

# 欧拉角转换为四元数, 旋转顺序为ZYX(偏航角yaw, 俯仰角pitch, 横滚角roll)
def eular2quat(self,yaw, pitch, roll):
    # 注意这里必须先转换为弧度, 因为这里的三角计算均使用的是弧度.
    yaw = math.radians(yaw)
    pitch = math.radians(pitch)
    roll = math.radians(roll)

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    # 笛卡尔坐标系
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    # Direct3D, 笛卡尔坐标的X轴变为Z轴, Y轴变为X轴, Z轴变为Y轴
    # w = cr * cp * cy + sr * sp * sy
    # x = cr * sp * cy + sr * cp * sy
    # y = cr * cp * sy - sr * sp * cy
    # z = sr * cp * cy - cr * sp * sy

    return w, x, y, z

    