#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时获取小车的位置，累加计算出小车行驶的路程
rostopic hz /odom   max 333hz
@author: ziqi han
"""

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


last_position_x = 0.0
last_position_y = 0.0
s = 0.0

def odom_callback(msg):
    global last_position_x,last_position_y,s
    odom_pub = rospy.Publisher("trajectory_length", Float32, queue_size=1)
    trajectory_length = Float32()

    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    delta_x = position_x - last_position_x
    delta_y = position_y - last_position_y

    # Env Reset
    if math.fabs(position_x) <= 0.005 and math.fabs(position_y) <= 0.005:
        last_position_x = position_x
        last_position_y = position_y
        s = 0.0
        # print "路程为:", s
        trajectory_length.data = s
        odom_pub.publish(trajectory_length)
        return None
    # 小车运行到一定位置后才开启此节点
    # 小车瞬移
    elif math.fabs(delta_x) >= 0.01 or math.fabs(delta_y) >= 0.01:
        last_position_x = position_x
        last_position_y = position_y
        s = 0.0
        # print "路程为:", s
        trajectory_length.data = s
        odom_pub.publish(trajectory_length)
        return None

    # 防止静态误差的累计
    if math.fabs(delta_x) <= 0.00001:
        delta_x = 0
    if  math.fabs(delta_y) <= 0.00001:
        delta_y = 0
    
    s = s + math.sqrt(delta_x** 2 + delta_y** 2)

    # print "路程为:", s
    last_position_x = position_x
    last_position_y = position_y
    trajectory_length.data = s
    odom_pub.publish(trajectory_length)
    #return s

if __name__=='__main__':
  rospy.init_node('calculate_trajectory',anonymous=True)
  odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)
  rospy.spin()


