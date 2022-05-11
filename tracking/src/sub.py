#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid

def callback_fuc(data):      #该data则相当于发布者的send_msg结构体
    a = np.array(data.data,dtype=int)
    a = np.where(a > 0, 255, 0)
    cv2.imshow('1',a)
    vv = 1
    
def listener():
    rospy.init_node('listener', anonymous=True)  #初始化名为“listener”的订阅者节点：anonymous=True会为节点生成唯一的名称避免节点重名
    rospy.Subscriber("/move_base/GlobalPlanner/potential", OccupancyGrid, callback_fuc)  #设置订阅者属性：订阅的话题名“chatter”，话题对应的msg消息文件名，回调函数名
    rospy.spin()     #必须要有，否则无法接收，但会相当于死循环程序会卡在这里
    
if __name__ == '__main__':
    listener()
