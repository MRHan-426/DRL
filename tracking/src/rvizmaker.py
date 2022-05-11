#!/usr/bin/env python
#coding=utf-8
 
"""
这个例子是简单的在Rviz中显示一个Marker
"""	
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
class Maker():
    def __init__(self):
        rospy.init_node('maker', anonymous=True)
        rospy.Subscriber("forwardpoint",Point, self.callback)
        self.marker_pub = rospy.Publisher("/cube", Marker, queue_size=10)
        self.marker = Marker()
	
        #指定self.marker的参考框架
        self.marker.header.frame_id = "/map"
        
        #时间戳
        self.marker.header.stamp = rospy.Time.now()
        
        #ns代表namespace，命名空间可以避免重复名字引起的错误
        self.marker.ns = "basic_shapes"
        
        #self.marker的id号
        self.marker.id = 0
        
        #self.marker的类型，有ARROW，CUBE等
        self.marker.type = self.marker.CYLINDER
        
        #self.marker的尺寸，单位是m
        self.marker.scale.x = 0.01
        self.marker.scale.y = 0.01
        self.marker.scale.z = 0.01
        
        #self.marker的动作类型有ADD，DELETE等
        self.marker.action = self.marker.ADD
        
        #self.marker的位置姿态
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.2
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        
        #self.marker的颜色和透明度
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        
        #self.marker被自动销毁之前的存活时间，rospy.Duration()意味着在程序结束之前一直存在
        self.marker.lifetime = rospy.Duration()

    def callback(self,data):
        self.marker.pose.position.x = data.x
        self.marker.pose.position.y = data.y
        self.marker.pose.position.z = data.z
        self.marker_pub.publish(self.marker)


if __name__ == '__main__':
    maker = Maker()
    rospy.spin()
 
