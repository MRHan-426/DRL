#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/laneline_info话题，自定义消息类型laneline_publisher::laneline

import rospy
import tf,copy,math
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point,Twist
from std_msgs.msg import Int8
import numpy as np
import numpy.linalg as LA

class Tracking():
    def __init__(self):
        # car params
        controller_freq = 20
        self.baseSpeed = 0.3
        self.Lfw = 0.2 # 最小前馈距离
        self.goalRadius = 0.1 # 到达目标点容忍距离

        rospy.init_node('Tracking', anonymous=True)
        self.listener = tf.TransformListener()
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.pathCallback)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/robot/goal", PoseStamped, self.goalCallback)
        self.pub_new_goal = rospy.Publisher("/robot/arrive",Int8,queue_size=1)
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.pub_point = rospy.Publisher("/forwardpoint",Point,queue_size=1)
        self.timer1 = rospy.Timer(rospy.Duration((1.0)/controller_freq),self.time1callback)
        self.timer2 = rospy.Timer(rospy.Duration((1.0)/controller_freq),self.time2callback)

        # params
        self.path = Path()
        self.odom = Odometry()
        self.goal = PoseStamped().pose.position
        self.cmd_vel = Twist()

        # path params
        self.foundForwardPt = False
        self.goal_received = False
        self.goal_reached = False


    def pathCallback(self,data):
        self.path = data

    
    def odomCallback(self,data):
        self.odom = data
    
    def goalCallback(self,data):
        self.goal = data.pose.position
        self.goal_received = True
        self.goal_reached = False
        

    def time1callback(self,event):
        if(self.goal_received):
            car2goal_dis = self.getCar2GoalDist()
            if(car2goal_dis<self.goalRadius):
                self.goal_reached = True
                self.goal_received = False
                a = Int8()
                a = 1
                self.pub_new_goal.publish(a)


    def getCar2GoalDist(self):
        current_odom = copy.deepcopy(self.odom.pose.pose.position)
        car2goal_x = current_odom.x - self.goal.x
        car2goal_y = current_odom.y - self.goal.y
        dist2goal = math.sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y)
        return dist2goal


    def getYaw(self,pose):
        q = tf.transformations.euler_from_quaternion((pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
        return q[2]

    def getEta(self,carPose):
        odom_car2WayPtVec = self.get_odom_car2WayPtVec(carPose)
        eta = math.atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x)
        mk = 2*math.sin(eta)/math.sqrt(odom_car2WayPtVec.y*odom_car2WayPtVec.y+odom_car2WayPtVec.x*odom_car2WayPtVec.x)
        return mk

    def get_odom_car2WayPtVec(self,carPose):
        carPose_pos = Point()
        carPose_pos = carPose.position
        carPose_yaw = self.getYaw(carPose)
        forwardPt = Point()
        odom_car2WayPtVec = Point()
        myforwardPoint = Point()
        self.foundForwardPt = False
        if not self.goal_reached:
            path = copy.deepcopy(self.path.poses)

            for i in range(len(path)):
                map_path_pose = path[i]
                # odom_path_pose = PoseStamped()
                # self.listener.waitForTransform("odom", "map", rospy.Time(), rospy.Duration(4.0))
                # odom_path_pose  = self.listener.transformPose("odom",map_path_pose)
                try:
                    self.listener.waitForTransform("odom", "map", rospy.Time(), rospy.Duration(4.0))
                    odom_path_pose  = self.listener.transformPose("odom",map_path_pose)
                    odom_path_wayPt = odom_path_pose.pose.position
                    _isForwardWayPt = self.isForwardWayPt(odom_path_wayPt,carPose)
                    if(_isForwardWayPt):
                        _isWayPtAwayFromLfwDist = self.isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos)
                        if(_isWayPtAwayFromLfwDist):
                            forwardPt = odom_path_wayPt
                            self.foundForwardPt = True


                            # draw the point
                            myforwardPoint.x = path[30].pose.position.x
                            myforwardPoint.y = path[30].pose.position.y
                            myforwardPoint.z = path[30].pose.position.z
                            self.pub_point.publish(myforwardPoint)
                            break
                except:
                    pass
        elif(self.goal_reached):
            forwardPt = self.odom
            self.foundForwardPt = False
        odom_car2WayPtVec.x = math.cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + math.sin(carPose_yaw)*(forwardPt.y - carPose_pos.y)
        odom_car2WayPtVec.y = -math.sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + math.cos(carPose_yaw)*(forwardPt.y - carPose_pos.y)
        return odom_car2WayPtVec


    def isForwardWayPt(self,wayPt,carPose):
        car2wayPt_x = wayPt.x - carPose.position.x
        car2wayPt_y = wayPt.y - carPose.position.y
        car_theta = self.getYaw(carPose)

        car_car2wayPt_x = math.cos(car_theta)*car2wayPt_x + math.sin(car_theta)*car2wayPt_y
        car_car2wayPt_y = -1*math.sin(car_theta)*car2wayPt_x + math.cos(car_theta)*car2wayPt_y

        if(car_car2wayPt_x >0):
            return True
        else:
            return False   
    def isWayPtAwayFromLfwDist(self,wayPt,car_pos):
        dx = wayPt.x - car_pos.x
        dy = wayPt.y - car_pos.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < self.Lfw:
            return False
        else:
            return True

    def time2callback(self,event):
        q = copy.deepcopy(self.odom)
        carPose = q.pose.pose
        carVel = q.twist.twist
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        if(self.goal_received):
            eta = self.getEta(carPose)
            if (self.foundForwardPt):
                if not self.goal_reached:
                    self.cmd_vel.linear.x = self.baseSpeed
                    self.cmd_vel.angular.z = eta*self.cmd_vel.linear.x
        self.pub.publish(self.cmd_vel)


        
if __name__ == '__main__':
    track = Tracking()
    rospy.spin()
