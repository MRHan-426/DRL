#!/usr/bin/env python3
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
        self.baseSpeed = 0.4  # 小车速度


        # 暂时没用
        self.maxSpeed = 0.5
        self.minSpeed = 0.2
        self.K = -1*0.8

        
        self.Lfw = 0.2 # 最小前馈距离
        self.goalRadius = 0.1 # 到达目标点容忍距离
        self.forwardpath = 20 # 启动时的路径曲率前馈距离
        # self.minforwardpath = 10 # 最小路径曲率前馈距离
        self.vscale = 0.01

        rospy.init_node('Tracking', anonymous=True)
        self.listener = tf.TransformListener()
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.pathCallback)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/robot/goal", PoseStamped, self.goalCallback)
        self.pub_new_goal = rospy.Publisher("/robot/arrive",Int8,queue_size=1)
        self.pub = rospy.Publisher("/nav_cmd_vel",Twist,queue_size=1)
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
        self.baseSpeed = 0.4
        

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
        d = math.sqrt(odom_car2WayPtVec.y*odom_car2WayPtVec.y+odom_car2WayPtVec.x*odom_car2WayPtVec.x)

        # d,eta = self.get_odom_car2WayPtVec(carPose)

        # self.baseSpeed = 0.3
        # self.baseSpeed = max(self.K * abs(eta) + self.maxSpeed,self.minSpeed)
        print([eta,self.baseSpeed])
        mk = 2*math.sin(eta)/d
        return mk

    def get_odom_car2WayPtVec(self,carPose):
        carPose_pos = Point()
        carPose_pos = carPose.position
        carPose_yaw = self.getYaw(carPose)
        odom_car2WayPtVec = Point()
        myforwardPoint = Point()
        forwardPt = Point()
        # path_yaw = 0
        self.foundForwardPt = False
        if not self.goal_reached:
            path = copy.deepcopy(self.path.poses)
            if len(path)<20:
                dis = self.getCar2GoalDist()
                if dis < self.Lfw:
                    self.foundForwardPt = True
                    forwardPt = self.goal
                    self.baseSpeed= 0.1

            
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
                            # path_yaw = self.getYaw(odom_path_pose.pose)
                            self.foundForwardPt = True

                            # draw the point
                            # myforwardPoint.x = path[30].pose.position.x
                            # myforwardPoint.y = path[30].pose.position.y
                            # myforwardPoint.z = path[30].pose.position.z
                            # self.pub_point.publish(myforwardPoint)
                            break
                except:
                    pass
        elif(self.goal_reached):
            forwardPt = self.odom.pose.pose.position
            self.foundForwardPt = False
        odom_car2WayPtVec.x = math.cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + math.sin(carPose_yaw)*(forwardPt.y - carPose_pos.y)
        odom_car2WayPtVec.y = -math.sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + math.cos(carPose_yaw)*(forwardPt.y - carPose_pos.y)

        # d = math.sqrt(odom_car2WayPtVec.y*odom_car2WayPtVec.y+odom_car2WayPtVec.x*odom_car2WayPtVec.x)
        # diff_angle = path_yaw - carPose_yaw

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

    def PJcurvature(self,x,y):
        """
        input  : the coordinate of the three point
        output : the curvature and norm direction
        refer to https://github.com/Pjer-zhang/PJCurvature for detail
        """
        t_a = LA.norm([x[1]-x[0],y[1]-y[0]])
        t_b = LA.norm([x[2]-x[1],y[2]-y[1]])
        
        M = np.array([
            [1, -t_a, t_a**2],
            [1, 0,    0     ],
            [1,  t_b, t_b**2]
        ])

        a = np.matmul(LA.inv(M),x)
        b = np.matmul(LA.inv(M),y)

        kappa = 2*(a[2]*b[1]-b[2]*a[1])/(a[1]**2.+b[1]**2.)**(1.5)
        return kappa
    
    def vel_adjust(self):
        forwardPt = Point()
        if not self.goal_reached:
            path = copy.deepcopy(self.path.poses)
            # print(len(path))
            # vx :1

            # vx :2
            if len(path)>20:
                a = int(min(self.forwardpath,len(path)-1))
                x = [path[0].pose.position.x,path[int(a/2)].pose.position.x,path[a].pose.position.x]
                y = [path[0].pose.position.y,path[int(a/2)].pose.position.y,path[a].pose.position.y]
                kappa = abs(self.PJcurvature(x,y))
                if kappa > 1.0:
                    self.baseSpeed = self.baseSpeed - self.vscale
                    self.baseSpeed = max(self.baseSpeed, self.minSpeed)
                if kappa < 0.8:
                    self.baseSpeed = self.baseSpeed + self.vscale
                    self.baseSpeed = min(self.baseSpeed, self.maxSpeed)
                print([kappa,self.baseSpeed])



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
                    # self.vel_adjust()
                    self.cmd_vel.linear.x = self.baseSpeed
                    self.cmd_vel.angular.z = eta*self.cmd_vel.linear.x
        self.pub.publish(self.cmd_vel)


        
if __name__ == '__main__':
    track = Tracking()
    rospy.spin()
