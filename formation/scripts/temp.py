
import rospy
import os
import numpy as np
import math
import time
import cv2
import tf
from std_msgs.msg import Float32
from actionlib_msgs.msg import GoalStatusArray
from gazebo_msgs.msg import ModelState,ModelStates,ContactsState
from move_base_msgs.msg import MoveBaseActionResult,MoveBaseGoal,MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Twist
from nav_msgs.srv import GetMap, GetMapRequest
from nav_msgs.msg import Odometry

from std_srvs.srv import Empty,EmptyRequest



# def collision_front_callback(msg):
#     if msg.states != []:
#         print("BumpFRONT FRONT FRONT")
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)

# def collision_back_callback(msg):
#     if msg.states != []:
#         print("BumpBACK BACK")
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)
#         print(msg.states[0].total_wrench.force.x)

# if __name__=='__main__':
rospy.init_node('gazebo_control_node', anonymous=True)

#     rospy.Subscriber("front/bumper_states", ContactsState, collision_front_callback)
#     rospy.Subscriber("back/bumper_states", ContactsState, collision_back_callback)
#     rospy.spin()

os.system("rosnode kill /move_base")
vel_reset = rospy.Publisher("cmd_vel", Twist,queue_size = 1)
t_2 = time.time() + 2
while(time.time() <= t_2):
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    vel_reset.publish(msg) 

gazebo_reset = rospy.ServiceProxy("/gazebo/reset_world",Empty)
gazebo_reset.wait_for_service()
gazebo_reset_request = EmptyRequest()
gazebo_reset.call(gazebo_reset_request)
os.system("rosnode kill /slam_karto")
rospy.sleep(0.2)
print("Gazebo Env has been reset!") 