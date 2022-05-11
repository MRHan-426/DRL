# -*- coding: utf-8 -*-
'''
Obtain map data and model position
author@ziqi han
'''
import rospy
import cv2
import message_filters
import numpy as np
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates

def callback(mapmsg,posemsg):
  try:
    map = mapmsg.data
    height = mapmsg.info.height
    width = mapmsg.info.width
    print(height,width)
    # name = posemsg.name  
    # The Car is in the third position
    pose_x = posemsg.pose[2].position.x
    pose_y = posemsg.pose[2].position.y

    print "Current Position ",pose_x,pose_y

    mapdata = np.array(map) 
    mapdata = mapdata.reshape((height,width))

    tem = np.zeros((height,width))
    for i in range(height):
      for j in range(width):
        if(mapdata[i,j]==-1):
            tem[i,j]=0
        else:
          #  tem[i,j]=255-map[i,j]
            tem[i,j]=255

    cv2.imwrite('/home/hzq/multirobot_formation/src/formation/maps/(' + str(pose_x) + "," + str(pose_y) + ').jpg', tem)
    print "Img stored successfully"
  except Exception,e:
    print e
    rospy.loginfo('Fital Error')

if __name__=='__main__':
  rospy.init_node('map',anonymous=True)
  map_sub = message_filters.Subscriber("map",OccupancyGrid)
  pose_sub = message_filters.Subscriber("gazebo/model_states",ModelStates)
  ts = message_filters.ApproximateTimeSynchronizer([map_sub, pose_sub], 10, 1, allow_headerless=True)
  ts.registerCallback(callback)
  rospy.spin()
