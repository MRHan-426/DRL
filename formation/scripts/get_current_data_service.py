# -*- coding: utf-8 -*-
'''
Obtain map data and model position
author@ziqi han
'''
import rospy
import cv2
import time
import message_filters
import numpy as np
from nav_msgs.srv import GetMap, GetMapRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

class get_current_data():
    def __init__(self):
        rospy.init_node('get_map',anonymous=True)

    def get_map_data(self):
        get_map_data = rospy.ServiceProxy("dynamic_map",GetMap)
        get_map_data.wait_for_service()
        map_data_request = GetMapRequest()
        map_data_response = get_map_data.call(map_data_request)

        # x,y position
        # get_position = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
        # get_position.wait_for_service()
        # get_position_request = GetModelStateRequest()
        # get_position_request.model_name = "ares1"
        # get_position_response = get_position.call(get_position_request)
        # pose_x = get_position_response.pose.position.x
        # pose_y = get_position_response.pose.position.y
        # print("Position",pose_x,pose_y)

        # map data process
        height = map_data_response.map.info.height
        width = map_data_response.map.info.width
        map_data = np.array(map_data_response.map.data)
        map_data = map_data.reshape((height,width))


        selfheight = 300
        selfwidth = 100
        temp = np.zeros((selfheight,selfwidth))

        if height < selfheight:
            for i in range(height,selfheight):
                for j in range(min(width,selfwidth)):
                    temp[i,j] = 100
        if width < selfwidth:
            for i in range(min(height,selfheight)):
                for j in range(width,selfwidth):
                    temp[i,j] = 100
        if height < selfheight and width < selfwidth:
            for i in range(height,selfheight):
                for j in range(width,selfwidth):
                    temp[i,j] = 100

        for i in range(min(height,selfheight)):
            for j in range(min(width,selfwidth)):
                temp[i,j] = temp[i,j] + map_data[i,j]
        map_data = temp

        for i in range(selfheight):
            for j in range(selfwidth):
                # unknown zone
                if(map_data[i,j] == -1):
                    temp[i,j] = 100
                
                elif(map_data[i,j] == 100):
                    temp[i,j] = 0

                else:
                    temp[i,j] = 255
                
                

        cv2.imwrite('/home/hanziqi/even_ws/src/formation/maps/' + str(time.time()) + '.jpg', temp)
        # print "Img stored successfully"
        return map_data

if __name__=='__main__':
    get_map = get_current_data()
    map_data = get_map.get_map_data()




    # get_map_data = rospy.ServiceProxy("dynamic_map",GetMap)
    #     get_map_data.wait_for_service()
    #     map_data_request = GetMapRequest()
    #     map_data_response = get_map_data.call(map_data_request)
    #     self.map_data = np.array(map_data_response.map.data)
    #     # map data process
    #     height = map_data_response.map.info.height
    #     width = map_data_response.map.info.width
    #     self.map_data = self.map_data.reshape((height,width))
    #     temp = np.zeros((self.height,self.width))

    #     if height < self.height:
    #         for i in range(height,self.height):
    #             for j in range(min(width,self.width)):
    #                 temp[i,j] = 100
    #     if width < self.width:
    #         for i in range(min(height,self.height)):
    #             for j in range(width,self.width):
    #                 temp[i,j] = 100
    #     if height < self.height and width < self.width:
    #         for i in range(height,self.height):
    #             for j in range(width,self.width):
    #                 temp[i,j] = 100

    #     for i in range(min(height,self.height)):
    #         for j in range(min(width,self.width)):
    #             temp[i,j] = temp[i,j] + self.map_data[i,j]
    #     self.map_data = temp
    #     return self.map_data
