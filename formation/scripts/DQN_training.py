# -*- coding: utf-8 -*-
"""
Train

@author: ziqihan
"""

import numpy as np
import time
import os
import cv2

from DQN_model import DQN
from gazebo_env import gazebo_env
import tensorflow.compat.v1 as tf
# os.environ['CUDA_VISIBLE_DEVICES'] = "0"

DQN_model_path = "../model/model_gpu"
DQN_log_path = "../model/logs_gpu/"
# WIDTH = 22
# HEIGHT = 144

EPISODES = 10000
big_BATCH_SIZE = 16
UPDATE_STEP = 50
# times that evaluate the network
num_step = 0
# used to save log graph
target_step = 0
# used to update target Q network
paused = True
# used to stop training

if __name__ == '__main__':
    # Gazebo Init
    gz_env = gazebo_env()
    # WIDTH, HEIGHT = gz_env.get_map_size()
    gz_env.get_origin_xy()
    WIDTH = 324
    HEIGHT =164
    print("本次训练的图片大小为:",HEIGHT,WIDTH)
    action_size = WIDTH * HEIGHT

    # DQN Init
    agent = DQN(WIDTH, HEIGHT, action_size, DQN_model_path, DQN_log_path)
    for episode in range(EPISODES):
        print("##########",episode,"############")
        station = gz_env.get_map()
        # change graph to WIDTH * HEIGHT for station input
        target_step = 0
        # used to update target Q network
        done = 0
        total_reward = 0
        last_time = time.time()
        accident = 0
        x = 0
        while True:
            gz_env.goal_target = 0         
            station = np.array(station).reshape(-1,HEIGHT,WIDTH,1)[0]
            # reshape station for tf input placeholder
            print('loop took {} seconds'.format(time.time()-last_time))
            last_time = time.time()
            target_step += 1
            # get the action by state
            action = agent.Choose_Action(station)
            print("Output Action is :",action)
            feedback = gz_env.take_action(action,WIDTH)

            # target point is in unknown zone
            if(feedback == 0):
                print("target point is in unknown Zone or Occupied")

            else:
                # wait for agent to get to target point
                print("target point has been sent")
                time_start=time.time()
                while(gz_env.goal_target != 1):
                    time_end=time.time()

                    if gz_env.goal_target == 2:
                        # plan always failed 
                        print("target cannot reach, navigation failed")
                        x = 1
                        break
                    elif (time_end - time_start) >= 50:
                        print("move_base ERROR Accidentally")
                        accident = 1
                        x = 1
                        break
            next_station = gz_env.get_map()
            next_station = np.array(next_station).reshape(-1,HEIGHT,WIDTH,1)[0]
            if accident == 1:
                reward = -1
                done = 1
                accident = 0
            else:
                # get action reward
                reward, done = gz_env.get_reward()

            if done == 1:
                next_station = station
            agent.Store_Data(station, action, reward, next_station, done)
            if len(agent.replay_buffer) > big_BATCH_SIZE:
                num_step += 1
                # save loss graph
                # print('train')
                agent.Train_Network(big_BATCH_SIZE, num_step)
            if target_step % UPDATE_STEP == 0:
                agent.Update_Target_Network()
                # update target Q network
            station = next_station
            total_reward += reward
            if done == 1 or done == 2:
                break
        if episode % 10 == 0:
            agent.save_model()
            # save model
        print('episode: ', episode, 'Evaluation Total Reward:', total_reward)

        if x == 1:
            gz_env.reset_all()
            x = 0
        else:
            gz_env.reset_karto()

        # wait for topic to change
        time.sleep(2)
            
            
            
            
            
        
        
    
    