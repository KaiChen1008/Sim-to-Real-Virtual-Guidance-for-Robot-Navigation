from math               import atan2
from enum               import Enum
from nav_msgs.msg       import Path
from std_msgs.msg       import Bool
from geometry_msgs.msg  import Point, Twist, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import rospy
from basic_def import *
import numpy as np
import cv2
# specify the communication protocol
COM_PROTOCOL = PROTOCOLS.SOCKET
#COM_PROTOCOL = PROTOCOLS.ROS
#import requests
import sender
import imagezmq

REST_PATH_LEN_MIN   = 5
THRESHOLD           = 0.15

class Ball_Generator():
    def __init__(self):
        self.xavier_ip      = '192.168.0.125'
        self.xavier_port    = '6666'
        self.nano_ip        = '192.168.0.198'
        self.nano_port      = '7777'
        self.receiver_setup()
        self.sender_setup()
        self.init_params()

    def init_params(self):
        self.rest_path_len      = 100 # int
        self.temp_goal          = Position()
        self.curr_pose          = Position()
        self.curr_direction     = 0


    def receiver_setup(self):
        self.path_sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)
        self.pose_sub = rospy.Subscriber("/slam_pose", PoseWithCovarianceStamped, self.pose_callback)
    
    def sender_setup(self):
        if (COM_PROTOCOL == PROTOCOLS.ROS):
            self.vel_pub    = rospy.Publisher("/fake_cmd_vel", Twist, queue_size = 1)
            self.goal_pub   = rospy.Publisher("/if_goal"     , Bool,  queue_size=10)
        elif (COM_PROTOCOL == PROTOCOLS.SOCKET):
            #self.vel_pub    = sender.sender(url=vel_url,   port=vel_port)
            #self.goal_pub   = sender.sender(url=goal_url,  port=goal_port)
            self.dir_sender  = imagezmq.ImageSender(connect_to='tcp://%s:%s'%(self.xavier_ip, self.xavier_port))
            self.goal_sender = imagezmq.ImageSender(connect_to='tcp://%s:%s'%(self.nano_ip, self.nano_port))
            print('sender setup')
        else:
            print("ERROR PROTOCOL TYPE")
    
    def pose_callback(self, msg):
        self.curr_pose.x = msg.pose.pose.position.x
        self.curr_pose.y = msg.pose.pose.position.y
        
        curr_rotation = msg.pose.pose.orientation
        
        self.curr_direction = self.get_direction(curr_rotation)
    
    def path_callback(self, msg):
        if len(msg.poses) > 3:
            self.rest_path_len = len(msg.poses)
            self.temp_goal.x = msg.poses[4].pose.position.x
            self.temp_goal.y = msg.poses[4].pose.position.y
            self.spin()
            
            print('rest path point = ' ,self.rest_path_len)


    def get_direction(self, r):
        (roll, pitch, theta) = euler_from_quaternion([r.x, r.y, r.z, r.w])
        return theta
    
    def is_goal(self):
        if self.rest_path_len < REST_PATH_LEN_MIN:
            return 1 # is goal
        else:
            return 0
    
    def send_msg(self, s):
        if (COM_PROTOCOL == PROTOCOLS.ROS):
            # TO DO
            print("ROS-COM")
        elif (COM_PROTOCOL == PROTOCOLS.SOCKET):
            # send direction
            img = np.asarray([[s,0,0],[0,0,0]])#.astype(np.unit8)
            img = img.astype(np.uint8)
            self.dir_sender.send_image("direction",img) 
            print("send direction")
            # send is_goal
            goal_msg = self.is_goal()
            img = np.asarray([[goal_msg,0,0],[0,0,0]])
            self.goal_sender.send_image("goal", img)

        else:
            print("Error COMMUNICATION PROTOCOL")
            img = img.astype(np.uint8)
    
    def spin(self):
        goal_vector = [ self.temp_goal.x - self.curr_pose.x, 
                        self.temp_goal.y - self.curr_pose.y
                    ]
        angle_to_goal = atan2(goal_vector[1], goal_vector[0])

        delta_theta = angle_to_goal - self.curr_direction
        
        print('--------------------------')
        print(delta_theta)

        if  (delta_theta > THRESHOLD and delta_theta > 3.14) or (delta_theta <-THRESHOLD and delta_theta >=-3.14):
            print('right')
            s = 2
        elif(delta_theta > THRESHOLD and delta_theta <=3.14) or (delta_theta <-THRESHOLD and delta_theta <-3.14):
            print('left')
            s = 1
        else:
            print('go straight')
            s = 0
        print('--------------------------')
        
        # send msg
        self.send_msg(s)


if __name__ == '__main__':
    rospy.init_node("fake_local_planner")
    rate = rospy.Rate(4)
    print("fake local planner is on")

    BG = Ball_Generator()

    while not rospy.is_shutdown():
        #BG.spin()
        rate.sleep()
    
    

