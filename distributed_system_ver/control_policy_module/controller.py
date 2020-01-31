#!/usr/bin/env python
import sys
import os
import rospy
import roslib
from time import sleep
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from std_msgs.msg import Bool

pre_path = os.path.abspath('../')
sys.path.append(pre_path)
from utils.communication import sender


# Global variable 
speed_ratio_l = 0   # To slow down linear velocity smoothly
speed_ratio_a = 0   # To slow down angular velocity smoothly
pre_linear_v = 0    # Record previous motion
pre_angular_v = 0   # Record previous motion
full_speed = 0.3   # Highest speed


def create_speed(angular_v):
    # ----------------------------------------------
    # angular_v: angular velocity, 1 is right, -1 is left 
    # ----------------------------------------------
    global speed_ratio_a, pre_angular_v, full_speed
    if angular_v:
        if pre_angular_v * angular_v == -1: # Suddenly change of direction leads to speed drop to 0.
            speed_ratio_a = 0
        pre_angular_v = angular_v
        speed_ratio_a = speed_ratio_a + 0.05 if speed_ratio_a < 1 else 1
        return speed_ratio_a * full_speed * angular_v
    else:
        speed_ratio_a = speed_ratio_a - 0.1 if speed_ratio_a >= 0.1 else 0
        return speed_ratio_a * full_speed * pre_angular_v

class controller():
    def __init__(self):
        rospy.init_node('controller')
        self.goal = False
        self.sendman = sender()
        self.twist = Twist()
        self.action_count = 0
        self.plus = 0
        self.minus = 0
        self.j = self.create_zero_twist()

        print('node started')

        rospy.Subscriber("twitch", Twist, self.callback)
        rospy.Subscriber("if_goal", Bool, self.is_goal)
        rospy.Subscriber("Emergency", Bool, self.is_Emergency)
        rospy.spin()
    # receive emergency signal sended by joystick
    def is_Emergency(self, data):
        if data.data == True:
            self.goal = True
            self.j = self.create_zero_twist()
            self.sendman.send(self.j)
            print('Emergency!!')
            a = '0'
            while a != 'y' and  a != 'Y':
                a = input('is ok???: ')
                print(a)
            self.goal = False
    # receive RL model's output and convert it to json format
    def callback(self, t):
        #print('receive t...')
        if (self.goal is False):
            self.j = self.msgs2json(t)
            self.sendman.send(self.j)
            print(self.j)
    # receive signal sended by planner module to check whether AGV reaches goal or not
    def is_goal(self, data):
        if data.data == True:
            self.j = self.create_zero_twist()
            self.sendman.send(self.j)
            self.goal = True
            print('achieve')
            # continue
            a = 'z'
            while a != 'y' or a != 'Y':
                a = input("if next goal exists:")
                
            self.goal = False
    # convert ROS twist to json format (for emergency)
    def create_zero_twist(self):
        j = {
            'linear_x' : 0.0,
            'linear_y' : 0.0,
            'linear_z' : 0.0,
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.0,
        }

        return j
    # convert ROS twist from RL module to json format
    def msgs2json(self, t, smooth=False):
        if smooth == True:
            z = create_speed(t.angular.z)
        else:
            z = t.angular.z * 0.3
        j = {
            'linear_x' : t.linear.x +0.2,
            'linear_y' : t.linear.y,
            'linear_z' : t.linear.z,
            'angular_x': t.angular.x,
            'angular_y': t.angular.y,
            'angular_z': z 
        }
        if t.angular.z > 0.0:
            self.plus = self.plus +1
        if t.angular.z < 0.0:
            self.minus = self.minus +1

        print(str(self.plus) + ',' + str(self.minus))
        return j


if __name__ == '__main__':
    try:
        print('starting controll husky....')
        c = controller()

    except rospy.ROSInterruptException:
        print('ROS error')
        pass
