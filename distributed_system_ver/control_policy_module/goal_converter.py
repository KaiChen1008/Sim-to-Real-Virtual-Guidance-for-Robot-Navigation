import sys
import os
import rospy
from std_msgs.msg import Bool
import numpy as np
import cv2

pre_path = os.path.abspath('../')
sys.path.append(pre_path)
from utils import imagezmq


if __name__ == '__main__':
    rospy.init_node("cc")
    sub = rospy.Publisher('if_goal', Bool, queue_size=1)
    reciever = imagezmq.ImageHub(open_port='tcp://*:7777')
    print("OK")
    while True:
        #print("Ready")
        name, image = reciever.recv_image()
        #print("Recieved")
        data = image[0][0]
        msg = Bool()
        if data == 1 :
            msg = True
        else :
            msg = False
        print(data)
        sub.publish(msg)
        reciever.send_reply()
            
