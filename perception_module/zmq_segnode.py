from __future__ import print_function
import sys
import os 
import numpy as np
from time import sleep
import json
#for color ball
import time
sys.path.append('utils')
import get_dataset_colormap
import cv2
import image_test
import tensorflow as tf 
from PIL import Image as Image_pil
import add_ball_obstacle_cutside as add_ball
from matplotlib import pyplot as plt
from matplotlib import cm
pre_path = os.path.abspath('../')
sys.path.append(pre_path)
from utils import imagezmq


#change to cv2 image, then list etc

class image_converter:

	def __init__(self,model):
		#print('start listener')
	
		config = tf.ConfigProto(log_device_placement=False, allow_soft_placement=True)
		config.gpu_options.allow_growth = True

		self.ip = '192.168.0.198'
		self.port = '5555'
		self.sender = imagezmq.ImageSender(connect_to='tcp://%s:%s'%(self.ip, self.port))
		if(model=='i' or model=='indoor'):
			self.model = 'i'
		elif(model=='o' or model=='outdoor'):
			self.model = 'o'
		else:
			print('Please input "indoor" or "outdoor".')
		#model

		if(self.model == 'i'):
			self.MODEL = image_test.DeepLabModel('models/mobilev2indoor_stride16_90000.tar.gz')
		else:
			self.MODEL = image_test.DeepLabModel('models/mobilev2_stride16add153_100000.tar.gz')
		self.elapsed_times = []
		self.first = True
		self.vector = 2
		self.ADD_BALL = add_ball.Add_Ball()
		self.frame_rate = 5
		self.image_hub = imagezmq.ImageHub(open_port='tcp://*:6666')
		self.prev = 0

	def receive_image(self):
		#print("Receive")
		self.rpi_name, self.package = self.image_hub.recv_image()
		self.image_hub.send_reply(b'OK')

		if self.rpi_name=="zed_image":
			#print("image")
			self.image = self.package
			self.image_callback()
		elif self.rpi_name=="direction":
			#print("vector")
			self.angular = self.package
			self.vector_callback()
		else:
			print("Not receive image or vector.")

	def vector_callback(self):
		v = self.angular[0][0]
		#print(v)
		if v == 0:#straight
			self.vector = 2
		elif v == 1: #left
			self.vector = 1
		elif v == 2: #right
			self.vector = 3
		else:
			self.vector = 0

	def image_callback(self):
				
				image = Image_pil.fromarray(self.image,'RGB')
				resized_im, seg_map = self.MODEL.run(image)
				#color, original, overlay in numpy .array form
				if(self.model =='o'):
					seg_image = get_dataset_colormap.label_to_color_image_rl(seg_map,'cityscapes').astype(np.uint8) #color image
				else:
					seg_image = get_dataset_colormap.label_to_color_image_rl(seg_map,'ade20k').astype(np.uint8) #color image
				resized_im = np.array(resized_im) #resized original

				#get size and initialize
				if self.first==True:
						self.ADD_BALL.initialize(seg_image)
						self.first = False

		#for keyboard input
				key = cv2.waitKey(33)
				if key == 119:
						#print("up")
						self.vector = 2
				elif key == 97:
						#print("left")
						self.vector = 1
				elif key == 100:
						#print("right")
						self.vector = 3
				elif key == 27:
						self.vector = 0
				else :
						pass


		#resize
				dim = (120, 80)
				seg_image = cv2.resize(seg_image, dim, interpolation=cv2.INTER_AREA)
				resized_im = cv2.resize(resized_im, dim, interpolation=cv2.INTER_AREA)

		#for draw ball
				seg_image = self.ADD_BALL.add_check(self.vector,seg_image)
				self.sender.send_image("seg_image", seg_image)
		#for demo
				resized_im = cv2.cvtColor(resized_im, cv2.COLOR_BGR2RGB) #change to brg color space
				color_and_mask = cv2.addWeighted(resized_im, 0.3, seg_image, 0.7, 0.0) #overlayed image
				color_and_mask = cv2.resize(color_and_mask, (360,240), interpolation=cv2.INTER_AREA)
				cv2.imshow('frame', color_and_mask)


def main(args):
	server = image_converter(args[1])
	while True:
		server.receive_image()
	

if __name__ == '__main__':
	main(sys.argv)
