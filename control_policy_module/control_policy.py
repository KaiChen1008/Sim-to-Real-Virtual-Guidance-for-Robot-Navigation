from __future__ import print_function
import sys
import rospy
import os 
import numpy as np
from geometry_msgs.msg import Twist
import time

import cv2
import tensorflow as tf 

pre_path = os.path.abspath('../')
sys.path.append(pre_path)
from utils import imagezmq

# ============================== Pretrained Model ==========================
meta_path = './Unitedmap_0906_reload-0/RobotBrain/model-12063494.cptk.meta'
ckpt_path = './Unitedmap_0906_reload-0/RobotBrain/'
# ==========================================================================

config = tf.ConfigProto(log_device_placement=False, allow_soft_placement=True)
config.gpu_options.allow_growth = True

# RL model class
class RL_Model:
	# for testing, linear speed would be set to 0.2, while angular speed set to 0.3 or 0.4186
	def __init__(self, dir, linear_speed=0.2):
		# ------------------------------------------------
		# dir: RL model's directory 
		# linear_speed: linear speed (x-axis) for AGV, default is 0.2
		# ------------------------------------------------
		tf.reset_default_graph()
		self.sess = tf.Session()
		self.saver = tf.train.import_meta_graph(path)
		graph = tf.get_default_graph()
		# get variable by name from tensorflow graph
		self.visual_in = graph.get_tensor_by_name('visual_observation_0:0')
		self.action = graph.get_tensor_by_name('action:0')
		self.action_mask = graph.get_tensor_by_name('action_masks:0') 

		self.action_pub = rospy.Publisher('twitch', Twist, queue_size=1)
		self.linear_speed = linear_speed
		self.angular_speed = angular_speed
		self.move_command = Twist()
		# create mask to enable three action
		self.mask = np.array([[1, 1, 1]])
		self.saver.restore(self.sess, tf.train.latest_checkpoint(ckpt_path))
		
	def restore_and_run(self, img):
		# ----------------------------------------------------
		# img_test: input image from segmentation module
		# ----------------------------------------------------

		# initialize parameters
		self.move_command.angular.z = 0
		self.move_command.linear.x = self.linear_speed

		# for multinomial sampling, using " act = tf.multinomial(self.action, 1) " and revise session in next row
		prob = self.sess.run([self.action], feed_dict = {self.visual_in:img, self.action_mask:self.mask})
		direction = np.argmax(prob)

		# 3-Action
		# Keep moving forward
		if direction == 0 :
			self.move_command.angular.z = 0
		# Turn Left
		elif direction == 1:
			self.move_command.angular.z = 1
		# Turn Right
		elif direction == 2:
			self.move_command.angular.z = -1
		
		# publish Twist
		self.action_pub.publish(self.move_command)
# External class for RL model
class PolicyModel:
	def __init__(self):	
		
		self.RLmodel = RL_Model(meta_path)
		self.last_time = time.time()

	def callback(self,resize_image):
		# --------------------------------------------------
		# resize_image: image received from segmentation module
		# --------------------------------------------------
		self.RLmodel.restore_and_run(resize_image)
		self.last_time = time.time()

	# RL model's test function
	def test(self):
		for i in range(100):
			fake_image = np.zeros((1, 80, 120, 3))
			self.RLmodel.restore_and_run(fake_image)

if __name__ == '__main__':
	rospy.init_node('control_model', anonymous=True)
	hub = imagezmq.ImageHub()
	cm = PolicyModel()

	while True:
		name, image = hub.recv_image() # recieve image
		start = time.time()
		cv2.imshow("Image", image)
		image = [image]
		cm.callback(image) # process the image
		print(time.time()-start)
		hub.send_reply() # get ready for next image
		cv2.waitKey(1)
