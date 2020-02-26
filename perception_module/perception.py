# Lint as: python3
# ==============================================================================
"""Run to start the perception module.
Usage: 
python perception.py -m <model> -ip <input_ip> -io <input_port> -op <output_ip> -oo <output_port>
python perception.py --model=<model> --input_ip=<input_ip> --input_port=<input_port> --output_ip=<output_ip> --output_port=<output_port>
if not specified, runs on default arguments.
A zmq listener listens for images from the camera.
A zmq listener listens for direction vector from the planner
A zmq sender sends the segmentation image to control policy module.
"""
import sys
import os 
import numpy as np
import cv2
import tensorflow as tf 
from PIL import Image
import inference
import virtual_guide
sys.path.append('utils')
import get_dataset_colormap
pre_path = os.path.abspath('../')
sys.path.append(pre_path)
from utils import imagezmq

class zmq_node:
	"""Listens for images from the camera, and direction vector from the planner.
	Upon receiving, the image is segmentated and the virtual guide is added.
	A zmq sender then sends the segmentation image to the control policy module.
	Args:
	  model: i or indoor for indoor model trained on ADE20K dataset
		 o or outdoor for outdoor model trained on Cityscapes datset
	  sender_ip: zmq sender ip to the control policy module
	  sender_port: zmq sender port to the control policy module
	  reciever_ip: zmq listener ip from the camera and planner
	  reciever_port: zmq listener port from the camera and planner
	Returns:
	  Sends a segmentated image with the Virtual guide to specified port and ip.
	Raises:
	  Not receive image or vector: Upon invalid data recieved.
	"""
	def __init__(self,model,sender_ip,sender_port,reciever_ip,reciever_port):
		#configurations for tensorflow
		config = tf.ConfigProto(log_device_placement=False, allow_soft_placement=True)
		config.gpu_options.allow_growth = True

		#sender.
		self.sender = imagezmq.ImageSender(connect_to='tcp://%s:%s'%(sender_ip, sender_port))

		#reciever.
		self.image_hub = imagezmq.ImageHub(open_port='tcp://%s:%s'%(reciever_ip, reciever_port))

		#choose indoor or outdoor model
		if(model=='i' or model=='indoor'):
			self.model = 'i'
		elif(model=='o' or model=='outdoor'):
			self.model = 'o'
		else:
			print('Please choose model "indoor" or "outdoor". Outdoor is chosen on defalt.')
		if(self.model == 'i'):
			self.MODEL = inference.DeepLabModel('models/mobilev2indoor_stride16_90000.tar.gz')
		else:
			self.MODEL = inference.DeepLabModel('models/mobilev2_stride16add153_100000.tar.gz')

		self.first = True
		self.vector = 2
		self.ADD_BALL = virtual_guide.Add_Guide()

	def receive_image(self):
		"""Callback function for receiving data.
		All data recieved in this function.
		"""
		self.rpi_name, self.package = self.image_hub.recv_image()
		self.image_hub.send_reply(b'OK')

		#check the package name of data
		if self.rpi_name=="zed_image":
			self.image = self.package
			self.image_callback()
		elif self.rpi_name=="direction":
			self.angular = self.package
			self.vector_callback()
		else:
			print("Not receive image or vector.")

	def vector_callback(self):
		"""Vector callback function for directional data from the Planner module
		"""
		v = self.angular[0][0]
		#straight
		if v == 0:
			self.vector = 2
		#left
		elif v == 1:
			self.vector = 1
		#right
		elif v == 2:
			self.vector = 3
		#no direction specified
		else:
			self.vector = 0
					 

	def image_callback(self):
		"""Image callback function for directional data from the Camera
		"""
		#run the segmentation model
		image = Image.fromarray(self.image,'RGB')
		resized_im, seg_map = self.MODEL.run(image)
		resized_im = np.array(resized_im)

		#get colormap for specified model
		if(self.model =='o'):
			seg_image = get_dataset_colormap.label_to_color_image_rl(seg_map,'cityscapes').astype(np.uint8)
		else:
			seg_image = get_dataset_colormap.label_to_color_image_rl(seg_map,'ade20k').astype(np.uint8)

		#get size and initialize
		if self.first==True:
			self.ADD_BALL.initialize(seg_image)
			self.first = False

		#for debug keyboard input (wad)
			key = cv2.waitKey(33)
			if key == 119: 
				#print("front")
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

		#add virtual guide
		seg_image = self.ADD_BALL.add_check(self.vector,seg_image)
		self.sender.send_image("seg_image", seg_image)
		
		#for visualization 
		resized_im = cv2.cvtColor(resized_im, cv2.COLOR_BGR2RGB) #change to brg color space
		color_and_mask = cv2.addWeighted(resized_im, 0.3, seg_image, 0.7, 0.0) #overlayed image
		color_and_mask = cv2.resize(color_and_mask, (360,240), interpolation=cv2.INTER_AREA) #make larger
		cv2.imshow('frame', color_and_mask)


def main(argv):
	model = "outdoor"
	input_ip = '*'
	input_port = '6666'
	output_ip = '192.168.0.198'
	output_port = '5555'
	try:
		opts, args = getopt.getopt(argv,"-h-m:-ip:-io:-op:-oo:",["help","model=","input_ip=","input_port=","output_ip=","output_port="])
	except getopt.GetoptError:
		print('GetoptError, usage: perception.py --m <model> -ip <input_ip> -io <input_port> -op <output_ip> -oo <output_port>')
		sys.exit(2)
	for opt, arg in opts:
		if opt in ('-h','--help'):
			print("Run to start the perception module.\n\
Usage: perception.py -m <model> -ip <input_ip> -io <input_port> -op <output_ip> -oo <output_port>\n\
or perception.py --model=<model> --input_ip=<input_ip> --input_port=<input_port> --output_ip=<output_ip> --output_port=<output_port>\n\
Arguments: o for outdoor model, i for indoor model.")
			sys.exit()
		elif opt in ('-m','--model'):
			if opt in ('o','i','outdoor','indoor'):
				model = arg
			else:
				print('Please chose model "indoor" or "outdoor". Outdoor is chosen on defalt.')
				sys.exit()
		elif opt in ('-ip','--input_ip'):
			input_ip = arg
		elif opt in ('-io','--input_port'):
			input_port = arg
		elif opt in ('-oi','--output_ip'):
			output_ip = arg
		elif opt in ('-oo','--output_port'):
			output_port = arg
	print("model: ",model, " input_ip: ", input_ip, " input_port: ", input_port, " output_ip: ", output_ip, " output_port: ", output_port)
	server = zmq_node(model,output_ip,output_port,input_ip,input_port)
	while True:
		server.receive_image()
	
if __name__ == '__main__':
	main(sys.argv[1:])