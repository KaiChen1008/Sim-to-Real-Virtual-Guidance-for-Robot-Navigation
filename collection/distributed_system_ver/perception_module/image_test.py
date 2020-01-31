import os
import tarfile
import tempfile
import numpy as np
from PIL import Image, ImageDraw
import sys
import cv2
import tensorflow as tf
import time
#for color ball
import add_ball
# Needed to show segmentation colormap labels
sys.path.append('utils')
import get_dataset_colormap
#zed camera
sys.path.append('ZED')
from zedcv2 import Camera
#ros
#from sensor_msgs.msg import Image as ROS_image
#import rospy
#from std_msgs.msg import String
#from cv_bridge import CvBridge, CvBridgeError
#zed subscribe
#import zednode_sub

config = tf.ConfigProto(log_device_placement=False, allow_soft_placement=True)
config.gpu_options.allow_growth = True

class DeepLabModel(object):
  """Class to load deeplab model and run inference."""

  INPUT_TENSOR_NAME = 'ImageTensor:0'
  OUTPUT_TENSOR_NAME = 'SemanticPredictions:0'
  INPUT_SIZE = 321 #513
  FROZEN_GRAPH_NAME = 'frozen_inference_graph'

  def __init__(self, tarball_path):
    """Creates and loads pretrained deeplab model."""
    self.graph = tf.Graph()

    graph_def = None
    # Extract frozen graph from tar archive.
    tar_file = tarfile.open(tarball_path)
    for tar_info in tar_file.getmembers():
      if self.FROZEN_GRAPH_NAME in os.path.basename(tar_info.name):
        file_handle = tar_file.extractfile(tar_info)
        graph_def = tf.GraphDef.FromString(file_handle.read())
        break

    tar_file.close()
    if graph_def is None:
      raise RuntimeError('Cannot find inference graph in tar archive.')

    with self.graph.as_default():
      tf.import_graph_def(graph_def, name='')
    self.sess = tf.Session(config = config, graph=self.graph)

  def run(self, image):
    width, height = image.size
    resize_ratio = 1.0 * self.INPUT_SIZE / max(width, height)
    target_size = (int(resize_ratio * width), int(resize_ratio * height))
    resized_image = image.convert('RGB').resize(target_size, Image.ANTIALIAS)

    global start_t
    start_t = time.time()
    #print(resized_image.size)

    batch_seg_map = self.sess.run(
        self.OUTPUT_TENSOR_NAME,
        feed_dict={self.INPUT_TENSOR_NAME: [np.asarray(resized_image)]})
    seg_map = batch_seg_map[0]
    return resized_image, seg_map


def create_pascal_label_colormap():
  colormap = np.zeros((256, 3), dtype=int)
  ind = np.arange(256, dtype=int)

  for shift in reversed(range(8)):
    for channel in range(3):
      colormap[:, channel] |= ((ind >> channel) & 1) << shift
    ind >>= 3

  return colormap


def label_to_color_image(label):

  if label.ndim != 2:
    raise ValueError('Expect 2-D input label')
  colormap = create_pascal_label_colormap()
  if np.max(label) >= len(colormap):
    raise ValueError('label value too large.')
  return colormap[label]

LABEL_NAMES = np.asarray([
    'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus',
    'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike',
    'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tv'
])

FULL_LABEL_MAP = np.arange(len(LABEL_NAMES)).reshape(len(LABEL_NAMES), 1)
FULL_COLOR_MAP = label_to_color_image(FULL_LABEL_MAP)

def image_inference(model_name, picure, label_color, output_name, output_name_color):
  #run single image on model
  MODEL = DeepLabModel(model_name) 
  print('model loaded successfully!')
  
  image = Image.open(picure)
  resized_im, seg_map = MODEL.run(image)

  #color, original, overlay in numpy array form
  seg_image = get_dataset_colormap.label_to_color_image(seg_map,label_color).astype(np.uint8)
  resized_im = np.array(resized_im)
  color_and_mask = cv2.addWeighted(resized_im, 0.4, seg_image, 0.6, 0.0)
  
  #chage to image form
  resized_im = Image.fromarray(resized_im, 'RGB')
  seg_image = Image.fromarray(seg_image, 'RGB')
  color_and_mask = Image.fromarray(color_and_mask, 'RGB')
  
  #time calculation
  global start_t
  duration = time.time()-start_t
  print(duration)
  duration = format(duration,'.4f')

  #save image
  seg_image.save('static/'+output_name, format='PNG')
  color_and_mask.save('static/'+output_name_color, format='PNG')

  return duration

def video_inference(model_name, picture, label_color, output_name):
  #run video on model
  elapsed_times = []
  MODEL = DeepLabModel(model_name) 
  print('model loaded successfully!')
  
  cap = cv2.VideoCapture(picture)
  
  #for save video
  fps = cap.get(cv2.CAP_PROP_FPS)
  fourcc = cv2.VideoWriter_fourcc(*'MP4V')
  out = cv2.VideoWriter( output_name+'.mp4',fourcc, fps, (321,180))

  while (cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
      #array to cv2
      image = Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
      #run model
      resized_im, seg_map = MODEL.run(image)

      #color, original, overlay in numpy array form
      seg_image = get_dataset_colormap.label_to_color_image(seg_map,label_color).astype(np.uint8) #color image
      resized_im = np.array(resized_im) #resized original
      color_and_mask = cv2.addWeighted(resized_im, 0.4, seg_image, 0.6, 0.0) #overlayed image
 
      #time calculation
      global start_t
      duration = time.time()-start_t
      elapsed_times.append(duration)
      
      #show frame
      cv2.imshow('frame', seg_image)
      
      #save as video
      vid = cv2.cvtColor(color_and_mask, cv2.COLOR_RGB2BGR)
      out.write(vid)
      if cv2.waitKey(25) & 0xFF == ord('q'):
          break
    else:
      break
  cap.release()
  #out.release()
  print("video_end")
  print('Average time: {:.4f}, about {:.6f} fps'.format(np.mean(elapsed_times), 1/np.mean(elapsed_times)))

def video_inference_withball(model_name, picture, label_color, output_name):
  #run video on model with ball
  elapsed_times = []
  MODEL = DeepLabModel(model_name) 
  print('model loaded successfully!')

  cap = cv2.VideoCapture(picture)
  
  #for save video
  fps = cap.get(cv2.CAP_PROP_FPS)
  fourcc = cv2.VideoWriter_fourcc(*'MP4V')
  out = cv2.VideoWriter( output_name+'.mp4',fourcc, fps, (321,180)) #change by hand dimention
  
  first = True
  vector = 2
  while (cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
      #array to cv2
      image = Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
      #run model
      resized_im, seg_map = MODEL.run(image)
      
      #color, original, overlay in numpy array form
      seg_image = get_dataset_colormap.label_to_color_image(seg_map,label_color).astype(np.uint8) #color image
      resized_im = np.array(resized_im) #resized original

      #get size and initialize
      if first==True:
        ADD_BALL = add_ball.Add_Ball(seg_image)
        first = False
      
      #for keyboard input
      key = cv2.waitKey(33)
      if key == 119:
        #print("up")
        vector = 2
      elif key == 97:
        #print("left")
        vector = 1
      elif key == 100:
        #print("right")
        vector = 3
      else :
        pass

      #for draw ball 
      seg_image = ADD_BALL.add_check(vector,seg_image)
      color_and_mask = cv2.addWeighted(resized_im, 0.4, seg_image, 0.6, 0.0) #overlayed image

      #time calculation
      global start_t
      duration = time.time()-start_t
      elapsed_times.append(duration)
      
      #show frame
      cv2.imshow('frame', seg_image)
      
      #save as video
      vid = cv2.cvtColor(color_and_mask, cv2.COLOR_RGB2BGR)
      out.write(vid)
      if cv2.waitKey(25) & 0xFF == ord('q'):
          break
    else:
      break
  cap.release()
  out.release()
  print("video_end")
  print('Average time: {:.4f}, about {:.6f} fps'.format(np.mean(elapsed_times), 1/np.mean(elapsed_times)))


def image_inference_withball(model_name, picure, label_color, output_name_color):
  #run single image on model
  MODEL = DeepLabModel(model_name) 
  print('model loaded successfully!')
  
  image = Image.open(picure)
  print(type(image))
  resized_im, seg_map = MODEL.run(image)

  #color, original, overlay in numpy array form
  seg_image = get_dataset_colormap.label_to_color_image(seg_map,label_color).astype(np.uint8)
  resized_im = np.array(resized_im)

  #for draw ball 
  ADD_BALL = add_ball.Add_Ball(seg_image)
  seg_image = ADD_BALL.add_check(1,seg_image)
  color_and_mask = cv2.addWeighted(resized_im, 0.4, seg_image, 0.6, 0.0) #overlayed image
  
  #chage to image form
  resized_im = Image.fromarray(resized_im, 'RGB')
  seg_image = Image.fromarray(seg_image, 'RGB')
  color_and_mask = Image.fromarray(color_and_mask, 'RGB')
  
  #time calculation
  global start_t
  duration = time.time()-start_t
  print(duration)
  duration = format(duration,'.4f')

  #save image
  color_and_mask.save(output_name_color+'.png', format='PNG')

  return duration

def zed_inference_withball(model_name , label_color, output_name):
  #ros test
  rospy.init_node('image_converter', anonymous=True)
  #rospy.Rate(10) # 10hz
  pub = rospy.Publisher("chatter",ROS_image)
  #run video on model with ball
  elapsed_times = []
  MODEL = DeepLabModel(model_name) 
  #print('model loaded successfully!')

  #cap = cv2.VideoCapture(picture)
  zed = Camera()
  zed.set_resolution("720p")    # "2K", "1080p", "720p", "WVGA"
  zed.set_eye("left")         # "left", "right", "both"
  frame_rate = 3
  prev = 0

  #for save video
  #fps = cap.get(cv2.CAP_PROP_FPS)
  #fps = 20
  #fourcc = cv2.VideoWriter_fourcc(*'MP4V')
  #out = cv2.VideoWriter( output_name+'.mp4',fourcc, fps, (321,180)) #change by hand dimention
  
  first = True
  vector = 2
  while not rospy.is_shutdown():

    #array to cv2
    time_elapsed = time.time() - prev
    frame = zed.retrieve_image()
    if time_elapsed > 1./frame_rate:
      #time calculation
      #global start_t
      start_t = time.time()
      prev = time.time()
      
      image = Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
      #run model
      resized_im, seg_map = MODEL.run(image)
      
      #color, original, overlay in numpy .array form
      seg_image = get_dataset_colormap.label_to_color_image_rl(seg_map,label_color).astype(np.uint8) #color image

      #for demo
      seg_original = get_dataset_colormap.label_to_color_image(seg_map,label_color).astype(np.uint8) #color image

      resized_im = np.array(resized_im) #resized original
      #get size and initialize
      if first==True:
        ADD_BALL = add_ball.Add_Ball(seg_image)
        first = False

      #for keyboard input
      key = cv2.waitKey(33)
      if key == 119:
        #print("up")
        vector = 2
      elif key == 97:
        #print("left")
        vector = 1
      elif key == 100:
        #print("right")
        vector = 3
      else :
        pass
      #for draw ball 
      seg_image = ADD_BALL.add_check(vector,seg_image)
      color_and_mask = cv2.addWeighted(resized_im, 0, seg_image, 1.0, 0.0) #overlayed image

      #for demo
      #seg_original = ADD_BALL.add_check(vector,seg_original)
      #color_and_mask_original = cv2.addWeighted(resized_im, 0.3, seg_original, 0.7, 0.0) #overlayed image
      
      #show frame
      #cv2.imshow('frame', color_and_mask_original)
      
      #save as video
      vid = cv2.cvtColor(color_and_mask, cv2.COLOR_RGB2BGR)
      #out.write(vid)

      #ros test
      CV = CvBridge()
      image_message = CV.cv2_to_imgmsg(vid, encoding="passthrough")
      rospy.loginfo(image_message)
      pub.publish(image_message)
      
      #time calculation
      #global start_t
      duration = time.time()-start_t
      elapsed_times.append(duration)

      if cv2.waitKey(25) & 0xFF == ord('q'):
        break
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
  #cap.release()
  #out.release()
  print("video_end")
  print('Average time: {:.4f}, about {:.6f} fps'.format(np.mean(elapsed_times), 1/np.mean(elapsed_times)))


