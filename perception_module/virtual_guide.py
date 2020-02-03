# Lint as: python3
# ==============================================================================
"""Class to draw the virtual guide.
   Cuts the image into 144(12*12) squares
   Dynamically adjusts placement according to user needs. 
"""
import numpy as np
from PIL import Image, ImageDraw
import cv2

class Add_Guide():
    """Adds the virtual guide to the segmentated image.
    The virtual guide here is a 2D yellow ball.
	Args:
	  seg_image: the segmentated image in np.array format.
      vector: the direction given by the planner module. 1 -> left, 3 -> right
	Returns:
	  The segmentated image with the Virtual guide in np.array format.
	Raises:
	  Not receive image or vector: Upon invalid data recieved.
	"""
    def __init__(self):
        pass
    def initialize(self, seg_image):
        """Cuts the image into 144(12*12) squares, and sets the size of the virtual guide"""
        #calculate the four points of each square
        height = seg_image.shape[0]
        width = seg_image.shape[1]

        self.straight_ball_size = 5
        self.turn_ball_size = 10
        self.gap_y = height/3/4
        self.gap_x = width/3/4
        self.jitter_lx = -1
        self.jitter_ly = 0
        self.jitter_rx = 1
        self.jitter_ry = -6

        left_points = np.zeros((32,4,2), dtype=int)
        middle_points = np.zeros((32,4,2), dtype=int)
        right_points = np.zeros((32,4,2), dtype=int)

        for y in range(0,8):
            for x in range(0,4):
                left_points[y*4+x,0,:] = [x*self.gap_x, y*self.gap_y+(height/3-1)]
                left_points[y*4+x,1,:] = [(x+1)*self.gap_x, y*self.gap_y+(height/3-1)]
                left_points[y*4+x,2,:] = [x*self.gap_x, (y+1)*self.gap_y+(height/3-1)]
                left_points[y*4+x,3,:] = [(x+1)*self.gap_x, (y+1)*self.gap_y+(height/3-1)]

                middle_points[y*4+x,0,:] = [x*self.gap_x+(width/3), y*self.gap_y+(height/3-1)]
                middle_points[y*4+x,1,:] = [(x+1)*self.gap_x+(width/3), y*self.gap_y+(height/3-1)]
                middle_points[y*4+x,2,:] = [x*self.gap_x+(width/3), (y+1)*self.gap_y+(height/3-1)]
                middle_points[y*4+x,3,:] = [(x+1)*self.gap_x+(width/3), (y+1)*self.gap_y+(height/3-1)]

                right_points[y*4+x,0,:] = [x*self.gap_x+(width*2/3), y*self.gap_y+(height/3-1)]
                right_points[y*4+x,1,:] = [(x+1)*self.gap_x+(width*2/3), y*self.gap_y+(height/3-1)]
                right_points[y*4+x,2,:] = [x*self.gap_x+(width*2/3), (y+1)*self.gap_y+(height/3-1)]
                right_points[y*4+x,3,:] = [(x+1)*self.gap_x+(width*2/3), (y+1)*self.gap_y+(height/3-1)]
        self.left_points = left_points
        self.middle_points = middle_points
        self.right_points = right_points

        #   left          middle           right
        #00 01 02 03    00 01 02 03     00 01 02 03 
        #04 05 06 07    04 05 06 07     04 05 06 07 
        #08 09 10 11    08 09 10 11     08 09 10 11 
        #12 13 14 15    12 13 14 15     12 13 14 15 
		#16 17 18 19    16 17 18 19     16 17 18 19 
        #20 21 22 23    20 21 22 23     20 21 22 23 
		#28 29 30 31    28 29 30 31     28 29 30 31 

	    #squares to check, the middle points are not used
        self.p_right = [19,18,22,23,-1,-1,17,21,25,-1,-1,-1,14,15,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
        self.p_left =  [16,17,20,21,-1,-1,26,18,22,-1,-1,-1,13,12,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]

    def add_check(self, vector, seg_image):
        """Checks whether obstacles exists, and sets the virtual guide"""

        if vector==1: #left
            #sets the virtual guide
            circle_point_x = int((self.left_points[17,0,1]+self.left_points[17,3,1])/2)
            circle_point_y = int((self.left_points[17,0,0]+self.left_points[17,3,0])/2)

            #checks for obstacles, draw virtual guide
            for num in range(0,16):
                if (self.p_left[num]!=-1):
                    if ((np.array_equal(seg_image[self.left_points[self.p_left[num],0,1],self.left_points[self.p_left[num],0,0],:],[128,64,128]) or np.array_equal(seg_image[self.left_points[self.p_left[num],0,1],self.left_points[self.p_left[num],0,0],:],[244,35,232])) and 
                        (np.array_equal(seg_image[self.left_points[self.p_left[num],1,1],self.left_points[self.p_left[num],1,0],:],[128,64,128]) or np.array_equal(seg_image[self.left_points[self.p_left[num],1,1],self.left_points[self.p_left[num],1,0],:],[244,35,232])) and 
                        (np.array_equal(seg_image[self.left_points[self.p_left[num],2,1],self.left_points[self.p_left[num],2,0],:],[128,64,128]) or np.array_equal(seg_image[self.left_points[self.p_left[num],2,1],self.left_points[self.p_left[num],2,0],:],[244,35,232])) and
                        (np.array_equal(seg_image[self.left_points[self.p_left[num],3,1],self.left_points[self.p_left[num],3,0],:],[128,64,128]) or np.array_equal(seg_image[self.left_points[self.p_left[num],3,1],self.left_points[self.p_left[num],3,0],:],[244,35,232]))) :
                        cv2.circle(seg_image,(circle_point_y+self.jitter_lx, circle_point_x+self.jitter_ly),self.turn_ball_size, (220, 220, 0), -1)
                        break
        
        elif vector==3: #right
            #sets the virtual guide
            circle_point_x = int((self.right_points[18,0,1]+self.right_points[18,3,1])/2)
            circle_point_y = int((self.right_points[18,0,0]+self.right_points[18,3,0])/2)

            #checks for obstacles, draw virtual guide
            for num in range(0,16):
                if (self.p_right[num]!=-1):
                    if ((np.array_equal(seg_image[self.right_points[self.p_right[num],0,1],self.right_points[self.p_right[num],0,0],:],[128,64,128]) or np.array_equal(seg_image[self.right_points[self.p_right[num],0,1],self.right_points[self.p_right[num],0,0],:],[244,35,232])) and 
                        (np.array_equal(seg_image[self.right_points[self.p_right[num],1,1],self.right_points[self.p_right[num],1,0],:],[128,64,128]) or np.array_equal(seg_image[self.right_points[self.p_right[num],1,1],self.right_points[self.p_right[num],1,0],:],[244,35,232])) and 
                        (np.array_equal(seg_image[self.right_points[self.p_right[num],2,1],self.right_points[self.p_right[num],2,0],:],[128,64,128]) or np.array_equal(seg_image[self.right_points[self.p_right[num],2,1],self.right_points[self.p_right[num],2,0],:],[244,35,232])) and
                        (np.array_equal(seg_image[self.right_points[self.p_right[num],3,1],self.right_points[self.p_right[num],3,0],:],[128,64,128]) or np.array_equal(seg_image[self.right_points[self.p_right[num],3,1],self.right_points[self.p_right[num],3,0],:],[244,35,232]))) :
                        cv2.circle(seg_image,(circle_point_y+self.jitter_rx, circle_point_x+self.jitter_ry), self.straight_ball_size, (220, 220, 0), -1)
                        break

        elif vector==0: #straight
            #no virtual guide is needed
            pass
        else:
            print("no vector provided")
            pass
    
        seg_image = np.array(seg_image)
        return seg_image
