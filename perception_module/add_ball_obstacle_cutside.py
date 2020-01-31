import numpy as np
from PIL import Image, ImageDraw
import cv2
class Add_Ball():
    def __init__(self):
        pass
    def initialize(self, seg_image):
    #def __init__(self, seg_image):
        #cut bottem half of picture into 108 (3*6*6) squares
        #calculate points
        height = 80
        width = 120
        self.straight_ball_size = 5
        self.turn_ball_size = 10
        #height = seg_image.shape[0]
        #width = seg_image.shape[1]
        self.gap_y = height/3/4
        self.gap_x = width/3/4

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

        #priority map         left             middle           right
        #00 01 02 03       01 02 05 10      07 02 01 05      10 05 02 01
        #04 05 06 07       03 04 06 11      08 04 03 06      11 06 04 03
        #08 09 10 11       07 08 09 12      15 10 09 13      12 09 08 07
        #12 13 14 15       13 14 15 16      16 12 11 14      16 15 14 13
		#16 17 18 19       17 18 19 20      23 18 17 21      17 18 19 20
        #20 21 22 23 	   21 22 23 24      24 20 19 22      21 22 23 24
		#24 25 26 27       25 26 27 28      31 26 25 29      25 26 27 28
		#28 29 30 31       29 30 31 32      32 28 27 30      29 30 31 32

	#squares to check
        self.p_right = [19,18,22,23,-1,-1,17,21,25,-1,-1,-1,14,15,13,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
        #self.p_middle = [9,10,13,14,17,18,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
        self.p_left =  [16,17,20,21,-1,-1,26,18,22,-1,-1,-1,13,12,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]

    def add_check(self, vector, seg_image):
        #self.right_image = seg_image[:,80:120,:]
        #self.left_image = seg_image[:,0:40,:]

        jitter = 1
        jitter_h = 0
        #check and add square
        if vector==1: #left
            circle_point_x = int((self.left_points[17,0,1]+self.left_points[17,3,1])/2)
            circle_point_y = int((self.left_points[17,0,0]+self.left_points[17,3,0])/2)
            #cv2.circle(seg_image,(circle_point_y-jitter, circle_point_x+jitter_h),self.turn_ball_size, (220, 220, 0), -1)
            #"""
            for num in range(0,16):
                if (self.p_left[num]!=-1):
                    has_obstacle = False
                    to_check = 0
                    """
                    while has_obstacle==False and to_check<=self.gap_x:
                        try : 
                            if ((np.array_equal(seg_image[to_check+self.left_points[self.p_left[num],0,1],circle_point_y,:],[128,64,128]) or np.array_equal(seg_image[to_check+self.left_points[self.p_left[num],0,1],circle_point_y,:],[244,35,232]))) : 
                                to_check = to_check + 1
                            else :
                                has_obstacle = True
                                break
                        except : 
                            break
                        else : 
                            pass
                    """
                    to_check = 0
                    """
                    while has_obstacle==False and to_check<=self.gap_y:
                        try : 
                            if ((np.array_equal(seg_image[circle_point_x,to_check+self.left_points[self.p_left[num],0,0],:],[128,64,128]) or np.array_equal(seg_image[circle_point_x,to_check+self.left_points[self.p_left[num],0,0],:],[244,35,232]))) : 
                                to_check = to_check + 1
                            else :
                                has_obstacle = True
                                break
                        except : 
                            break
                        else : 
                            pass
                    """
                    try : 
                        if ((has_obstacle == False) and
                            (np.array_equal(seg_image[self.left_points[self.p_left[num],0,1],self.left_points[self.p_left[num],0,0],:],[128,64,128]) or np.array_equal(seg_image[self.left_points[self.p_left[num],0,1],self.left_points[self.p_left[num],0,0],:],[244,35,232])) and 
                            (np.array_equal(seg_image[self.left_points[self.p_left[num],1,1],self.left_points[self.p_left[num],1,0],:],[128,64,128]) or np.array_equal(seg_image[self.left_points[self.p_left[num],1,1],self.left_points[self.p_left[num],1,0],:],[244,35,232])) and 
                            (np.array_equal(seg_image[self.left_points[self.p_left[num],2,1],self.left_points[self.p_left[num],2,0],:],[128,64,128]) or np.array_equal(seg_image[self.left_points[self.p_left[num],2,1],self.left_points[self.p_left[num],2,0],:],[244,35,232])) and
                            (np.array_equal(seg_image[self.left_points[self.p_left[num],3,1],self.left_points[self.p_left[num],3,0],:],[128,64,128]) or np.array_equal(seg_image[self.left_points[self.p_left[num],3,1],self.left_points[self.p_left[num],3,0],:],[244,35,232]))) :
                            cv2.circle(seg_image,(circle_point_y-jitter, circle_point_x+jitter_h),self.turn_ball_size, (220, 220, 0), -1)
                            #seg_image[self.left_points[self.p_left[num],0,1]:self.left_points[self.p_left[num],2,1],self.left_points[self.p_left[num],0,0]:self.left_points[self.p_left[num],1,0],:] = [220, 220, 0] 
                            break
                    except :  
                        pass
                    else : 
                        pass
                else : 
                    pass
                    #"""
        elif vector==3: #right
            circle_point_x = int((self.right_points[18,0,1]+self.right_points[18,3,1])/2)
            circle_point_y = int((self.right_points[18,0,0]+self.right_points[18,3,0])/2)
            #cv2.circle(seg_image,(circle_point_y+jitter, circle_point_x+jitter_h), self.turn_ball_size, (220, 220, 0), -1)
            #"""
            for num in range(0,32):
                if (self.p_right[num]!=-1):
                    has_obstacle = False
                    to_check = 0
                    """
                    while has_obstacle==False and to_check<=self.gap_x:
                        try : 
                            if ((np.array_equal(seg_image[to_check+self.right_points[self.p_right[num],0,1],circle_point_y,:],[128,64,128]) or np.array_equal(seg_image[to_check+self.right_points[self.p_right[num],0,1],circle_point_y,:],[244,35,232]))) : 
                                to_check = to_check + 1
                            else :
                                has_obstacle = True
                                break
                        except : 
                            break
                        else : 
                            pass
                    to_check = 0
                    while (has_obstacle==False and to_check<=self.gap_y):
                        try :
                            if ((np.array_equal(seg_image[circle_point_x,to_check+self.right_points[self.p_right[num],0,0],:],[128,64,128]) or np.array_equal(seg_image[circle_point_x,to_check+self.right_points[self.p_right[num],0,0],:],[244,35,232]))) : 
                                to_check = to_check + 1
                            else :
                                has_obstacle = True
                                break
                        except : 
                            break
                        else : 
                            pass
                            """
                    try : 
                        if ((has_obstacle == False) and 
                            (np.array_equal(seg_image[self.right_points[self.p_right[num],0,1],self.right_points[self.p_right[num],0,0],:],[128,64,128]) or np.array_equal(seg_image[self.right_points[self.p_right[num],0,1],self.right_points[self.p_right[num],0,0],:],[244,35,232])) and 
                            (np.array_equal(seg_image[self.right_points[self.p_right[num],1,1],self.right_points[self.p_right[num],1,0],:],[128,64,128]) or np.array_equal(seg_image[self.right_points[self.p_right[num],1,1],self.right_points[self.p_right[num],1,0],:],[244,35,232])) and 
                            (np.array_equal(seg_image[self.right_points[self.p_right[num],2,1],self.right_points[self.p_right[num],2,0],:],[128,64,128]) or np.array_equal(seg_image[self.right_points[self.p_right[num],2,1],self.right_points[self.p_right[num],2,0],:],[244,35,232])) and
                            (np.array_equal(seg_image[self.right_points[self.p_right[num],3,1],self.right_points[self.p_right[num],3,0],:],[128,64,128]) or np.array_equal(seg_image[self.right_points[self.p_right[num],3,1],self.right_points[self.p_right[num],3,0],:],[244,35,232]))) :
                            cv2.circle(seg_image,(circle_point_y+1, circle_point_x-6), self.straight_ball_size, (220, 220, 0), -1)
                            #seg_image[self.right_points[self.p_right[num],0,1]:self.right_points[self.p_right[num],2,1],self.right_points[self.p_right[num],0,0]:self.right_points[self.p_right[num],1,0],:] = [220, 220, 0] 
                            break
                    except : 
                        pass
                    else : 
                        pass
                else : 
                    pass
        elif vector==0: 
            pass
        else:
            #print("no vector provided")
            pass
    
        seg_image = np.array(seg_image) #change to array
        return seg_image
