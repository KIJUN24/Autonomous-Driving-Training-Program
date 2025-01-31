#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os

class MovingAverage:

    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]

    def get_mm(self):
        return float(sum(self.data)) / len(self.data)

    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
prev_r_mv = MovingAverage(15)
prev_l_mv = MovingAverage(15)
prev_flag = False
bridge = CvBridge()
motor = None
mv = MovingAverage(30)  #define mv variable

# camera image topic callback
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)

# drive car with hough transformation
def start():

    global image
    global prev_flag
    global motor
	
    rospy.init_node('my_hough_drive')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print( "---------- Xycar ----------" )
        
    while not rospy.is_shutdown():

        while not image.size == (640*480*3):
            continue

        img = image.copy()

        # ====================
        #  Find line position
        # ====================       
	
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(5, 5), 0) 
        edge_img = cv2.Canny(np.uint8(blur_gray), 40, 75)  #60 ==> 40

        #cv2.imshow("original", img)
        #cv2.imshow("gray", gray)
        #cv2.imshow("gaussian blur", blur_gray)
        #cv2.imshow("edge", edge_img)
        #cv2.waitKey(1)
        #cv2.destroyAllWindows()

        # HoughLinesP
        all_lines = cv2.HoughLinesP(edge_img, 1, math.pi/180,30,30,10)
        print("Number of lines : %d" % len(all_lines))
        # print(all_lines)

        # draw lines
        line_img = img.copy()
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1), (x2, y2), (0,255,0), 2)

        #cv2.imshow("lines", line_img)
        #cv2.waitKey(1)

        # ROI Area
        roi_edge_img = edge_img[240:480, 0:640]
        #cv2.imshow("roi_edge_img", roi_edge_img)

        # HoughLinesP
        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180,50,50,20)
        print("After ROI, number of lines : %d" % len(all_lines))

        # draw lines in ROI area
        line_img = img.copy()
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1+240), (x2, y2+240), (0, 255, 0), 2)

        #cv2.imshow("roi_lines", line_img)
        #cv2.waitKey(1)

        # Change parameters in HoughLinesP
        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180,30,30,10)
        print("Other Hough, Number of lines : %d" % len(all_lines))

        # draw lines
        line_img = img.copy()
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1+240), (x2, y2+240), (0, 255, 0), 2)

        #cv2.imshow("parameter_change_lines", line_img)
        #cv2.waitKey(1)
		
        # calculate slope and do filtering  
        slopes = []
        new_lines = []

        for line in all_lines:
            x1, y1, x2, y2 = line[0]

            if (x2 - x1) == 0:
                slope = 0
            else:
                slope = float(y2-y1) / float(x2-x1)
        
            if 0.1 < abs(slope):
                slopes.append(slope)
                new_lines.append(line[0])

        print("Number of lines after slope filtering : %d" % len(new_lines))

        # divide lines left and right
        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = new_lines[j]
            slope = slopes[j]

            x1, y1, x2, y2 = Line

            if (slope < 0) and (x2 < 320):
                left_lines.append([Line.tolist()])

            elif (slope > 0) and (x1 > 320):
                right_lines.append([Line.tolist()])

        print("Number of left lines : %d" % len(left_lines))
        print("Number of right lines : %d" % len(right_lines))

        # draw right&left lines in different color
        line_img = img.copy()
		
        for line in left_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1+240), (x2, y2+240), (0, 0, 255), 2)

        for line in right_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1+240), (x2, y2+240), (0, 255, 255), 2)

        #cv2.imshow("left & right lines", line_img)
        #cv2.waitKey(1)

        # get average left-line
        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
        m_left, b_left = 0.0, 0.0
        size = len(left_lines)

        for line in left_lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        if(size != 0):
            x_avg = x_sum / (size * 2)
            y_avg = y_sum / (size * 2)
            m_left = m_sum / size
            b_left = y_avg - m_left * x_avg 

        if size == 0:
            x1 = 0
            x2 = 0
        else:
            x1 = int((0.0 - b_left) / m_left)
            x2 = int((240.0 - b_left) / m_left)

        cv2.line(line_img, (x1, 0+240), (x2, 240+240), (255, 0, 0), 2)

        # get average right-line
        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
        m_right, b_right = 0.0, 0.0
        size = len(right_lines)

        for line in right_lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        if(size != 0):
            x_avg = x_sum / (size * 2)
            y_avg = y_sum / (size * 2)
            m_right = m_sum / size
            b_right = y_avg - m_right * x_avg

        if size == 0:
            x1 = 640
            x2 = 640
        else:
            x1 = int((0.0 - b_right) / m_right)
            x2 = int((240.0 - b_right) / m_right)

        cv2.line(line_img, (x1, 0+240), (x2, 240+240), (255, 0, 0), 2)

        cv2.imshow("detected lines", line_img)
        #cv2.waitKey(1)

        # get left/right line positions

        # y = m * x + b
        # x = (y - b_left) / m_left 
        # x = (y - b_right) / m_right 
        if prev_flag == False:
            prev_l_mv.add_sample(0) 
            prev_r_mv.add_sample(0)
            prev_flag = True

        if m_left == 0.0:
            x_left = prev_l_mv.get_mm() 
        else:
            x_left = int((120.0 - b_left) / m_left)
    
        if m_right == 0.0:
            x_right = prev_r_mv.get_mm()
        else:
            x_right = int((120.0 - b_right) / m_right)

        prev_l_mv.add_sample(x_left)
        prev_r_mv.add_sample(x_right)

        x_center = (x_left + x_right) // 2 

        print("Line Positions : %d %d" %(x_left, x_right))
        print("Line Center : %d" %(x_center))

        # draw rectangles at left/right/center positions
        x_left = int(x_left) 
        x_right = int(x_right)
        x_center = int(x_center)
		
        cv2.line(line_img, (0, 360), (640, 360), (0,255,255), 2)
        cv2.rectangle(line_img, (x_left-5, 360-5), (x_left+5, 360+5), (0,255,0), 4)
        cv2.rectangle(line_img, (x_right-5, 360-5), (x_right+5, 360+5), (0,255,0), 4)
        cv2.rectangle(line_img, (x_center-5, 360-5), (x_center+5, 360+5), (0,255,0), 4)
        cv2.rectangle(line_img, (320-5, 360-5), (320+5, 360+5), (0,0,255), 4)

        cv2.imshow("line positions", line_img)
        cv2.waitKey(1)        

	#calculate angle 
        angle = (x_center - 320)
        speed = 5
	
        mv.add_sample(angle)   #indentation error 
        new_angle = mv.get_wmm( )

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 70])
        upper_white = np.array([131, 255, 2550])
        stop_img = cv2.inRange(hsv, lower_white, upper_white)
        area = stop_img[450:460, 100:540]

        if(cv2.countNonZero(area) > 2200):
	        speed = 0
        else:
	        speed = 5
       	        
        drive(new_angle, speed)


if __name__ == '__main__':
        start()
