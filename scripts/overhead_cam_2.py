#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Dear Reader When I was writing this code, only god and me knew what i was writing
#Now only god knows
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
class robot_camera():

    def get_rotation (self,msg):
        global roll, pitch, yaw, old_yaw,x,y
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        old_yaw = yaw
        if yaw<0:
            yaw += 2*np.pi


    def __init__(self):
        rospy.init_node("kamera")
        rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.camera_cb)
        self.kp=0.9
        self.kd = 7
        self.bridge = CvBridge()
        self.detected = False
        sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        self.green_list=[]
        self.demo = []
        self.red =[]
        self.dead_end_skip = 1
        rospy.spin()
    def turn_right(self):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        r = rospy.Rate(10)
        target = yaw+np.pi/2
        command = Twist()
        while np.abs(target*math.pi/180 - yaw)>0.01:
            error = [target-yaw]
            error.append(target-yaw)
            command.angular.z = -(self.kp * (target-yaw) + self.kd* (error[-1]-error[-2])/0.1)
            pub.publish(command)
            error.append(target-yaw)
            r.sleep()
        command.angular.z =0
        pub.publish(command)
        return None
    def camera_cb(self, mesaj):
        self.cap = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        # Convert BGR to HSV
        hsv = cv2.cvtColor(self.cap, cv2.COLOR_BGR2HSV)
        # define range of red color in HSV
        lower_red = np.array([0, 10, 120])
        upper_red = np.array([15, 255, 255])


        mask = cv2.inRange (hsv, lower_red, upper_red)
        mask1 = cv2.inRange(hsv,(0,223,0),(127,255,235))
        img = self.cap.copy()
        img[mask1>0] = [255,3,3]
        contours,_= cv2.findContours(mask.copy(),
                                cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            red_area = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(red_area)
            #cv2.rectangle(self.cap,(x, y),(x+w, y+h),(0, 255, 0), 2)
        w_x = 0
        w_y = 0
        b_x = 0
        b_y = 0
        if len(self.demo)>0:
            for ele in self.demo:
                start = ele[0]
                end = ele[1]
                #cv2.rectangle(self.cap,start,end,(0,255,0),-1)
                self.cap[start][end] = [0,255,0]
        if len(self.red)>0:
            for ele in self.red:
                start = ele[0]
                end = ele[1]
                cv2.rectangle(img,start,end,(255,0,0),-1)
        cv2.imshow('Walls',img)
        #white_detected = False
        #black_detected = False
        #for i in range(x,x+w):
            #for j in range(y,y+h):
                #pixel = self.cap[j][i]
                #if pixel[2]>200 and pixel[1]>125 and pixel[0]>125 and not white_detected:
                    #w_y = j
                    #w_x = i
                    #self.cap[j][i] = [255,255,255]
                    #white_detected = True
                #if pixel[0]<50 and pixel[1]<50 and pixel[2]<200 and not black_detected:
                    #b_y = j
                    #b_x = i
                    #self.cap[j][i] = [0,0,0]
                    #black_detected = True
        #print(f'{w_x},{w_y}  {b_x},{b_y}')
        #if np.abs(w_x - b_x)<=1:
            #forward = [0,np.sign(w_y-b_y)]
        #else:
            #forward = [np.sign(w_x-b_x),0]
        #print(forward)
        forward = [0,0]
        if np.abs(old_yaw)<0.02:
            forward=[0,1]
        if np.abs(old_yaw+1.57)<0.02:
            forward = [-1,0]
        if np.abs(old_yaw-1.57)<0.02:
            forward = [1,0]
        if np.abs(old_yaw-3.14)<0.02:
            forward = [0,-1]


        
        direction = [[1,0],[0,1],[-1,0],[0,-1]]
        wall_list=[]
        green_wall_list=[]
        for dir in direction:
            
            wall_sum = 0
            green_detected = False
            red_detected = False
            for i in range(x+dir[0]*w,x+dir[0]*w+w):
                for j in range(y+dir[1]*h,y+dir[1]*h+h):
                    pixel = img[j][i]
                    wall_sum+=pixel[0]
                    pixel1= self.cap[j][i]
                    if pixel1[1] > 250 and not green_detected:
                        cv2.rectangle(self.cap,(x+dir[0]*w,y+dir[1]*h),(x+dir[0]*w+w,y+dir[1]*h+h),(230,216,173))
                        green_wall_list.append(dir)
                        green_detected = True
                    if pixel1[2]>253:
                        cv2.rectangle(self.cap,(x+dir[0]*w,y+dir[1]*h),(x+dir[0]*w+w,y+dir[1]*h+h),(0,0,255))
                        red_detected = True
            wall_sum = wall_sum/(w*h)
            print(f'Dir:{dir} Wall Sum:{wall_sum}')
    
            
            if wall_sum >240 or red_detected:
                #int(f"Wall detected at: {dir}")
                cv2.rectangle(self.cap,(x+dir[0]*w,y+dir[1]*h),(x+dir[0]*w+w,y+dir[1]*h+h),(203,192,255))
                wall_list.append(dir)
                


        in_front = False
        turning = False
        in_right = False
        in_left = False
        in_back = False
        if forward[0] == -1 and forward[1] == 0:
            check_left = [0,1]
            check_right = [0,-1]
        if forward[0] == 0 and forward[1] == -1:
            check_left = [-1,0]
            check_right = [1,0]
        if forward[0] == 1 and forward[1] == 0:
            check_left = [0,-1]
            check_right = [0,1]
        if forward[0] == 0 and forward[1] == 1:
            check_left = [1,0]
            check_right = [-1,0]
        for ele in wall_list:
            if ele[0] == forward[0] and ele[1] == forward[1]:
                in_front = True
            try:
                if ele[0] == check_left[0] and ele[1] == check_left[1]:
                    in_right = True
                if ele[0] == check_right[0] and ele[1] == check_right[1]:
                    in_left = True
                if ele[0] == - forward[0] and ele[1] == -forward[1]:
                    in_back = True
            except:
                continue
        if (len(wall_list) == 3  and not in_front) :
            print('Dead End')
            self.demo.pop()
            self.red.append([(x,y),(int(x+1.1*w),int(y+1.1*h))])
        if not in_front and not turning:
            print('Forward')
            move = Twist()
            move.linear.x = 0.5
            pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
            pub.publish(move)
            self.demo.append([int(y+h/2),int(x+w/2)])
            #self.cap[int(y+h/2)][int(x+w/2)] = [230,216,173]

            
        if in_front and not turning:
            pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
            r1 = rospy.Rate(1)
            command = Twist()
            command.linear.x = -0.1
            pub.publish(command)
            r1.sleep()
            command.linear.x = 0
            pub.publish(command)
            if in_left:
                print('Turning Left')
                turning = True
                pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
                r = rospy.Rate(10)
                target = yaw+np.pi/2
                if target<0:
                    target += 2*np.pi
                if target>2*np.pi:
                    target -= 2*np.pi
                command = Twist()
                while np.abs(target - yaw)>0.01:
                    #print(f'Target:{target} Yaw:{yaw}')
                    error = [target-yaw]
                    error.append(target-yaw)
                    command.angular.z = -(self.kp * (target-yaw) + self.kd* (error[-1]-error[-2])/0.1)
                    pub.publish(command)
                    error.append(target-yaw)
                    #print(yaw)
                    r.sleep()
                command.angular.z =0
                pub.publish(command)
                in_left = False
            else:
                print('Turning Right')
                turning = True
                pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
                r = rospy.Rate(10)
                target = yaw-np.pi/2
                if target<0:
                    target += 2*np.pi
                if target>2*np.pi:
                    target -= 2*np.pi
                command = Twist()
                while np.abs(target - yaw)>0.01:
                    #print(f'Target:{target} Yaw:{yaw}')
                    error = [target-yaw]
                    error.append(target-yaw)
                    command.angular.z = -(self.kp * (target-yaw) + self.kd* (error[-1]-error[-2])/0.1)
                    pub.publish(command)
                    error.append(target-yaw)
                    #print(yaw)
                    r.sleep()
                command.angular.z =0
                pub.publish(command)

        
        cv2.imshow('frame', self.cap)
        cv2.imwrite('/home/stavan/camera_ws/src/rover_description/scripts/frame2.png',self.cap)
        cv2.waitKey(1)


  
nesne = robot_camera()

