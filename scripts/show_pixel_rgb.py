import cv2
img = cv2.imread('/home/stavan/camera_ws/src/rover_description/scripts/frame1.png')
cv2.imshow('img',img)
import numpy as np
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask1 = cv2.inRange(hsv,(0,223,0),(127,255,235))
#mask1 = cv2.bitwise_not(mask1)

cv2.imshow('Perspective mask',mask1)
greenBGR = np.uint8([[[103,1,1 ]]])
 
hsv_green = cv2.cvtColor(greenBGR,cv2.COLOR_BGR2HSV)
img[mask1>0] = [255,3,3]
cv2.imshow('Corrected',img)
cv2.waitKey(0)