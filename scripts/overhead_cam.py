#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def image_callback(msg: Image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgba8')
    cv2.imshow("",cv_image)

if __name__ == "__main__":
    rospy.init_node('camera_subscriber')
    #sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,callback=image_callback)
    #rospy.spin()
    while not rospy.is_shutdown():
        data = rospy.wait_for_message("/rrbot/camera1/image_raw", Image)
        # do stuff
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, desired_encoding='rgba8')

        cv2.imshow("",img)
        cv2.imwrite('/home/stavan/camera_ws/src/rover_description/scripts/frame1.png',img)
        cv2.waitKey(1)

