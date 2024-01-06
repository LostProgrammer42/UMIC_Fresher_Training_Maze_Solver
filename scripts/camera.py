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
    #sub = rospy.Subscriber("/rover/camera1/image_raw",Image,callback=image_callback)
    #rospy.spin()
    while not rospy.is_shutdown():
        data = rospy.wait_for_message("/rover/camera1/image_raw", Image)
        # do stuff
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgra8')
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        thresh = 40
        img = cv2.threshold(img, thresh, 255, cv2.THRESH_BINARY)[1]
        qcd = cv2.QRCodeDetector()
        info, d,b = qcd.detectAndDecode(img)
        rospy.loginfo(info)
        cv2.imshow("",cv_image)
        cv2.imwrite('frame_qr.png',cv_image)
        cv2.waitKey(10)
