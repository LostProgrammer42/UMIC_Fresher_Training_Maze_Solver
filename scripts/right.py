#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

roll = pitch = yaw = 0.0
target = 0.3
kp=2
kd = 2
error = []
def get_rotation (msg):
    x=msg.pose.pose.position.x
    command =Twist()
    r = rospy.Rate(10)
    error.append(target-x)
    if np.abs(target-x)>0.01:
        error.append(target-x)
        command.linear.x = -(kp * (target-x) + kd* (error[-1]-error[-2])/0.1)
        pub.publish(command)
        error.append(target-yaw)
        print(f"target={target} current:{x}")
    else:
      command.linear.x =0
      pub.publish(command)

rospy.init_node('rotate_robot')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


rospy.spin()

