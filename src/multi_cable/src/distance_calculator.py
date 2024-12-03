#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def base_pos_callback(msg):
    global base_pos
    rospy.loginfo("base_pos_callback triggered")
    base_pos = msg.pose.pose.position
    calculate_distance()

def load_pos_callback(msg):
    global load_pos
    rospy.loginfo("load_pos_callback triggered")
    load_pos = msg.pose.pose.position
    calculate_distance()

def calculate_distance():
    if base_pos is not None and load_pos is not None:
        distance = calculate_euclidean_distance(base_pos, load_pos)
        log_message = "Distance between base and load: %.2f" % distance
        
        # Log to ROS
        rospy.loginfo(log_message)
        # Print to console
        print(log_message)

def calculate_euclidean_distance(pos1, pos2):
    return ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2) ** 0.5

if __name__ == '__main__':
    rospy.init_node('distance_calculator', anonymous=True)

    base_pos = None
    load_pos = None

    rospy.Subscriber('/base_pos', Odometry, base_pos_callback)
    rospy.Subscriber('/load_pos', Odometry, load_pos_callback)

    rospy.spin()