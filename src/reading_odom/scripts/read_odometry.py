#! /usr/bin/python3

import rospy
from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo("x %s", data.pose.pose.position.x)
    rospy.loginfo("y %s", data.pose.pose.position.y)
    rospy.loginfo("z %s", data.pose.pose.position.z)

if __name__ == '__main__':
    rospy.init_node("robot_controller")

    rospy.Subscriber("/pose", Odometry, callback)
    
    rospy.spin()
    rospy.loginfo("Closing subscriber")