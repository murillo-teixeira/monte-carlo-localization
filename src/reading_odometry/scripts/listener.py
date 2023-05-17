#! /usr/bin/python3

import rospy
from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.pose)

if __name__ == '__main__':
    rospy.init_node("robot_controller")

    rospy.Subscriber("/pose", Odometry, callback)
    
    rospy.spin()
    rospy.loginfo("Closing subscriber")