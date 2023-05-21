#!/usr/bin/env python3

import matplotlib.pyplot as plt
import cv2
import rospy
import numpy as np

from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

from classes.Particle import Particle
from classes.ParticleFilter import ParticleFilter
from classes.Map import Map
from classes.Visualizer import Visualizer

class MonteCarloLocalizationNode:

    def __init__(self, visualizer):
        
        rospy.init_node('monte_carlo_localization')
        rospy.loginfo_once('MCL has started')

        map_file = rospy.get_param("~map_file")
        map_roi = [
            rospy.get_param("map_roi_xmin", 0),
            rospy.get_param("map_roi_xmax", 0),
            rospy.get_param("map_roi_ymin", 0),
            rospy.get_param("map_roi_ymax", 0)
        ]
        self.map = Map(map_file, map_roi)
    
        self.node_frequency = rospy.get_param("node_frequency", 1)

        self.particle_filter = ParticleFilter(self.map)
        self.particle_filter.initialize_particles(1000)

        self.visualizer = visualizer
        self.visualizer.plot_particles(self.map, self.particle_filter)
        
        self.sub_pose_topic = None
        self.initialize_subscribers()


    def initialize_subscribers(self):
        self.sub_pose_topic = rospy.Subscriber('/pose', Odometry, self.callback_read_odometry)
        self.sub_scan_topic = rospy.Subscriber('/scan', LaserScan, self.callback_read_laser)

    def callback_read_odometry(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.w
        self.visualizer.plot_odometry_reading(x, y, theta)

    def callback_read_laser(self, msg):
        pass



def main():
    
    visualizer = Visualizer()

    # Create an instance of the MonteCarloLocalizationNode class
    mcl_node = MonteCarloLocalizationNode(visualizer)

    visualizer.spin()

if __name__ == '__main__':
    main()
