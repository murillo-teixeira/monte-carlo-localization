#!/usr/bin/env python3

import rospy
from threading import Lock
import numpy as np
np.random.seed(0)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from classes.ParticleFilter import ParticleFilter
from classes.Map import Map
from classes.Visualizer import Visualizer

from scripts.euler_from_quaternion import euler_from_quaternion

class MonteCarloLocalizationNode:

    def __init__(self):
        
        rospy.init_node('monte_carlo_localization')
        rospy.loginfo_once('MCL has started')

        self.load_parameters()

        self.map = Map(self.map_file, self.map_roi)
    
        self.particle_filter = ParticleFilter(self.map, self.zhit, self.zrand, self.x_sensor, self.y_sensor)
        self.particle_filter.initialize_particles(1000)

        self.visualizer = Visualizer()
        self.visualizer.plot_particles(self.map, self.particle_filter)
        self.visualizer.plot_likelihood_field(self.map)
        
        self.last_odometry_msg = None

        self.current_odometry_msg = None
        self.current_laser_msg = None
        self.mutex = Lock()
        self.initialize_subscribers()

        self.initialize_timer()

        self.visualizer.spin()      

    def load_parameters(self):
        self.map_file = rospy.get_param("~map_file")
        self.map_roi = [
            rospy.get_param("map_roi_xmin", 0),
            rospy.get_param("map_roi_xmax", 0),
            rospy.get_param("map_roi_ymin", 0),
            rospy.get_param("map_roi_ymax", 0)
        ]
        self.node_frequency = rospy.get_param("node_frequency", 1)
        self.zhit = rospy.get_param("zhit", 0.7)
        self.zrand = rospy.get_param("zrand", 0.3)
        self.x_sensor = rospy.get_param("x_sensor", 0)
        self.y_sensor = rospy.get_param("y_sensor", 0)

    def initialize_timer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timer_callback) 

    def initialize_subscribers(self):
        self.sub_pose_topic = rospy.Subscriber('/pose', Odometry, self.callback_read_odometry)
        self.sub_scan_topic = rospy.Subscriber('/scan', LaserScan, self.callback_read_laser)

    def callback_read_odometry(self, msg):
        self.current_odometry_msg = msg

    def callback_read_laser(self, msg):
        self.current_laser_msg = msg

    def timer_callback(self, timer):
        # print("Start", timer)
        if not self.last_odometry_msg:
            self.last_odometry_msg = self.current_odometry_msg
        else:
            self.processing_laser_msg = self.current_laser_msg
            self.processing_odometry_msg = self.current_odometry_msg
            
            current_x = self.processing_odometry_msg.pose.pose.position.x
            previous_x = self.last_odometry_msg.pose.pose.position.x
            current_y = self.processing_odometry_msg.pose.pose.position.y
            previous_y = self.last_odometry_msg.pose.pose.position.y

            _, _, current_theta = euler_from_quaternion(
                self.processing_odometry_msg.pose.pose.orientation.x,
                self.processing_odometry_msg.pose.pose.orientation.y,
                self.processing_odometry_msg.pose.pose.orientation.z,
                self.processing_odometry_msg.pose.pose.orientation.w
            )
            _, _, previous_theta = euler_from_quaternion(
                self.last_odometry_msg.pose.pose.orientation.x,
                self.last_odometry_msg.pose.pose.orientation.y,
                self.last_odometry_msg.pose.pose.orientation.z,
                self.last_odometry_msg.pose.pose.orientation.w
            )
            u  = [
                current_x - previous_x,
                current_y - previous_y,
                current_theta - previous_theta,
                previous_theta
            ]

            # Run the odometry motion model to update the particles' positions
            self.particle_filter.motion_model_odometry(u, [0.2, 0.2, 0.1, 0.1])
            
            if np.hypot(u[0], u[1]) > 0.1:
                # Run the likelihood field algorithm
                self.particle_filter.likelihood_field_algorithm(self.processing_laser_msg)

                # Resampling particles
                self.particle_filter.resampler()

            # Plot odometry reading
            self.visualizer.plot_odometry_reading(current_x, current_y, current_theta)
            
            # Plot the laser projection
            self.visualizer.plot_laser_projection(self.current_laser_msg)

            # # Update the visualizer
            self.visualizer.update_particles(self.map, self.particle_filter)

            self.last_odometry_msg = self.processing_odometry_msg
        # print("End", timer)

def main():
    print("[Versão 2]")
    MonteCarloLocalizationNode()

if __name__ == '__main__':
    main()