#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from classes.ParticleFilter import ParticleFilter
from classes.Map import Map
from classes.Visualizer import Visualizer

from threading import Lock

class MonteCarloLocalizationNode:

    def __init__(self):
        
        rospy.init_node('monte_carlo_localization')
        rospy.loginfo_once('MCL has started')

        self.load_parameters()

        self.map = Map(self.map_file, self.map_roi)
    
        self.particle_filter = ParticleFilter(self.map)
        self.particle_filter.initialize_particles(10)

        visualizer = Visualizer()
        self.visualizer = visualizer
        self.visualizer.plot_particles(self.map, self.particle_filter)
        self.visualizer.plot_likelihood_field(self.map)
        
        self.last_odometry_msg = None
        self.current_odometry_msg = None
        self.current_laser_scan = None
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

    def initialize_timer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timer_callback) 

    def initialize_subscribers(self):
        self.sub_pose_topic = rospy.Subscriber('/pose', Odometry, self.callback_read_odometry)
        self.sub_scan_topic = rospy.Subscriber('/scan', LaserScan, self.callback_read_laser)

    def callback_read_odometry(self, msg):
        self.current_odometry_msg = msg

    def callback_read_laser(self, msg):
        self.current_laser_scan = msg

    def timer_callback(self, timer):
        self.mutex.acquire()
        if self.last_odometry_msg:
            u  = [
                self.current_odometry_msg.pose.pose.position.x - \
                    self.last_odometry_msg.pose.pose.position.x,
                self.current_odometry_msg.pose.pose.position.y - \
                    self.last_odometry_msg.pose.pose.position.y,
                self.current_odometry_msg.pose.pose.orientation.w - \
                    self.last_odometry_msg.pose.pose.orientation.w,
                self.last_odometry_msg.pose.pose.orientation.w
            ]

            # Run the odometry motion model to update the particles' positions
            self.particle_filter.motion_model_odometry(u, [0.01, 0.01, 0.01, 0.01])
            
            # Update the particles' weights
            
            # Run the likelihood field algorithm
            self.particle_filter.likelihood_field_algorithm(self.current_laser_scan)

            # Plot the laser projection
            self.visualizer.plot_laser_projection(self.current_laser_scan)
            
            self.visualizer.update_particles(self.particle_filter)
            # Resampling particles
            # self.particle_filter.resampler()

            # Update the particles' weights
            # self.visualizer.update_particles(self.particle_filter)

        self.last_odometry_msg = self.current_odometry_msg
        self.mutex.release()

def main():
    MonteCarloLocalizationNode()

if __name__ == '__main__':
    main()