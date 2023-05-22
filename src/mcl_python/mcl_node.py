#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

from classes.Particle import Particle
from classes.ParticleFilter import ParticleFilter
from classes.Map import Map
from classes.Visualizer import Visualizer

from threading import Lock

class MonteCarloLocalizationNode:

    def __init__(self):
        
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
        self.particle_filter.initialize_particles(1)

        visualizer = Visualizer()
        self.visualizer = visualizer
        self.visualizer.plot_particles(self.map, self.particle_filter)
        self.visualizer.plot_likelihood_field(self.map)
        self.sub_pose_topic = None
        self.initialize_subscribers()

        self.last_odometry_msg = None
        self.callback_counter = 0

        self.visualizer.spin()      

    def initialize_subscribers(self):
        self.sub_pose_topic = rospy.Subscriber('/pose', Odometry, self.callback_read_odometry)
        self.sub_scan_topic = rospy.Subscriber('/scan', LaserScan, self.callback_read_laser)

    def callback_read_odometry(self, msg):
        if self.callback_counter % 10 == 0:
            mutex = Lock()
            mutex.acquire()
            if self.last_odometry_msg:
                u  = [
                    int(round((msg.pose.pose.position.x - self.last_odometry_msg.pose.pose.position.x)/0.05, 0)),
                    int(round((msg.pose.pose.position.y - self.last_odometry_msg.pose.pose.position.y)/0.05, 0)),
                    msg.pose.pose.orientation.w - self.last_odometry_msg.pose.pose.orientation.w
                ]
                print(u)
                self.particle_filter.motion_model_odometry(u, [0.1, 0.1, 0.01, 0.01])
            self.last_odometry_msg = msg
            mutex.release()
        self.callback_counter += 1

    def callback_read_laser(self, msg):
        mutex = Lock()
        mutex.acquire()
        self.visualizer.plot_laser_projection(msg)
        self.particle_filter.likelihood_field_algorithm(msg)
        self.visualizer.plot_particles(self.map, self.particle_filter)
        mutex.release()

def main():
    
    # Create an instance of the MonteCarloLocalizationNode class
    MonteCarloLocalizationNode()


if __name__ == '__main__':
    main()
