#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import numpy as np

from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64

from classes.Particle import Particle
from classes.ParticleFilter import ParticleFilter
from classes.Map import Map

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
        self.particle_filter.initialize_particles(100)

        self.map.plot_map()
        self.particle_filter.plot_particles()
        plt.show()


def main():

    # Create an instance of the DemoNode class
    mcl_node = MonteCarloLocalizationNode()

    # Spin to keep the script for exiting
    # rospy.spin()

if __name__ == '__main__':
    main()
