#!/usr/bin/env python3

import rospy
import sys
import matplotlib.pyplot as plt
import numpy as np

from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64

from classes.Particle import Particle

class MonteCarloLocalizationNode:

    def __init__(self):
        
        rospy.init_node('monte_carlo_localization')
        rospy.loginfo_once('MCL has started')

        self.map_file       = rospy.get_param("~map_file")
        self.node_frequency = rospy.get_param("node_frequency", 1)
        self.map_xmin       = rospy.get_param("map_xmin", 0)
        self.map_xmax       = rospy.get_param("map_xmax", 0)
        self.map_ymin       = rospy.get_param("map_ymin", 0)
        self.map_ymax       = rospy.get_param("map_ymax", 0)
        
        self.map_matrix = self.load_map()
        self.plot_map()

        
        self.particles = None

        # Initialize the ROS node
        self.initialize_particles(10)

    def load_map(self):
        with open(self.map_file, 'rb') as f:
            lines = f.readlines()
            width, height = np.array(lines[2].split(), dtype=int)
            map_array = np.array([el for el in lines[4]], dtype=np.int16)
            # print('unique: ', np.unique(map_array))
            map_matrix = np.array(map_array, dtype=float).reshape(height, width)
            map_matrix[map_matrix==205] = 0.5
            map_matrix[map_matrix==0] = 0
            map_matrix[map_matrix==254] = 1
        
        return map_matrix

    def plot_map(self):
        plt.imshow(self.map_matrix[self.map_ymin:self.map_ymax, self.map_xmin:self.map_xmax], cmap='gray')
        plt.show()

    def initialize_particles(self, number_of_particles = 50):
        self.particles = []
        for _ in range(number_of_particles) :
            self.particles.append(Particle(0, 0, 0))
        # rospy.loginfo_once(self.particles)

def main():

    # Create an instance of the DemoNode class
    mcl_node = MonteCarloLocalizationNode()

    # Spin to keep the script for exiting
    rospy.spin()

if __name__ == '__main__':
    main()
