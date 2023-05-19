#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from classes.Particle import Particle

class MonteCarloLocalizationNode:

    def __init__(self):

        # Initialize some necessary variables here
        self.node_frequency = None
        self.sub_fake_sensor_topic = None
        self.pub_demo_topic = None
        
        self.particles = None

        # Initialize the ROS node
        rospy.init_node('demo_node')
        rospy.loginfo_once('Demo Node has started')
        self._initialize_particles(10)

    def _initialize_particles(self, number_of_particles = 50):
        self.particles = []
        for _ in range(number_of_particles) :
            self.particles.append(Particle(0, 0, 0))
        rospy.loginfo_once(self.particles)


def main():

    # Create an instance of the DemoNode class
    mcl_node = MonteCarloLocalizationNode()

    # Spin to keep the script for exiting
    rospy.spin()

if __name__ == '__main__':
    main()
