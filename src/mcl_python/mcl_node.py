#!/usr/bin/env python3

import rospy
import time
import numpy as np
np.random.seed(0)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray

from classes.ParticleFilter import ParticleFilter
from classes.Map import Map
from classes.Visualizer import Visualizer

from scripts.euler_from_quaternion import euler_from_quaternion
from scripts.quaternion_from_euler import quaternion_from_euler

class MonteCarloLocalizationNode:

    def __init__(self):
        
        rospy.init_node('monte_carlo_localization')

        self.load_parameters()

        self.map = Map(self.map_file, self.map_roi, self.likelihood_field_variance)
    
        self.particle_filter = ParticleFilter(self.map, self.zhit, self.zrand, self.x_sensor, self.y_sensor)
        
        # Initialize particles (locally or globally)
        if self.global_localization:
            self.particle_filter.initialize_particles_globally(self.number_of_particles)
        else:
            self.particle_filter.initialize_particles_locally(self.number_of_particles, self.initial_pos_x, self.initial_pos_y, self.initial_pos_sd, self.initial_theta, self.initial_theta_sd)
            
        self.last_odometry_msg = None
        self.current_odometry_msg = None
        self.current_laser_msg = None

        if self.is_plt_on == 1:
            self.visualizer = Visualizer()
            self.visualizer.plot_particles(self.map, self.particle_filter)
            self.visualizer.plot_likelihood_field(self.map)
        
        self.initialize_subscribers()
        self.initialize_publishers()
        self.initialize_timer()
        
        if self.is_plt_on == 1:
            self.visualizer.spin()
        else:
            rospy.spin()

    def load_parameters(self):
        self.map_file = rospy.get_param("~map_file")
        self.map_roi = [
            rospy.get_param("map_roi_xmin", 0),
            rospy.get_param("map_roi_xmax", 1984),
            rospy.get_param("map_roi_ymin", 0),
            rospy.get_param("map_roi_ymax", 1984)
        ]
        self.node_frequency = rospy.get_param("node_frequency", 1)
        self.number_of_particles = rospy.get_param("number_of_particles", 100)

        self.zhit = rospy.get_param("zhit", 0.7)
        self.zrand = rospy.get_param("zrand", 0.3)
        self.likelihood_field_variance = rospy.get_param("sigma_squared", 0)
        
        # Relative position of the LIDAR
        self.x_sensor = rospy.get_param("x_sensor", 0)
        self.y_sensor = rospy.get_param("y_sensor", 0)

        self.global_localization = rospy.get_param("global_localization", 1)

        if self.global_localization == 0:
            self.initial_pos_x = rospy.get_param("initial_pos_x", 992)
            self.initial_pos_y = rospy.get_param("initial_pos_y", 992)
            self.initial_pos_sd = rospy.get_param("initial_pos_sd", 10000000)
            self.initial_theta = rospy.get_param("initial_theta", 0)
            self.initial_theta_sd = rospy.get_param("initial_theta_sd", 100)

        self.is_plt_on = rospy.get_param("plt_on", 0)
        
    def initialize_timer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timer_callback) 

    def initialize_subscribers(self):
        self.sub_pose_topic = rospy.Subscriber('/pose', Odometry, self.callback_read_odometry)
        self.sub_scan_topic = rospy.Subscriber('/scan', LaserScan, self.callback_read_laser)

    def initialize_publishers(self):
        self.pub_mcl_particles = rospy.Publisher('/mcl_particles', PoseArray, queue_size=1)

    def callback_read_odometry(self, msg):
        self.current_odometry_msg = msg

    def callback_read_laser(self, msg):
        self.current_laser_msg = msg

    def timer_callback(self, timer):
        if not self.last_odometry_msg:
            self.last_odometry_msg = self.current_odometry_msg
        else:
            self.processing_laser_msg = self.current_laser_msg
            self.processing_odometry_msg = self.current_odometry_msg
            
            current_x = self.processing_odometry_msg.pose.pose.position.x
            previous_x = self.last_odometry_msg.pose.pose.position.x
            current_y = self.processing_odometry_msg.pose.pose.position.y
            previous_y = self.last_odometry_msg.pose.pose.position.y

            current_orientation = self.processing_odometry_msg.pose.pose.orientation
            previous_orientation = self.last_odometry_msg.pose.pose.orientation
            _, _, current_theta = euler_from_quaternion(current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w)
            _, _, previous_theta = euler_from_quaternion(previous_orientation.x, previous_orientation.y, previous_orientation.z, previous_orientation.w)
            
            u  = [
                current_x - previous_x,
                current_y - previous_y,
                current_theta - previous_theta,
                previous_theta
            ]

            # Run the odometry motion model to update the particles' positions
            self.particle_filter.motion_model_odometry(u, [0.01, 0.01, 0.1, 0.01])
            
            # Run the likelihood field algorithm
            self.particle_filter.likelihood_field_algorithm(self.processing_laser_msg)

            # Deal with particles outside the map
            self.particle_filter.remove_outside_map_particles()
            
            # Normalizing weights for the chosen resampler
            self.particle_filter.normalize_weights()

            
            # Resampling particles only if the effective number 
            # of particles is less than 80% of the real one
            n_eff, number_of_particles = self.particle_filter.get_n_eff()
            if n_eff < 0.8*number_of_particles:
                self.particle_filter.resampler()
            
            if self.is_plt_on == 1:
                # Plot odometry reading
                self.visualizer.plot_odometry_reading(current_x, current_y, current_theta)
                
                # Plot the laser projection
                self.visualizer.plot_laser_projection(self.current_laser_msg)

                # Update the visualizer
                self.visualizer.update_particles(self.map, self.particle_filter)

            self.last_odometry_msg = self.processing_odometry_msg

            # self.publish_particles()

    # Tentativa de publicar as partículas para o RViz
    # Funciona, mas precisa de alguma transformação de coordenadas
    def publish_particles(self):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        for particle in self.particle_filter.particles.T:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = particle[0] - 1984*0.05/2,  1984*0.05/2 - particle[1], 0.2
            x, y, z, w = quaternion_from_euler(0, 0, particle[2])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = x, y, z, w
            pose_array.poses.append(pose)
        self.pub_mcl_particles.publish(pose_array)

def main():
    MonteCarloLocalizationNode()

if __name__ == '__main__':
    main()
    