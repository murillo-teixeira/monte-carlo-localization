#!/usr/bin/env python3

import rospy
from datetime import datetime
import numpy as np
np.random.seed(9999)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped

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
        self.current_amcl_pose = None
        self.current_amcl_particle_cloud = None

        if self.is_plt_on == 1:
            self.visualizer = Visualizer(self.plt_mode)
            self.visualizer.plot_particles(self.map, self.particle_filter)
            
            if self.plt_mode == 0:
                self.visualizer.plot_likelihood_field(self.map)
            if self.plt_mode == 1:
                self.visualizer.plot_ground_truth_map(self.map)
        
        self.initialize_subscribers()
        self.initialize_publishers()
        self.initialize_timer()

        if self.output_on == 1:
            self.file_object = open(f'{self.output_path}/{datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}.csv', 'a')

        if self.is_plt_on == 1:
            self.visualizer.spin()
        else:
            rospy.spin()

        if self.output_on == 1:
            self.file_object.close()

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
        self.motion_model_noise = rospy.get_param("motion_model_noise", [0.01, 0.01, 0.1, 0.008])
        
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
        if self.is_plt_on == 1:
            self.plt_mode = rospy.get_param("plt_mode", 0)

        self.output_on = rospy.get_param("output_on", 0)
        if self.output_on == 1:
            self.output_path = rospy.get_param("output_path", "")

    def initialize_timer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timer_callback) 

    def initialize_subscribers(self):
        self.sub_pose_topic = rospy.Subscriber('/pose', Odometry, self.callback_read_odometry)
        self.sub_scan_topic = rospy.Subscriber('/scan', LaserScan, self.callback_read_laser)
        if self.plt_mode == 1:
            self.amcl_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_read_amcl_pose)
            self.amcl_particle_cloud = rospy.Subscriber('/particlecloud', PoseArray, self.callback_read_amcl_particle_cloud)

    def initialize_publishers(self):
        self.pub_mcl_particles = rospy.Publisher('/mcl_particles', PoseArray, queue_size=1)

    def callback_read_amcl_particle_cloud(self, msg):
        self.current_amcl_particle_cloud = msg

    def callback_read_amcl_pose(self, msg):
        self.current_amcl_pose = msg

    def callback_read_odometry(self, msg):
        self.current_odometry_msg = msg

    def callback_read_laser(self, msg):
        self.current_laser_msg = msg

    def timer_callback(self, timer):
        if not self.last_odometry_msg:
            self.last_odometry_msg = self.current_odometry_msg
        elif self.current_laser_msg:
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
            self.particle_filter.motion_model_odometry(u, self.motion_model_noise)
            
            # Run the likelihood field algorithm
            self.particle_filter.likelihood_field_algorithm(self.processing_laser_msg)

            # Deal with particles outside the map
            self.particle_filter.remove_outside_map_particles()
            
            # Normalizing weights for the chosen resampler
            self.particle_filter.normalize_weights()

            # Resampling particles only if the effective number 
            # of particles is less than 80% of the real one
            n_eff, number_of_particles = self.particle_filter.get_n_eff()
            if n_eff < 0.9*number_of_particles:
                self.particle_filter.resampler()

            # Update the visualizer
            x_mean, y_mean, theta_mean = self.particle_filter.particles.get_mean_particle(90)
            if self.output_on == 1:
                self.file_object.write(f'{timer.current_real}, {x_mean}, {y_mean}, {theta_mean}, ')
            self.visualizer.update_particles(self.map, self.particle_filter, x_mean, y_mean, theta_mean)

            if self.is_plt_on == 1:
                # If plotting in debug mode
                if self.plt_mode == 0:
                    # Plot odometry reading
                    self.visualizer.plot_odometry_reading(current_x, current_y, current_theta)
                    
                    # Plot the laser projection
                    self.visualizer.plot_laser_projection(self.current_laser_msg)
                
                # If plotting in results mode
                if self.plt_mode == 1:
                    x_amcl = self.current_amcl_pose.pose.pose.position.x
                    y_amcl = self.current_amcl_pose.pose.pose.position.y
                    orientation_amcl = self.current_amcl_pose.pose.pose.orientation
                    _, _, theta_amcl = euler_from_quaternion(orientation_amcl.x, orientation_amcl.y, orientation_amcl.z, orientation_amcl.w)
                    if self.output_on == 1:
                        self.file_object.write(f'{x_amcl}, {y_amcl}, {theta_amcl}\n')
                    self.visualizer.plot_amcl_pose(self.map, self.current_amcl_particle_cloud, x_amcl, y_amcl, theta_amcl)

            self.last_odometry_msg = self.processing_odometry_msg


    # Tentativa de publicar as partículas para o RViz
    # Funciona, mas precisa de alguma transformação de coordenadas
    def publish_particles(self):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        for particle in self.particle_filter.particles.T:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = particle[0], particle[1], 0.2
            x, y, z, w = quaternion_from_euler(0, 0, particle[2])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = x, y, z, w
            pose_array.poses.append(pose)
        self.pub_mcl_particles.publish(pose_array)

def main():
    MonteCarloLocalizationNode()
    

if __name__ == '__main__':
    main()
    