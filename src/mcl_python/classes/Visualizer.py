import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Wedge

from classes.Map import Map
from classes.ParticleFilter import ParticleFilter
from scripts.euler_from_quaternion import euler_from_quaternion
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

class Visualizer:

    def __init__(self, plt_mode):
        plt.ion()
        self.plt_mode = plt_mode
        if self.plt_mode == 0:
            self.fig, self.axes = plt.subplots(nrows=2, ncols=2, gridspec_kw={'width_ratios': [1, 1], 'height_ratios': [1, 1]}, figsize=[6, 6])
            
            self.odom_subplot       = self.axes[0, 1]
            self.laser_subplot      = self.axes[1, 1]
            self.map_subplot        = self.axes[0, 0]
            self.laser_proj_subplot = self.axes[1, 0]
        
            self.map_subplot.set_title("Map with particles")
            self.odom_subplot.set_title("Odometry data")
            self.laser_subplot.set_title("Likelihood field")
            self.laser_proj_subplot.set_title("Laser projection")


        if self.plt_mode == 1:
            self.fig, self.axes = plt.subplots(nrows=1, ncols=2, gridspec_kw={'width_ratios': [1, 1]}, figsize=[7, 4])
            
            self.map_subplot        = self.axes[0]
            self.gd_map_subplot     = self.axes[1]
            self.map_subplot.set_title("Our results")
            self.gd_map_subplot.set_title("AMCL results")

        for ax in self.axes.flatten():
            ax.set_xticks([])
            ax.set_yticks([])

        self.particle_set = None
        self.particle_set_mean = None
        self.amcl_particle = None
        self.amcl_particle_cloud = None

        self.plot_counter = 0
           
    def spin(self):
        plt.show(block=True)

    def draw(self):
        plt.draw()
        plt.pause(0.01)

    def plot_ground_truth_map(self, map: Map):
        self.plot_map(map, self.gd_map_subplot)
        
    def plot_amcl_pose(self, map: Map, amcl_particle_cloud: PoseArray, x, y, theta):
        
        if not self.amcl_particle_cloud:
            self.amcl_particle_cloud = self.gd_map_subplot.quiver(
                [(-particle.position.x/map.resolution + 1984/2) for particle in amcl_particle_cloud.poses],
                [(-particle.position.y/map.resolution + 1984/2) for particle in amcl_particle_cloud.poses],
                -0.001*np.cos([theta]),
                -0.001*np.sin([theta]),
                color='red', alpha=0.1, angles='xy', pivot='mid'
            )
        else:
            X = np.array([(-particle.position.x/map.resolution + 1984/2) for particle in amcl_particle_cloud.poses])
            Y = np.array([(-particle.position.y/map.resolution + 1984/2) for particle in amcl_particle_cloud.poses])
            U = -0.001*np.cos([theta]),
            V = -0.001*np.sin([theta]),
            self.amcl_particle_cloud.set_offsets(np.array([X.flatten(), Y.flatten()]).T)
            self.amcl_particle_cloud.set_UVC(U, V)

        if not self.amcl_particle:
            self.amcl_particle = self.gd_map_subplot.quiver(
                [-x/map.resolution + 1984/2],
                [-y/map.resolution + 1984/2],
                -0.001*np.cos([theta]),
                -0.001*np.sin([theta]),
                color='black', alpha=1, angles='xy', pivot='mid'
            )
        else:
            X = np.array([-x/map.resolution + 1984/2])
            Y = np.array([-y/map.resolution + 1984/2])
            U = -0.001*np.cos([theta]),
            V = -0.001*np.sin([theta]),
            self.amcl_particle.set_offsets(np.array([X.flatten(), Y.flatten()]).T)
            self.amcl_particle.set_UVC(U, V)

        self.draw()

    def plot_odometry_reading(self, x, y, theta):
        self.odom_subplot.quiver([x], [y], [0.1*np.cos(theta)], [0.1*np.sin(theta)], color='red', angles='xy', pivot='mid')
        self.draw()

    def plot_map(self, map : Map, subplot):
        subplot.imshow(map.map_matrix, cmap='gray')
        subplot.axis(xmin=map.roi_xmin,xmax=map.roi_xmax)
        subplot.axis(ymin=map.roi_ymin,ymax=map.roi_ymax)

    def plot_particles(self, map : Map, particle_filter : ParticleFilter):
        self.plot_map(map, self.map_subplot)
        particle_filter.particles.update_attr()
        self.particle_set = self.map_subplot.quiver(
            particle_filter.particles.x_positions/map.resolution,
            particle_filter.particles.y_positions/map.resolution,
            0.001*np.cos(particle_filter.particles.orientations),
            0.001*np.sin(particle_filter.particles.orientations),
            color='red', alpha=0.1, angles='xy', pivot='mid'
        )
        self.draw()

    def update_particles(self, map : Map, particle_filter : ParticleFilter, x, y, theta):
        particle_filter.particles.update_attr()
        X = particle_filter.particles.x_positions/map.resolution
        Y = particle_filter.particles.y_positions/map.resolution
        U = 0.001*np.cos(particle_filter.particles.orientations),
        V = 0.001*np.sin(particle_filter.particles.orientations),
        self.particle_set.set_offsets(np.array([X.flatten(), Y.flatten()]).T)
        self.particle_set.set_UVC(U, V)
        
        if not self.particle_set_mean:
            self.particle_set_mean = self.map_subplot.quiver(
                [x/map.resolution],
                [y/map.resolution],
                0.001*np.cos([theta]),
                0.001*np.sin([theta]),
                color='black', alpha=1, angles='xy', pivot='mid'
            )
        else:
            X = np.array([x/map.resolution])
            Y = np.array([y/map.resolution])
            U = 0.001*np.cos([theta]),
            V = 0.001*np.sin([theta]),
            self.particle_set_mean.set_offsets(np.array([X.flatten(), Y.flatten()]).T)
            self.particle_set_mean.set_UVC(U, V)

    def plot_likelihood_field(self, map: Map):
        self.laser_subplot.imshow(map.likelihood_field, cmap='gray')
        self.laser_subplot.axis(xmin=map.roi_xmin,xmax=map.roi_xmax)
        self.laser_subplot.axis(ymin=map.roi_ymin,ymax=map.roi_ymax)
    
    def plot_laser_projection(self, laser_msg: LaserScan):
        if self.plot_counter % 5 == 0:
            self.laser_proj_subplot.cla()
            self.laser_proj_subplot.set_xticks([])
            self.laser_proj_subplot.set_yticks([])
            self.laser_proj_subplot.axis(xmin=-6,xmax=6)
            self.laser_proj_subplot.axis(ymin=-6,ymax=6)
            self.laser_proj_subplot.set_title("Laser projection")
            self.laser_proj_subplot.quiver([0], [0], [0.1], [0], color='red', angles='xy', pivot='mid')
            wedge = Wedge((0, 0), laser_msg.range_max, np.rad2deg(laser_msg.angle_min), np.rad2deg(laser_msg.angle_max), color='b', alpha=0.5, fill=False)
            self.laser_proj_subplot.add_patch(wedge)
            for i, range in enumerate(laser_msg.ranges):
                if i % 10 == 0:
                    if laser_msg.range_min < range < laser_msg.range_max:
                        current_angle = laser_msg.angle_min + i*laser_msg.angle_increment
                        self.laser_proj_subplot.scatter([range*np.cos(current_angle)], [range*np.sin(current_angle)], c='gray', marker='.')
            self.draw()
        self.plot_counter += 1