import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Wedge

from classes.Map import Map
from classes.ParticleFilter import ParticleFilter
from classes.Particle import Particle

class Visualizer:

    def __init__(self):
        plt.ion()
        self.fig, self.axes = plt.subplots(nrows=2, ncols=2, gridspec_kw={'width_ratios': [1, 1], 'height_ratios': [1, 1]}, figsize=[6, 6])
        
        for ax in self.axes.flatten():
            ax.set_xticks([])
            ax.set_yticks([])

        self.odom_subplot       = self.axes[0, 1]
        self.laser_subplot      = self.axes[1, 1]
        self.map_subplot        = self.axes[0, 0]
        self.laser_proj_subplot = self.axes[1, 0]

        self.particle_set = None

        self.configure_subplots()

        # Plotar apenas a cada 10 chamadas
        self.plot_counter = 0

    def configure_subplots(self):
        self.map_subplot.set_title("Map with particles")
        self.odom_subplot.set_title("Odometry data")
        self.laser_subplot.set_title("Likelihood field")
        self.laser_proj_subplot.set_title("Laser projection")

    def spin(self):
        plt.show(block=True)

    def draw(self):
        plt.draw()
        plt.pause(0.01)

    def plot_odometry_reading(self, x, y, theta):
        odom_particle = Particle(x, y, theta)
        self.plot_single_particle(odom_particle, self.odom_subplot)
        self.draw()

    def plot_map(self, map):
        self.map_subplot.imshow(map.map_matrix, cmap='gray')
        self.map_subplot.axis(xmin=map.roi_xmin,xmax=map.roi_xmax)
        self.map_subplot.axis(ymin=map.roi_ymax,ymax=map.roi_ymin)

    def plot_particles(self, map, particle_filter):
        self.plot_map(map)
        self.particle_set = self.map_subplot.quiver(
            [int(particle.x/0.05) for particle in particle_filter.particles], 
            [int(particle.y/0.05) for particle in particle_filter.particles], 
            [0.1*np.cos(particle.theta) for particle in particle_filter.particles], 
            [0.1*np.sin(particle.theta) for particle in particle_filter.particles], 
            color='red', alpha=1.0, angles='xy', pivot='mid')
        self.draw()

    def update_particles(self, particle_filter):
        X = np.array([int(particle.x/0.05) for particle in particle_filter.particles])
        Y = np.array([int(particle.y/0.05) for particle in particle_filter.particles])
        U = [0.1*np.cos(particle.theta) for particle in particle_filter.particles], 
        V = [0.1*np.sin(particle.theta) for particle in particle_filter.particles], 
        self.particle_set.set_offsets(np.array([X.flatten(), Y.flatten()]).T)
        self.particle_set.set_UVC(U, V)

    def plot_single_particle(self, particle, subplot):
        subplot.quiver([particle.x], [particle.y], [0.1*np.cos(particle.theta)], [0.1*np.sin(particle.theta)], color='red', angles='xy', pivot='mid')
        
    def plot_likelihood_field(self, map):
        self.laser_subplot.imshow(map.likelihood_field, cmap='gray')
        self.laser_subplot.axis(xmin=map.roi_xmin,xmax=map.roi_xmax)
        self.laser_subplot.axis(ymin=map.roi_ymax,ymax=map.roi_ymin)
    
    def plot_laser_projection(self, laser_msg):
        if self.plot_counter % 5 == 0:
            self.laser_proj_subplot.cla()
            self.laser_proj_subplot.set_xticks([])
            self.laser_proj_subplot.set_yticks([])
            self.laser_proj_subplot.axis(xmin=-6,xmax=6)
            self.laser_proj_subplot.axis(ymin=-6,ymax=6)
            self.laser_proj_subplot.set_title("Laser projection")
            self.plot_single_particle(Particle(0, 0, 0), self.laser_proj_subplot)
            wedge = Wedge((0, 0), laser_msg.range_max, np.rad2deg(laser_msg.angle_min), np.rad2deg(laser_msg.angle_max), color='b', alpha=0.5, fill=False)
            self.laser_proj_subplot.add_patch(wedge)
            for i, range in enumerate(laser_msg.ranges):
                if i % 10 == 0:
                    if laser_msg.range_min < range < laser_msg.range_max:
                        current_angle = laser_msg.angle_min + i*laser_msg.angle_increment
                        self.laser_proj_subplot.scatter([range*np.cos(current_angle)], [range*np.sin(current_angle)], c='gray', marker='.')
            self.draw()
        self.plot_counter += 1