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
        self.fig, self.ax = plt.subplots(nrows=1, ncols=3, gridspec_kw={'width_ratios': [1, 1, 1]}, figsize=[10, 3])
        
        # self.odom_subplot       = self.ax[1, 2]
        self.laser_subplot      = self.ax[1]
        self.map_subplot        = self.ax[0]
        self.laser_proj_subplot = self.ax[2]

        self.configure_subplots()

        # Plotar apenas a cada 10 chamadas
        self.plot_counter = 0

    def configure_subplots(self):
        # self.odom_subplot.axis(xmin=-2.9,xmax=0)
        # self.odom_subplot.axis(ymin=-8.5,ymax=-14.1)
        self.map_subplot.set_title("Map with particles")
        # self.odom_subplot.set_title("Odometry data")
        self.laser_subplot.set_title("Likelihood field")
        self.laser_proj_subplot.set_title("Laser projection")

    def spin(self):
        plt.show(block=True)

    def draw(self):
        plt.draw()
        plt.pause(0.01)

    def plot_odometry_reading(self, x, y, theta):
        odom_particle = Particle(x, y, -theta)
        self.plot_single_particle(odom_particle, self.odom_subplot)
        self.draw()

    def plot_map(self, map):
        
        self.map_subplot.imshow(map.map_matrix, cmap='gray')
        self.map_subplot.axis(xmin=map.roi_xmin,xmax=map.roi_xmax)
        self.map_subplot.axis(ymin=map.roi_ymax,ymax=map.roi_ymin)

    def plot_particles(self, map, particle_filter):
        if self.plot_counter % 100 == 0:
            self.map_subplot.cla()
            self.plot_map(map)
            for particle in particle_filter.particles:
                self.plot_single_particle(particle, self.map_subplot)
            self.draw()

    def plot_single_particle(self, particle, subplot):
        marker, scale = self.gen_arrow_head_marker(particle.theta)
        marker_size = 15
        subplot.scatter([particle.x], [particle.y], c='red', marker=marker, s=(marker_size*scale)**2, alpha=particle.weight)

    def plot_likelihood_field(self, map):
        self.laser_subplot.imshow(map.likelihood_field, cmap='gray')
        self.laser_subplot.axis(xmin=map.roi_xmin,xmax=map.roi_xmax)
        self.laser_subplot.axis(ymin=map.roi_ymax,ymax=map.roi_ymin)
    
    def plot_laser_projection(self, laser_msg):
        if self.plot_counter % 5 == 0:
            self.laser_proj_subplot.cla()
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
        

    def gen_arrow_head_marker(self, rot):
        arr = np.array([[-.3, .3], [-.3, -.3], [.6, 0], [-.3, .3]])
        angle = rot
        rot_mat = np.array([
            [np.cos(angle), np.sin(angle)],
            [-np.sin(angle), np.cos(angle)]])
        arr = np.matmul(arr, rot_mat)
        x0 = np.amin(arr[:, 0])
        x1 = np.amax(arr[:, 0])
        y0 = np.amin(arr[:, 1])
        y1 = np.amax(arr[:, 1])
        scale = np.amax(np.abs([x0, x1, y0, y1]))
        codes = [mpl.path.Path.MOVETO, mpl.path.Path.LINETO,mpl.path.Path.LINETO, mpl.path.Path.CLOSEPOLY]
        arrow_head_marker = mpl.path.Path(arr, codes)
        return arrow_head_marker, scale