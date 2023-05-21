import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from classes.Map import Map
from classes.ParticleFilter import ParticleFilter
from classes.Particle import Particle

class Visualizer:

    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(ncols=3, gridspec_kw={'width_ratios': [1, 1, 1]}, figsize=[11, 3])
        self.odom_subpot_id = 0
        self.laser_subpot_id = 1
        self.map_subpot_id = 2
        self.ax[self.odom_subpot_id].axis(xmin=-3,xmax=0)
        self.ax[self.odom_subpot_id].axis(ymin=-5,ymax=-15)
        self.ax[self.map_subpot_id].set_title("Map with particles")
        self.ax[self.odom_subpot_id].set_title("Odometry data")
        self.ax[self.laser_subpot_id].set_title("Laser data")

    def spin(self):
        plt.show(block=True)

    def draw(self):
        plt.draw()
        plt.pause(0.01)

    def plot_odometry_reading(self, x, y, theta):
        odom_particle = Particle(x, y, -theta)
        self.plot_single_particle(odom_particle, self.odom_subpot_id)
        self.draw()

    def plot_map(self, map):
        self.ax[self.map_subpot_id].imshow(map.map_matrix, cmap='gray')
        self.ax[self.map_subpot_id].axis(xmin=map.roi_xmin,xmax=map.roi_xmax)
        self.ax[self.map_subpot_id].axis(ymin=map.roi_ymax,ymax=map.roi_ymin)

    def plot_particles(self, map, particle_filter):
        self.plot_map(map)
        for particle in particle_filter.particles:
            self.plot_single_particle(particle, self.map_subpot_id)
        self.draw()

    def plot_single_particle(self, particle, subplot_id):
        marker, scale = self.gen_arrow_head_marker(particle.theta)
        marker_size = 5
        self.ax[subplot_id].scatter([particle.x], [particle.y], c='red', marker=marker, s=(marker_size*scale)**2, alpha=0.3)

    def gen_arrow_head_marker(self, rot):
        arr = np.array([[.1, .3], [.1, -.3], [1, 0], [.1, .3]])
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