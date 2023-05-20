import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def plot_particle(self):
        marker, scale = self.gen_arrow_head_marker(self.theta)
        marker_size = 15
        plt.scatter([self.x], [self.y], c='red', marker=marker, s=(marker_size*scale)**2, alpha=0.3)

    def gen_arrow_head_marker(self, rot):
        arr = np.array([[.1, .3], [.1, -.3], [1, 0], [.1, .3]])
        angle = rot / 180 * np.pi
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