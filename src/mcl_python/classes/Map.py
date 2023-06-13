import numpy as np
import scipy.ndimage

class Map:
    def __init__(self, map_file, map_roi, likelihood_field_variance):
        self.roi_xmin, self.roi_xmax, self.roi_ymin, self.roi_ymax = map_roi
        self.map_matrix = None
        self.likelihood_field = None
        self.resolution = None
        self.likelihood_field_variance = likelihood_field_variance
        self.load_map(map_file)
        self.create_likelihood_field(self.likelihood_field_variance)

    def load_map(self, map_file):
        with open(map_file, 'rb') as f:
            lines = f.readlines()
            self.width, self.height = np.array(lines[2].split(), dtype=int)
            map_array = np.array([el for el in lines[4]], dtype=np.int16)
            self.resolution = float(lines[1].split()[3])
            self.map_matrix = np.array(map_array, dtype=float).reshape(self.height, self.width)
            self.map_matrix = np.flip(self.map_matrix, axis=1)
            self.map_matrix[self.map_matrix==205] = 0.5
            self.map_matrix[self.map_matrix==0] = 0.0
            self.map_matrix[self.map_matrix==254] = 1.0

    def create_likelihood_field(self, sigma_squared):
        distance_transform = scipy.ndimage.distance_transform_edt(self.map_matrix)
        squared_distances = distance_transform**2
        self.likelihood_field = np.exp(-0.5 * (squared_distances / sigma_squared))

    def get_meas_likelihood(self, x_meas, y_meas):
        return self.likelihood_field[y_meas, x_meas]
