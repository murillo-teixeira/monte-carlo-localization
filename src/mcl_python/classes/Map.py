import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage

class Map:
    def __init__(self, map_file, map_roi):
        self.roi_xmin, self.roi_xmax, self.roi_ymin, self.roi_ymax = map_roi
        self.map_matrix = None
        self.load_map(map_file)
        self.create_likelihood_field(15)

    def load_map(self, map_file):
        with open(map_file, 'rb') as f:
            lines = f.readlines()
            self.width, self.height = np.array(lines[2].split(), dtype=int)
            map_array = np.array([el for el in lines[4]], dtype=np.int16)
            # print('unique: ', np.unique(map_array))
            self.map_matrix = np.array(map_array, dtype=float).reshape(self.height, self.width)
            self.map_matrix[self.map_matrix==205] = 0.5
            self.map_matrix[self.map_matrix==0] = 0.0
            self.map_matrix[self.map_matrix==254] = 1.0

    def create_likelihood_field(self, sigma_squared):
        distance_transform = scipy.ndimage.distance_transform_edt(self.map_matrix)
        squared_distances = distance_transform**2
        self.likelihood_field = np.exp(-0.5 * (squared_distances / sigma_squared))
