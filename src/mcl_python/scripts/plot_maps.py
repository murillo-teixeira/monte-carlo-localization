import matplotlib.pyplot as plt
import numpy as np

map_file = "/home/murillo/catkin_ws/src/monte-carlo-localization/map/5th_floor.pgm"
roi_xmin = 930
roi_xmax = 1400
roi_ymin = 730
roi_ymax = 1160

with open(map_file, 'rb') as f:
    lines = f.readlines()
    width, height = np.array(lines[2].split(), dtype=int)
    map_array = np.array([el for el in lines[4]], dtype=np.int16)
    resolution = float(lines[1].split()[3])
    map_matrix = np.array(map_array, dtype=float).reshape(height, width)
    map_matrix = np.flip(map_matrix, axis=1)
    map_matrix[map_matrix==205] = 0.5
    map_matrix[map_matrix==0] = 0.0
    map_matrix[map_matrix==254] = 1.0

plt.subplot(131)
plt.imshow(map_matrix, cmap='gray')
plt.xlim([roi_xmin,roi_xmax])
plt.ylim([roi_ymin,roi_ymax])
plt.title("5º piso")
plt.axis('off')
map_file = "/home/murillo/catkin_ws/src/monte-carlo-localization/map/elevator.pgm"
roi_xmin = 800
roi_xmax = 1023
roi_ymin = 825
roi_ymax = 1035

with open(map_file, 'rb') as f:
    lines = f.readlines()
    width, height = np.array(lines[2].split(), dtype=int)
    map_array = np.array([el for el in lines[4]], dtype=np.int16)
    resolution = float(lines[1].split()[3])
    map_matrix = np.array(map_array, dtype=float).reshape(height, width)
    map_matrix = np.flip(map_matrix, axis=1)
    map_matrix[map_matrix==205] = 0.5
    map_matrix[map_matrix==0] = 0.0
    map_matrix[map_matrix==254] = 1.0

plt.subplot(132)
plt.imshow(map_matrix, cmap='gray')
plt.xlim([roi_xmin,roi_xmax])
plt.ylim([roi_ymin,roi_ymax])
plt.title("Elevador")
plt.axis('off')
map_file = "/home/murillo/catkin_ws/src/monte-carlo-localization/map/lab.pgm"
roi_xmin = 890
roi_xmax = 1125
roi_ymin = 900
roi_ymax = 1120

with open(map_file, 'rb') as f:
    lines = f.readlines()
    width, height = np.array(lines[2].split(), dtype=int)
    map_array = np.array([el for el in lines[4]], dtype=np.int16)
    resolution = float(lines[1].split()[3])
    map_matrix = np.array(map_array, dtype=float).reshape(height, width)
    map_matrix = np.flip(map_matrix, axis=1)
    map_matrix[map_matrix==205] = 0.5
    map_matrix[map_matrix==0] = 0.0
    map_matrix[map_matrix==254] = 1.0

plt.subplot(133)
plt.imshow(map_matrix, cmap='gray')
plt.xlim([roi_xmin,roi_xmax])
plt.ylim([roi_ymin,roi_ymax])
plt.title("LSDC4")
plt.axis('off')
plt.show()