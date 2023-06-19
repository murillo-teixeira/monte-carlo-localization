import matplotlib.pyplot as plt
import numpy as np
output_file = "/home/murillo/catkin_ws/src/monte-carlo-localization/output/5th_floor/local/2023_06_18_19_24_15.csv"

arr = np.loadtxt(output_file,
                 delimiter=",", dtype=float)

times = (arr[:, 0] - arr[0, 0])/1e9
x_mcl = arr[:, 1]
y_mcl = arr[:, 2]
theta_mcl = arr[:, 3]

x_amcl = arr[:, 4]
y_amcl = arr[:, 5]
theta_amcl = arr[:, 6]

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

fig, axes = plt.subplots(nrows=1, ncols=2, figsize=[7, 3])

for ax in axes.flatten():
    ax.set_xticks([])
    ax.set_yticks([])

axes[0].imshow(map_matrix, cmap='gray')
axes[0].axis(xmin=roi_xmin,xmax=roi_xmax)
axes[0].axis(ymin=roi_ymin,ymax=roi_ymax)
axes[1].imshow(map_matrix, cmap='gray')
axes[1].axis(xmin=roi_xmin,xmax=roi_xmax)
axes[1].axis(ymin=roi_ymin,ymax=roi_ymax)
axes[0].plot(x_mcl/0.05, y_mcl/0.05, alpha=1, c = "#0072BD")
axes[1].plot(-x_amcl/0.05 + 992, -y_amcl/0.05 + 992, alpha = 1, c ="#D95319")
axes[0].quiver(
    x_mcl[::5]/0.05,
    y_mcl[::5]/0.05,
    10*np.cos(theta_mcl[::5]),
    10*np.sin(theta_mcl[::5]),
    color="#0072BD", alpha=1, angles='xy', pivot='mid'
)
axes[1].quiver(
    -x_amcl[::5]/0.05 + 992,
    -y_amcl[::5]/0.05 + 992,
    -10*np.cos(theta_amcl[::5]),
    -10*np.sin(theta_amcl[::5]),
    color="#D95319", alpha=1, angles='xy', pivot='mid'
)
plt.show()