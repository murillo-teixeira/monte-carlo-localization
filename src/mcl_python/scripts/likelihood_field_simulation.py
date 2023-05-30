import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import scipy.ndimage

np.random.seed(0)

def create_likelihood_field(map_matrix, sigma_squared):
    distance_transform = scipy.ndimage.distance_transform_edt(map_matrix)
    squared_distances = distance_transform**2
    return np.exp(-0.5 * (squared_distances / sigma_squared))

class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 1
    
    def reset_weight(self):
        self.weight = 1

    def set_weight(self, weight):
        self.weight = weight

def plotPixel(map, x1, y1, x2, y2, dx, dy, decide):
   
    pk = 2 * dy - dx
    # ray = [[x1, y1]]
    for i in range(0,dx+1):
        if decide == 0:
            # print(map[x1, y1])
            # if map[x1, y1] < 50:
            #     return ray
            ray.append([x1, y1])
        else:
            # print(map[y1, x1])
            # if map[y1, x1] < 50:
            #     return ray
            ray.append([y1, x1])
    
        if(x1<x2):
            x1 = x1 + 1
        else:
            x1 = x1 - 1
        if (pk < 0):
        
            if (decide == 0):
            
                pk = pk + 2 * dy
            else:
                pk = pk + 2 * dy
        else:
            if(y1<y2):
                y1 = y1 + 1
            else:
                y1 = y1 - 1
            pk = pk + 2 * dy - 2 * dx
    return ray

z_hit = 0.7
z_rand = 0.1
z_max = 0.2

map_file = './src/mcl_python/scripts/mocked_map.bmp'
image = np.array(Image.open(map_file))[:, :, 0]
likelihood_field = create_likelihood_field(image, 35)

# Generate some example particles
origin = [270, 250]
num_particles = 8
particles_r = np.random.normal(0, 20, num_particles)
particles_alpha = np.random.uniform(0, 2*np.pi, num_particles)
particles_theta = np.random.normal(0, np.pi/20, num_particles)
particles = [
    Particle(
        origin[0] + particles_r[i]*np.cos(particles_alpha[i]),
        origin[1] + particles_r[i]*np.sin(particles_alpha[i]),
        particles_theta[i]
    ) for i in range(num_particles)]


robot_particle = Particle(origin[0], origin[1], 0)

min_angle = -130
max_angle = 130
step_angle = 5
N = (max_angle - min_angle)/step_angle + 1
angles = np.array([min_angle + i*step_angle for i in range(int(N))])
angles = np.deg2rad(angles)
ranges = []
max_range = 100

fig, axes = plt.subplots(nrows=3, ncols=3)

for ax in axes.flatten():
    ax.set_xticks([])
    ax.set_yticks([])

axes[0, 0].imshow(image, cmap='gray')
axes[0, 0].set_xlim([120, 370])
axes[0, 0].set_ylim([370, 120])
q = 1
for i, angle in enumerate(angles):
    x_max = (robot_particle.x + \
        max_range*np.cos(robot_particle.theta + angle))
    y_max = (robot_particle.y + \
        max_range*np.sin(robot_particle.theta + angle))
    
    dx = int(abs(x_max - robot_particle.x))
    dy = int(abs(y_max - robot_particle.y))
    
    ray = []
    if (dx > dy):
        ray = plotPixel(image, int(robot_particle.x), int(robot_particle.y), int(x_max), int(y_max), dx, dy, 0)
    else:
        ray = plotPixel(image, int(robot_particle.y), int(robot_particle.x), int(y_max), int(x_max), dy, dx, 1)
    
    ray = np.array(ray)
    measured_range_index = 0
    for i, pos in enumerate(ray):
        if image[pos[1], pos[0]] == 0:
            measured_range_index = i
            break
    
    ranges.append(np.hypot(pos[0] - origin[0], pos[1] - origin[1]))

    measurement_likelihood = likelihood_field[int(pos[0]), int(pos[1])]
    # Como explicar o z_rand/z_max?
    q = q*(z_hit*measurement_likelihood + z_rand/z_max)
    # factor = (z_hit*measurement_likelihood + z_rand)
    # if ranges[-1] == max_range:
    #     factor += z_max
    # q = q*factor

    if measured_range_index != 0:
        axes[0, 0].scatter([ray[:measured_range_index, 0][-1]], [ray[:measured_range_index, 1][-1]], c='red', marker='.', s=10)
        axes[0, 0].plot(ray[:measured_range_index, 0], ray[:measured_range_index, 1], alpha=0.1, c='gray', marker=',')

axes[0, 0].set_title(f'Baseline: {q:.2e}')

Q2 = axes[0, 0].quiver(
    [robot_particle.x], 
    [robot_particle.y], 
    [0.1*np.cos(robot_particle.theta)], 
    [0.1*np.sin(robot_particle.theta)], color='blue', angles='xy', pivot='mid', width=0.005)

for i, ax in enumerate(axes.flatten()[1:]):
    ax.imshow(likelihood_field, cmap='gray')
    ax.set_xlim([120, 370])
    ax.set_ylim([370, 120])
    Q = ax.quiver(
        [particles[i].x], 
        [particles[i].y], 
        [0.001*np.cos(particles[i].theta)], 
        [0.001*np.sin(particles[i].theta)], color='red', angles='xy', pivot='mid', width=0.005)
    
    q = 1
    for j, range in enumerate(ranges):
        x_meas = (particles[i].x + \
            range*np.cos(particles[i].theta + angles[j]))
        y_meas = (particles[i].y + \
            range*np.sin(particles[i].theta + angles[j]))
        ax.scatter([x_meas], [y_meas], c='red', marker='.', s=10)
        measurement_likelihood = likelihood_field[int(x_meas), int(y_meas)]
        # Como explicar o z_rand/z_max?
        q = q*(z_hit*measurement_likelihood + z_rand/z_max)
        # factor = (z_hit*measurement_likelihood + z_rand)
        # if ranges[-1] == max_range:
        #     factor += z_max
        # q = q*factor
    ax.set_title(f"Weight: {q:.2e}")

plt.suptitle("Likelihood field algorithm")
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()