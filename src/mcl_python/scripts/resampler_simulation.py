import random
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap

def resampler(particles, weights):
    number_of_particles = len(particles)
    new_particles = []
    r = random.uniform(0, 1/number_of_particles)
    c = weights[0]
    i = 1

    for m in range(number_of_particles):
        u = r + (m - 1)/number_of_particles
        while u > c:        #in this cycle we find the best place in our new particle set for the particle m
            i += 1
            c += weights[i-1]
        
        new_particles.append(particles[i-1])
    
    return new_particles

# Example particles and weights
particles = [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10]]
weights = [0.1, 0.2, 0.4, 0.1, 0.2]

# Set up the figure and axes
fig, ax = plt.subplots(figsize=(8, 6))

# Plot the initial particles with transparency
x_initial = [particle[0] for particle in particles]
y_initial = [particle[1] for particle in particles]
ax.scatter(x_initial, y_initial, color='blue', marker='o', alpha=0.2, s=100, label='Initial')

# Define colormap and marker styles
cmap = get_cmap('tab10')
markers = ['s', 'o', 'D', 'v', '^']
num_iterations = 4

for iteration in range(num_iterations):
    print(weights)
    resampled_particles = resampler(particles, weights)
    particles = resampled_particles
    if iteration == 1:
        weights = [0.05, 0.1, 0.65, 0.05, 0.15]
    elif iteration == 2:
        weights = [0, 0.1, 0.8, 0, 0.1]

    # Plot each resampled particle individually with a different color and marker shape
    marker = markers[iteration]
    x_resampled = [particle[0] for particle in resampled_particles]
    y_resampled = [particle[1] for particle in resampled_particles]
    ax.scatter(x_resampled, y_resampled,marker=marker, alpha=0.7,s=100, label='Iteration {}'.format(iteration + 1))
    
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Resampler')
ax.legend()
ax.grid(True)
plt.show()
