import random
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
import numpy as np


def generate_weights(size):
    
    #Creating the weights

    weights = []

    for i in range(size):
        
        if i<(size/10):
            wannabeweight = random.uniform(5,8)
        else: 
            wannabeweight= random.uniform(0,5)
        
        weights.append(wannabeweight)

    normalizer = sum(weights)
    weights = [w/normalizer for w in weights]
    return weights




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


#Creating particles

size = 100
particles = []

for i in range(size):
    if i < size/10:
        random_array = [int(random.uniform(40,60)), int(random.uniform(40,60))]
    else: 
        random_array = [int(random.uniform(0,100)), int(random.uniform(0,100))]
    particles.append(random_array)


weights = generate_weights(size)

# Set up the figure and axes
fig, ax = plt.subplots(figsize=(100, 100))

# Plot the initial particles
x_initial = [particle[0] for particle in particles]
y_initial = [particle[1] for particle in particles]
ax.scatter(x_initial, y_initial, color='blue', marker='o', alpha=0.2, s=100, label='Initial')

# Define colormap and marker styles
cmap = get_cmap('tab10')
markers = ['s', 'o', 'D', 'v', '^']
num_iterations = 4

for iteration in range(num_iterations):
    resampled_particles = resampler(particles, weights)
    particles = resampled_particles

    # Plot each resampled particle individually with a different color and marker shape
    marker = markers[iteration]
    x_resampled = [particle[0] for particle in resampled_particles]
    y_resampled = [particle[1] for particle in resampled_particles]
    ax.scatter(x_resampled, y_resampled,marker=marker, alpha=0.7,s=100, label='Iteration {}'.format(iteration + 1))

    weights=generate_weights(size)

    print("\n\n\n\n\n")
    print(weights)
    
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Resampler')
ax.legend()
ax.grid(True)
plt.show()
