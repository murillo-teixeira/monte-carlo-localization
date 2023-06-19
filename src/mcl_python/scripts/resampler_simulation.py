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

    # Testando o caso de todos os pesos iguais
    # weight=1
    # normalizer =weight*size
    # weights = [weight/normalizer for w in weights]
    
    return weights

def resampler(particles, weights):

    number_of_particles = len(particles)
    new_particles = []
    
    r = random.uniform(0, 1/number_of_particles)

    c = weights[0]
    i = 0
    for m in range(number_of_particles):
        u = r + (m)/number_of_particles
        #in this cycle we find the best place in our new particle set for the particle m
        while u > c:        
            i += 1
            c += weights[i-1]

        new_particles.append(particles[i-1])
    
    return new_particles


#Creating particles

size = 250
particles = []

for i in range(size):
    if i < size/10:
        random_array = [random.uniform(40,60), random.uniform(40,60)]
    else: 
        random_array = [random.uniform(0,100), random.uniform(0,100)]
    particles.append(random_array)


weights = generate_weights(size)

# Set up the figure and axes
fig, ax = plt.subplots(1, 4, figsize=(10, 4))

# Plot the initial particles
x_initial = [particle[0] for particle in particles]
y_initial = [particle[1] for particle in particles]
ax[0].scatter(x_initial, y_initial, color='blue', marker='o', alpha=0.2, s=100, label='Initial')
ax[0].set_xlabel('X')
ax[0].set_ylabel('Y')
ax[0].grid(True)
num_iterations = 3

for iteration in range(num_iterations):
    resampled_particles = resampler(particles, weights)
    particles = resampled_particles

    # Plot each resampled particle individually with a different color and marker shape
    
    x_resampled = [particle[0] for particle in resampled_particles]
    y_resampled = [particle[1] for particle in resampled_particles]
    ax[iteration + 1].scatter(x_resampled, y_resampled,  color='blue', alpha=0.2,s=100, label='Iteration {}'.format(iteration + 1))

    ax[iteration + 1].set_xlabel('X')
    ax[iteration + 1].set_ylabel('Y')
    ax[iteration + 1].grid(True)
    weights=generate_weights(size)

fig.suptitle("Resampler", y=-0.9)

plt.tight_layout()
plt.show()
