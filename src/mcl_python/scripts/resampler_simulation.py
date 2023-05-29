import random
import matplotlib.pyplot as plt

def resampler(particles, weights):
    number_of_particles = len(particles)
    new_particles = []
    r = random.uniform(0, number_of_particles)
    c = weights[0]
    i = 1

    for m in range(number_of_particles):
        u = r + (m - 1)/number_of_particles
        while u > c:
            i += 1
            c += weights[i]
        
        new_particles.append(particles[i-1])
    
    return new_particles

# Example particles and weights
particles = [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10]]
weights = [0.1, 0.2, 0.4, 0.1, 0.2]

# Set up the figure and axes
fig, ax = plt.subplots(figsize=(8, 6))

# Plot the initial particles
x_initial = [particle[0] for particle in particles]
y_initial = [particle[1] for particle in particles]
ax.scatter(x_initial, y_initial, color='blue', label='Initial')

# Perform multiple iterations of resampling
num_iterations = 5

for iteration in range(num_iterations):
    resampled_particles = resampler(particles, weights)
    particles = resampled_particles

    # Extract x and y coordinates for plotting
    x_resampled = [particle[0] for particle in resampled_particles]
    y_resampled = [particle[1] for particle in resampled_particles]
    
    # Plot the resampled particles
    ax.scatter(x_resampled, y_resampled, label='Iteration {}'.format(iteration + 1))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Resampler')
ax.legend()
ax.grid(True)
plt.show()
