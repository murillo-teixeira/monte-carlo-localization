import math
import random
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

def motion_model_odometry(particles, u, alpha):
    new_particles = []
    
    for particle in particles:
        x = particle[0]
        y = particle[1]
        theta = particle[2]
        
        delta_rot1 = math.atan2(u[1], u[0]) - math.atan2(x, y)
        delta_trans = math.sqrt((u[0] - x)**2 + (u[1] - y)**2)
        delta_rot2 = u[2] - theta - delta_rot1
        
        delta_rot1_hat = delta_rot1 - random.gauss(0, alpha[0]*abs(delta_rot1) + alpha[1]*delta_trans)
        delta_trans_hat = delta_trans - random.gauss(0, alpha[2]*delta_trans + alpha[3]*(abs(delta_rot1) + abs(delta_rot2)))
        delta_rot2_hat = delta_rot2 - random.gauss(0, alpha[0]*abs(delta_rot2) + alpha[1]*delta_trans)
        
        x_hat = x + delta_trans_hat * math.cos(theta + delta_rot1_hat)
        y_hat = y + delta_trans_hat * math.sin(theta + delta_rot1_hat)
        theta_hat = theta + delta_rot1_hat + delta_rot2_hat
        
        new_particles.append([x_hat, y_hat, theta_hat])
    
    return new_particles

# Generate some example particles
num_particles = 100
particles = [[random.uniform(0, 1), random.uniform(0, 0.5), 0]
             for _ in range(num_particles)]

# Example odometry input
u = [1.0, 2.0, math.pi / 4]

# Example alpha values
alpha = [0.01, 0.01, 0.01, 0.01]

# Set up the figure and axes
fig, ax = plt.subplots(figsize=(8, 6))

# Define a color map for different iterations
colors = list(mcolors.TABLEAU_COLORS.keys())

# Plot the initial particles
x_initial = [particle[0] for particle in particles]
y_initial = [particle[1] for particle in particles]
theta_initial = [particle[2] for particle in particles]
ax.quiver(x_initial, y_initial, [0.01*math.cos(theta) for theta in theta_initial],
          [0.01*math.sin(theta) for theta in theta_initial], scale=10, color='black', label='Initial')

# Iterate and plot each motion model update
for i in range(1, 6):
    # Compute the new particles using the motion model
    new_particles = motion_model_odometry(particles, u, alpha)

    # Extract x, y, and theta coordinates for plotting
    x_after = [particle[0] for particle in new_particles]
    y_after = [particle[1] for particle in new_particles]
    theta_after = [particle[2] for particle in new_particles]

    # Plot the particles after motion update
    color = colors[i-1]
    ax.quiver(x_after, y_after, [math.cos(theta) for theta in theta_after], [math.sin(theta) for theta in theta_after],
              angles='xy', scale_units='xy', scale=10, color=color, label='Iteration {}'.format(i))

    # Set the updated particles as the input for the next iteration
    particles = new_particles


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_xlim([0, 10])
ax.set_ylim([0, 10])
ax.legend()
ax.set_title('Motion Model Odometry')
ax.grid(True)
plt.show()
