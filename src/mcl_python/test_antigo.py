import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from classes.Particle import Particle
import math
import random

u = [0, 0.5, 0, 0]
alpha = [0.00, 0.00, 0.00, 0.00]

def motion_model_odometry(particles, u, alpha):
    new_particles = []
    
    for particle in particles:
        x = particle[0]
        y = particle[1]
        theta = particle[2]
        
        delta_rot1 = math.atan2(u[1], u[0]) - theta
        delta_trans = math.sqrt((u[0])**2 + (u[1])**2)
        delta_rot2 = u[2] - delta_rot1

        delta_rot1_hat = delta_rot1 - random.gauss(0, alpha[0]*abs(delta_rot1) + alpha[1]*delta_trans)
        delta_trans_hat = delta_trans - random.gauss(0, alpha[2]*delta_trans + alpha[3]*(abs(delta_rot1) + abs(delta_rot2)))
        delta_rot2_hat = delta_rot2 - random.gauss(0, alpha[0]*abs(delta_rot2) + alpha[1]*delta_trans)
        
        # delta_rot1_hat = delta_rot1
        # delta_trans_hat = delta_trans
        # delta_rot2_hat = delta_rot2
        
        x_hat = x + delta_trans_hat * math.cos(theta + delta_rot1_hat)
        y_hat = y + delta_trans_hat * math.sin(theta + delta_rot1_hat)
        theta_hat = theta + delta_rot1_hat + delta_rot2_hat
        
        new_particles.append([x_hat, y_hat, theta_hat])
    
    return new_particles


fig, ax = plt.subplots(1,1)

# Generate some example particles
num_particles = 20
particles_r = np.random.normal(0, 0.5, num_particles)
particles_alpha = np.random.uniform(0, 2*np.pi, num_particles)
particles_theta = np.random.normal(np.pi/2, np.pi/3, num_particles)
particles = [
    Particle(
        particles_r[i]*np.cos(particles_alpha[i]),
        particles_r[i]*np.sin(particles_alpha[i]),
        particles_theta[i]
    ) for i in range(num_particles)]

Q = ax.quiver(
    [particle.x for particle in particles], 
    [particle.y for particle in particles], 
    [0.001*np.cos(particle.theta) for particle in particles], 
    [0.001*np.sin(particle.theta) for particle in particles], color='red', angles='xy', pivot='mid', width=0.005)

robot_particle = Particle(0, 0, np.pi/2)

Q2 = ax.quiver(
    [robot_particle.x], 
    [robot_particle.y], 
    [0.1*np.cos(robot_particle.theta)], 
    [0.1*np.sin(robot_particle.theta)], color='blue', angles='xy', pivot='mid', width=0.005)

total_frames = 20

def update_quiver(num, Q, Q2, particles):
    ax.cla()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-1, 10)

    robot_particle.x = u[0]*num
    robot_particle.y = u[1]*num

    particles = motion_model_odometry(particles, u, [0.00, 0.00, 0.00, 0.00])

    Q = ax.quiver(
        [particle.x for particle in particles], 
        [particle.y for particle in particles], 
        [0.001*np.cos(particle.theta) for particle in particles], 
        [0.001*np.sin(particle.theta) for particle in particles], color='red', angles='xy', pivot='mid', width=0.005)

    Q2 = ax.quiver(
        [robot_particle.x], 
        [robot_particle.y], 
        [0.1*np.cos(robot_particle.theta)], 
        [0.1*np.sin(robot_particle.theta)], color='blue', angles='xy', pivot='mid', width=0.005)

    return Q, Q2,

# you need to set blit=False, or the first set of arrows never gets
# cleared on subsequent frames
anim = animation.FuncAnimation(fig, update_quiver, fargs=(Q, Q2, particles),
                               frames=total_frames, interval=50, blit=False)
fig.tight_layout()
plt.show()