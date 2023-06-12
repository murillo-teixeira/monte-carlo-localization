import numpy as np
import math
import random
from classes.Particle import Particle
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

def update_particles(quiver_plot, particles):
    X = np.array([int(particle.x) for particle in particles])
    Y = np.array([int(particle.y/0.05) for particle in particles])
    U = [0.1*np.cos(particle.theta) for particle in particles], 
    V = [0.1*np.sin(particle.theta) for particle in particles], 
    quiver_plot.set_offsets(np.array([X.flatten(), Y.flatten()]).T)
    quiver_plot.set_UVC(U, V)


# Generate some example particles
num_particles = 100
particles_r = np.random.normal(0, 0.5, num_particles)
particles_alpha = np.random.uniform(0, 2*np.pi, num_particles)
particles_theta = np.random.normal(np.pi/2, np.pi/5, num_particles)
particles = [
    Particle(
        particles_r[i]*np.cos(particles_alpha[i]),
        particles_r[i]*np.sin(particles_alpha[i]),
        particles_theta[i]
    ) for i in range(num_particles)]

plt.quiver(
    [particle.x for particle in particles], 
    [particle.y for particle in particles], 
    [0.1*np.cos(particle.theta) for particle in particles], 
    [0.1*np.sin(particle.theta) for particle in particles], color='red', angles='xy', pivot='mid')

robot_particle = Particle(0, 0, np.pi/2)
plt.quiver(
    [robot_particle.x], 
    [robot_particle.y], 
    [0.1*np.cos(robot_particle.theta)], 
    [0.1*np.sin(robot_particle.theta)], color='blue', angles='xy', pivot='mid')

plt.show()