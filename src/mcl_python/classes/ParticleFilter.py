import math
import random
import numpy as np
from classes.Particle import Particle
from classes.Map import Map

class ParticleFilter:
    def __init__(self, map : Map):
        self.particles = None
        self.map = map
        
    def initialize_particles(self, number_of_particles = 50):
        self.particles = []
        for _ in range(number_of_particles):
            is_position_valid = False
            while not is_position_valid:
                x = np.random.randint(0, self.map.roi_xmax)
                y = np.random.randint(0, self.map.roi_ymax)
                if self.map.map_matrix[y][x] == 1.0:
                    is_position_valid = True
                    theta = np.random.uniform(0, 360)
            self.particles.append(Particle(x, y, theta))

    def motion_model_odometry(self, u, alpha):
        new_particles = []
        
        for particle in self.particles:
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

