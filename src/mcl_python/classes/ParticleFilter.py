import math
import random
import numpy as np
from classes.Particle import Particle
from classes.Map import Map

class ParticleFilter:
    def __init__(self, map : Map):
        self.particles = None
        self.map = map

        self.x_sensor = 0
        self.y_sensor = 0

        self.zhit  = 0.7
        self.zrand = 0.05
        self.zmax  = 0.25


    def initialize_particles(self, number_of_particles = 50):
        self.particles = []
        for _ in range(number_of_particles):
            is_position_valid = False
            while not is_position_valid:
                x = np.random.randint(0, self.map.roi_xmax)
                y = np.random.randint(0, self.map.roi_ymax)
                if self.map.map_matrix[y][x] == 1.0:
                    is_position_valid = True
                    x = x*0.05
                    y = y*0.05
                    theta = np.random.uniform(0, 2*np.pi)

            self.particles.append(Particle(x, y, theta))

    def reset_particle_weights(self):
        for particle in self.particles:
            particle.reset_weight()

    def likelihood_field_algorithm(self, laser_msg):
        weights = []
        for i, particle in enumerate(self.particles):
            q = 1
            for i, range in enumerate(laser_msg.ranges):
                if range and (laser_msg.range_min < range < laser_msg.range_max):
                    current_angle = laser_msg.angle_min + i*laser_msg.angle_increment
                    x_meas = (particle.x + \
                        self.x_sensor*np.cos(particle.theta) - \
                        self.y_sensor*np.sin(particle.theta) + \
                        range/0.05*np.cos(particle.theta + current_angle))
                    y_meas = (particle.y + \
                        self.y_sensor*np.cos(particle.theta) + \
                        self.x_sensor*np.sin(particle.theta) + \
                        range/0.05*np.sin(particle.theta + current_angle))
                    q = q*(self.zhit*self.map.likelihood_field[int(y_meas), int(x_meas)] + self.zrand/self.zmax)
            weights.append(-np.log(q, where = q > 0))
        
        # Maybe normalize?
        # total = sum(weights)
        # weights = np.array(weights)/total
        
        for i, particle in enumerate(self.particles):
            particle.set_weight(weights[i])

    def motion_model_odometry(self, u, alpha):
        new_particles = []
        
        for particle in self.particles:
            x = particle.x
            y = particle.y
            theta = particle.theta
            
            delta_rot1 = math.atan2(u[1], u[0]) - math.atan2(x, y)
            delta_trans = np.linalg.norm(u[0], u[1])
            delta_rot2 = u[2] - theta - delta_rot1
            
            # delta_rot1_hat = delta_rot1 - random.gauss(0, alpha[0]*abs(delta_rot1) + alpha[1]*delta_trans)
            # delta_trans_hat = delta_trans - random.gauss(0, alpha[2]*delta_trans + alpha[3]*(abs(delta_rot1) + abs(delta_rot2)))
            # delta_rot2_hat = delta_rot2 - random.gauss(0, alpha[0]*abs(delta_rot2) + alpha[1]*delta_trans)
            
            delta_rot1_hat = delta_rot1
            delta_trans_hat = delta_trans
            delta_rot2_hat = delta_rot2
            
            x_hat = x + delta_trans_hat * math.cos(theta + delta_rot1_hat)
            y_hat = y + delta_trans_hat * math.sin(theta + delta_rot1_hat)
            theta_hat = theta + delta_rot1_hat + delta_rot2_hat
            
            new_particles.append(Particle(x_hat, y_hat, theta_hat))

        self.particles = new_particles

