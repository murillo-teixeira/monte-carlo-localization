import math
import random
import numpy as np
from classes.Particle import Particle
from classes.Map import Map

class ParticleFilter:
    def __init__(self, map : Map, zhit, zrand, x_sensor, y_sensor):
        self.particles = None
        self.map = map

        self.x_sensor = x_sensor
        self.y_sensor = y_sensor

        self.zhit  = zhit
        self.zrand = zrand


    def initialize_particles(self, number_of_particles = 50):
        self.particles = []
        for _ in range(number_of_particles):
            is_position_valid = False
            while not is_position_valid:
                x = np.random.randint(0, self.map.roi_xmax)
                y = np.random.randint(0, self.map.roi_ymax)
                if self.map.map_matrix[y][x] == 1.0:
                    is_position_valid = True
                    x = x*self.map.resolution
                    y = y*self.map.resolution
                    theta = np.random.uniform(0, 2*np.pi)

            self.particles.append(Particle(x, y, theta))

    def reset_particle_weights(self):
        for particle in self.particles:
            particle.reset_weight()

    def likelihood_field_algorithm(self, laser_msg):
        weights = []
        measurement_angles = np.arange(laser_msg.angle_min, laser_msg.angle_max + laser_msg.angle_increment, laser_msg.angle_increment)
        for particle in self.particles:
            # The position (x, y) of the measurement of the particle was the robot
            x_meas = particle.x + laser_msg.ranges*np.cos(measurement_angles + particle.theta)
            y_meas = particle.y + laser_msg.ranges*np.sin(measurement_angles + particle.theta)

            # To exclude invalid readings and prepare for lookup likelihood field
            valid_readings = (np.array(laser_msg.ranges) > laser_msg.range_min) & (np.array(laser_msg.ranges) < laser_msg.range_max)
            x_meas_to_field = (x_meas[valid_readings]/self.map.resolution).astype(int)
            y_meas_to_field = (y_meas[valid_readings]/self.map.resolution).astype(int)
            
            # Calculating the weight
            meas_likelihood = self.map.get_meas_likelihood(x_meas_to_field, y_meas_to_field)
            weight = np.sum(self.zhit*meas_likelihood + self.zrand)
            weights.append(weight)    
        
        weights_sum = np.sum(weights)
        weights = np.array(weights)/weights_sum

        for particle, weight in zip(self.particles, weights):
            particle.weight = weight
            # print(particle.weight)

    def motion_model_odometry(self, u, alpha):
        new_particles = []
        
        for particle in self.particles:
            x = particle.x
            y = particle.y
            theta = particle.theta
            
            delta_rot1 = math.atan2(u[1], u[0]) - u[3]
            delta_trans = math.sqrt((u[0])**2 + (u[1])**2)
            delta_rot2 = u[2] - delta_rot1

            delta_rot1_hat = delta_rot1 - random.gauss(0, alpha[0]*abs(delta_rot1) + alpha[1]*delta_trans)
            delta_trans_hat = delta_trans - random.gauss(0, alpha[2]*delta_trans + alpha[3]*(abs(delta_rot1) + abs(delta_rot2)))
            delta_rot2_hat = delta_rot2 - random.gauss(0, alpha[0]*abs(delta_rot2) + alpha[1]*delta_trans)

            x_hat = x + delta_trans_hat * math.cos(theta + delta_rot1_hat)
            y_hat = y + delta_trans_hat * math.sin(theta + delta_rot1_hat)
            theta_hat = theta + delta_rot1_hat + delta_rot2_hat
            
            new_particles.append(Particle(x_hat, y_hat, theta_hat))

        self.particles = new_particles

    def resampler(self):
        number_of_particles = len(self.particles)
        new_particles = []
        r = random.uniform(0, 1/number_of_particles)
    
        i = 0
        c = self.particles[0].weight
        
        for m in range(number_of_particles):
            i = 1
            u = r + (m)/number_of_particles
            while (u > c) and (i < number_of_particles - 1):
                i += 1
                c += self.particles[i].weight
            new_particles.append(self.particles[i])

        self.particles = new_particles
