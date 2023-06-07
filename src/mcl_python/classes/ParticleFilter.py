import math
import random
import numpy as np

from classes.ParticleSet import ParticleSet
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
        # Initiating a ParticleSet object (a np.ndmatix)
        self.particles = ParticleSet(number_of_particles)

        # Finding the valid positions to positions the particles
        indices = np.argwhere(self.map.map_matrix == 1)
        num_indices = indices.shape[0]

        # Calculate random positions from the valid ones
        random_indices = np.random.choice(num_indices, size=number_of_particles)
        
        # Updating the positions
        sampled_positions = indices[random_indices].T
        sampled_positions[[1, 0], :] = sampled_positions[[0, 1], :]
        self.particles.set_positions(sampled_positions*self.map.resolution)

        # Calculating random orientations for the particles (0, 2*pi)
        orientations = np.random.uniform(0, 2*np.pi, number_of_particles)
        self.particles.set_orientations(orientations)

    def likelihood_field_algorithm(self, laser_msg):
        # Selecting only a subset of the readings
        number_of_particles = self.particles.number_of_particles
        subsampling_step = 10

        measurement_angles = np.tile(np.arange(laser_msg.angle_min, laser_msg.angle_max + laser_msg.angle_increment, laser_msg.angle_increment)[0:-1:subsampling_step], (number_of_particles, 1))
        measurement_ranges = np.array(laser_msg.ranges)[0:-1:subsampling_step]
        valid_readings = ((np.array(laser_msg.ranges) > laser_msg.range_min) & (np.array(laser_msg.ranges) < laser_msg.range_max))[0:-1:subsampling_step]

        measurement_ranges = np.compress(valid_readings, measurement_ranges)
        measurement_angles = np.compress(valid_readings, measurement_angles, axis=1)
        
        particles_x_array = self.particles.x_positions.reshape(1, number_of_particles)
        particles_y_array = self.particles.y_positions.reshape(1, number_of_particles)
        particles_theta_array = self.particles.orientations.reshape(1, number_of_particles)

        x_meas = particles_x_array.T + measurement_ranges*np.cos(measurement_angles + particles_theta_array.T)
        y_meas = particles_y_array.T + measurement_ranges*np.sin(measurement_angles + particles_theta_array.T)

        x_meas_to_field = (x_meas/self.map.resolution).astype(int)
        y_meas_to_field = (y_meas/self.map.resolution).astype(int)

        meas_likelihood = np.sum(self.zhit*self.map.get_meas_likelihood(x_meas_to_field, y_meas_to_field) + self.zrand, axis=1)
        weights = meas_likelihood 
        # weights = weights/np.sum(weights)
        self.particles.set_weights(weights)

    def motion_model_odometry(self, u, alpha):
        delta_rot1 = np.arctan2(u[1], u[0]) - u[3]
        delta_trans = np.hypot(u[0], u[1])
        delta_rot2 = u[2] - delta_rot1

        delta_rot1_hat = delta_rot1 - random.gauss(0, alpha[0]*abs(delta_rot1) + alpha[1]*delta_trans)
        delta_trans_hat = delta_trans - random.gauss(0, alpha[2]*delta_trans + alpha[3]*(abs(delta_rot1) + abs(delta_rot2)))
        delta_rot2_hat = delta_rot2 - random.gauss(0, alpha[0]*abs(delta_rot2) + alpha[1]*delta_trans)
        
        self.particles.update_attr()
        self.particles.set_x_positions(self.particles.x_positions + delta_trans_hat * np.cos(self.particles.orientations + delta_rot1_hat))
        self.particles.set_y_positions(self.particles.y_positions + delta_trans_hat * np.sin(self.particles.orientations + delta_rot1_hat))
        self.particles.set_orientations(self.particles.orientations + delta_rot1_hat + delta_rot2_hat)

    def resampler(self):
        self.particles.update_attr()
        number_of_particles = self.particles.number_of_particles
        weights = self.particles.weights
        new_particles = ParticleSet(number_of_particles)

        n_eff = 1/(np.sum(weights**2))
        if n_eff < 0.95*number_of_particles:
            print('resampling ', n_eff)
            r = random.uniform(0, 1/number_of_particles)
            previous_particles = self.particles.copy()

            c = self.particles.weights[0]
            i = 1
            
            for m in range(number_of_particles):
                u = r + m/number_of_particles
                while (u > c) and (i < number_of_particles - 1):
                    i += 1
                    c += self.particles.weights[i]
                # print(self.particles)
                # print(previous_particles.get_particle(i))
                new_particles.set_particle(m, previous_particles.get_particle(i))
                # print(new_particles)
            self.particles = new_particles
        else:
            print('not resampling ', n_eff)

    def remove_outside_map_particles(self):
        self.particles.update_attr()
        number_of_particles = self.particles.number_of_particles
        particles_x_array = self.particles.x_positions.reshape(1, number_of_particles)
        particles_y_array = self.particles.y_positions.reshape(1, number_of_particles)
        particles_x_array_to_field = (particles_x_array/self.map.resolution).astype(int)
        particles_y_array_to_field = (particles_y_array/self.map.resolution).astype(int)
        print('particles inside:', np.sum((self.map.map_matrix[particles_y_array_to_field,particles_x_array_to_field] == 1)))
        is_particle_inside_map = (self.map.map_matrix[particles_y_array_to_field,particles_x_array_to_field] == 1)
        weights = self.particles.weights*is_particle_inside_map
        self.particles.set_weights(weights)

    
    def normalize_weights(self):
        self.particles.update_attr()
        weights = self.particles.weights
        if np.sum(weights) != 0:
            weights = weights/np.sum(weights)
        self.particles.set_weights(weights)

    def get_n_eff(self):
        self.particles.update_attr()
        number_of_particles = self.particles.number_of_particles
        weights = self.particles.weights
        n_eff = number_of_particles
        if np.sum(weights) != 0:
            n_eff = 1/(np.sum(weights**2))
        return n_eff, number_of_particles
