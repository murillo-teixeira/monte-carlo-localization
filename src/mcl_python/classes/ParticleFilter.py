import numpy as np
from classes.Particle import Particle
from classes.Map import Map

class ParticleFilter:
    def __init__(self, map : Map):
        self.particles = None
        self.map = map

        self.x_sensor = 0
        self.y_sensor = 0

        self.zhit  = 1
        self.zrand = 1
        self.zmax  = 1


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

    def reset_particle_weights(self):
        for particle in self.particles:
            particle.reset_weight()

    def likelihood_field_algorithm(self, laser_msg):
        for i, particle in enumerate(self.particles):
            q = 1
            for i, range in enumerate(laser_msg.ranges):
                current_angle = laser_msg.angle_min + i*laser_msg.angle_increment
                x_meas = (particle.x + \
                    self.x_sensor*np.cos(particle.theta) - \
                    self.y_sensor*np.sin(particle.theta) + \
                    range/0.05*np.cos(particle.theta + current_angle))
                y_meas = (particle.y + \
                    self.y_sensor*np.cos(particle.theta) + \
                    self.x_sensor*np.sin(particle.theta) + \
                    range/0.05*np.sin(particle.theta + current_angle))
                q = q*(self.zhit + self.zrand/self.zmax)
            particle.set_weight(q)

