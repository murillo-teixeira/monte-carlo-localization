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

    def plot_particles(self):
        for particle in self.particles:
            particle.plot_particle()