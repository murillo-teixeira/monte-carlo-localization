import numpy as np

class ParticleSet(np.ndarray):

    def __new__(cls, number_of_particles):
        obj = np.zeros((4, number_of_particles)).view(cls)
        obj.number_of_particles = number_of_particles
        return obj
    
    def get_x_positions(self):
        return self[0, :]
    
    def get_y_positions(self):
        return self[0, :]

thing = ParticleSet(500)

print(thing)
print(thing.get_x_positions())

# thing = np.append(thing.T)
# print(thing.shape)


# a = np.array([[0, 0], [1, 1]])
# b = np.array([[3, 3]])
# print(a)
# a = np.insert(a, 2, b, axis=1)
# print(a)
