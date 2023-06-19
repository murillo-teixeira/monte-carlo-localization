import numpy as np
from sklearn.cluster import KMeans
from scipy import stats

class ParticleSet(np.ndarray):

    def __new__(cls, number_of_particles):
        obj = np.zeros((4, number_of_particles)).view(cls)
        obj.number_of_particles = number_of_particles
        return obj
    
    def update_attr(self):
        self.x_positions = self[0, :]
        self.y_positions = self[1, :]
        self.orientations = self[2, :]
        self.weights = self[3, :]

    def set_x_positions(self, new_positions):
        self[0, :] = new_positions

    def set_y_positions(self, new_positions):
        self[1, :] = new_positions
    
    def set_orientations(self, new_orientations):
        self[2, :] = new_orientations

    def set_positions(self, new_positions):
        self[0:2, :] = new_positions

    def get_positions(self):
        return self[0:2, :]

    def set_weights(self, new_weights):
        self[3, :] = new_weights

    def get_particle(self, i):
        return self[:, i]

    def set_particle(self, i, particle):
        self[:, i] = particle

    def get_mean_particle(self, percentile):
        self.update_attr()

        clusters = KMeans(n_clusters=4,random_state=0,n_init="auto").fit(self.get_positions().T)

        labels=np.array(clusters.labels_)

        cluster_certo = stats.mode(labels, keepdims=False)[0]

        return [
            np.mean(self.x_positions[labels== cluster_certo]),
            np.mean(self.y_positions[labels== cluster_certo]),
            np.mean(self.orientations[labels== cluster_certo])
        ]
