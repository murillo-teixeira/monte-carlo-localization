import numpy as np
import matplotlib.pyplot as plt
X, Y = np.meshgrid(np.linspace(0, 100), np.linspace(0, 100))
q = plt.quiver(X, Y , np.random.random(100), np.random.random(100))
plt.draw()
plt.pause(2)
q.set_offsets(q.get_offsets() * np.array([1, .5]))
plt.draw()
plt.pause(2)