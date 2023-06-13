import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

x, y = np.mgrid[0:1984:1, 0:1984:1]
pos = np.dstack((x, y))
rv = multivariate_normal([992, 992], [[100000, 0], [0, 100000]])
values = rv.pdf(pos)
print(values)
# plt.imshow(values)
# plt.show()