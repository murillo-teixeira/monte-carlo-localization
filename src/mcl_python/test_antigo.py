import numpy as np

x = np.array([[1, 2, 3, 4], [1, 2, 3, 4]])
bol = (x == 1)
print(bol)
print(x[bol])