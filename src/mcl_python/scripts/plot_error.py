import matplotlib.pyplot as plt
import numpy as np

output_file = "/home/murillo/catkin_ws/src/monte-carlo-localization/output/elevator/local/2023_06_18_23_55_38.csv"

arr = np.loadtxt(output_file,
                 delimiter=",", dtype=float)

times = (arr[:, 0] - arr[0, 0])/1e9
x_mcl = arr[:, 1]
y_mcl = arr[:, 2]
theta_mcl = np.arccos(np.cos(arr[:, 3]))

x_amcl = -arr[:, 4] + 992*0.05
y_amcl = -arr[:, 5] + 992*0.05
theta_amcl = np.arccos(-np.cos(arr[:, 6]))

distance = np.hypot(x_mcl - x_amcl, y_mcl - y_amcl)


plt.subplot(211)
plt.plot(times, distance, c = "#0072BD")
plt.xticks([])
plt.ylabel("Erro translacional (m)")

plt.subplot(212)
plt.plot(times, abs(theta_mcl - theta_amcl)*180/np.pi, c = "#0072BD")
plt.xlabel("Tempo (s)")
plt.ylabel("Erro angular (dg)")

plt.show()
