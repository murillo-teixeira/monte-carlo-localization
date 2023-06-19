import matplotlib.pyplot as plt
import numpy as np

output_files = [
    "/home/pberna/catkin_ws/src/monte-carlo-localization/output/elevator/100.csv",
    "/home/pberna/catkin_ws/src/monte-carlo-localization/output/elevator/500.csv",
    "/home/pberna/catkin_ws/src/monte-carlo-localization/output/elevator/1000.csv",
    "/home/pberna/catkin_ws/src/monte-carlo-localization/output/elevator/5000.csv",
    "/home/pberna/catkin_ws/src/monte-carlo-localization/output/elevator/10000.csv",
    "/home/pberna/catkin_ws/src/monte-carlo-localization/output/elevator/50000.csv",
    ]

for output_file in output_files:
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
    plt.plot(times, distance)
    # plt.xticks([])
    plt.ylabel("Erro translacional (m)")

    plt.subplot(212)
    plt.plot(times, abs(theta_mcl - theta_amcl)*180/np.pi)
    plt.xlabel("Tempo (s)")
    plt.ylabel("Erro angular (dg)")

plt.suptitle('Convergêcia de resultados com o AMCL')

plt.subplot(211)
plt.legend(['100 partículas', '500 partículas', '1000 partículas', '5000 partículas', '10000 partículas', '50000 partículas'], loc=1)
plt.subplot(212)
plt.legend(['100 partículas', '500 partículas', '1000 partículas', '5000 partículas', '10000 partículas', '50000 partículas'], loc=1)
plt.savefig(output_file.replace('.csv', '.png'))
plt.show()