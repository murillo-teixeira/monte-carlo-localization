import matplotlib.pyplot as plt
import numpy as np

# output_files = [
#     "/home/murillo/catkin_ws/src/monte-carlo-localization/output/lab/global/2023_06_19_13_20_56.csv",
#     "/home/murillo/catkin_ws/src/monte-carlo-localization/output/lab/local/2023_06_19_13_17_25.csv",
#     ]

output_files = [
    "/home/murillo/catkin_ws/src/monte-carlo-localization/output/5th_floor/global/2023_06_18_19_36_47.csv",
    "/home/murillo/catkin_ws/src/monte-carlo-localization/output/5th_floor/local/2023_06_18_19_24_15.csv",
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

plt.suptitle('Erro de localização para bag no mapa do 5º piso')

plt.subplot(211)
plt.legend(['Global localization', 'Pose tracking'], loc=1)
plt.subplot(212)
plt.legend(['Global localization', 'Pose tracking'], loc=1)
plt.savefig(output_file.replace('.csv', '.png'))
plt.show()
