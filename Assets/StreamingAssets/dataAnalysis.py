import numpy as np
import matplotlib.pyplot as plt


data = np.genfromtxt("export.csv", delimiter=",", skip_header=1)


plt.title("Speed as a function of time")
plt.plot(data[:, 0], data[:, 1])
plt.show()