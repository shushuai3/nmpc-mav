# data plot
import numpy as np
import matplotlib.pyplot as plt

# Comparision of 20 tests with random initial states and sensor noise
f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
plt.margins(x=0)
for fileNum in range(20):
    data = np.loadtxt("data/rand{:02d}.txt".format(fileNum+1))
    timePlot = np.arange(0, len(data[:, 0]))/100
    ax1.plot(timePlot, data[:, 1] - data[:, 0])
    ax2.plot(timePlot, data[:, 3] - data[:, 2])
    ax3.plot(timePlot, data[:, 5] - data[:, 4])
ax1.axhline(y=0, color='r', linestyle='-')
ax2.axhline(y=0, color='r', linestyle='-')
ax3.axhline(y=0, color='r', linestyle='-')
ax1.grid(True)
ax2.grid(True)
ax3.grid(True)
ax1.set_ylabel(r"Err $x_{ij}$ (m)", fontsize=12)
ax2.set_ylabel(r"Err $y_{ij}$ (m)", fontsize=12)
ax3.set_ylabel(r"Err $\mathrm{\psi}_{ij}$ (rad)", fontsize=12)
ax3.set_xlabel("Time (s)", fontsize=12)
f.subplots_adjust(hspace=0)
plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
plt.show()