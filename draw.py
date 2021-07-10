# Download the data folder from https://github.com/shushuai3/nmpc-mav/files/6795513/data.zip
# change "data/rand" or "data/nmpc" to plot the localization error
import numpy as np
import matplotlib.pyplot as plt

# Comparision of 20 tests with random initial states and sensor noise
f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
plt.margins(x=0)
for fileNum in range(30):
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

# Time distribution of the localization convergence
# time = [[], [], []]
# for fileNum in range(30):
#     data = np.loadtxt("data/nmpc{:02d}.txt".format(fileNum+1))
#     data_len = len(data[:, 0])
#     for j in range(3):
#         for i in range(data_len-1, 0, -1):
#             err = data[i, 1+j*2]-data[i, j*2]
#             if j==2:
#                 if abs(np.cos(err%3.14)) < 0.97:
#                     time[j].append(i/100.0)
#                     break
#             else:
#                 if err > 0.5:
#                     time[j].append(i/100.0)
#                     break
# data_nmpc = time
# time = [[], [], []]
# for fileNum in range(30):
#     data = np.loadtxt("data/rand{:02d}.txt".format(fileNum+1))
#     data_len = len(data[:, 0])
#     for j in range(3):
#         for i in range(data_len-1, 0, -1):
#             err = data[i, 1+j*2]-data[i, j*2]
#             if j==2:
#                 if abs(np.cos(err%3.14)) < 0.97:
#                     time[j].append(i/100.0)
#                     break
#             else:
#                 if err > 0.7:
#                     time[j].append(i/100.0)
#                     break
# data_rand = time
# labels = ['x_nmpc', 'x_rand', 'y_nmpc', 'y_rand', r'$\psi$_nmpc', r'$\psi$_rand']
# data_total = np.array([data_nmpc[0], data_rand[0], data_nmpc[1], data_rand[1],
#     data_nmpc[2], data_rand[2]])
# fig, ax = plt.subplots(nrows=1, ncols=1)
# # bplot1 = ax1.boxplot(data_nmpc, vert=True, patch_artist=True, labels=labels)
# bplot = ax.boxplot(data_total, vert=True, patch_artist=True, labels=labels)
# colors = ['lightblue', 'lightgreen', 'lightblue', 'lightgreen', 'lightblue', 'lightgreen']
# for patch, color in zip(bplot['boxes'], colors):
#     patch.set_facecolor(color)
# ax.set_ylabel('Convergence Time (s)')
# plt.show()

# Localization error after convergence
# x_rand, y_rand = [], []
# for fileNum in range(30):
#     data = np.loadtxt("data/rand{:02d}.txt".format(fileNum+1))
#     for i in range(3000, 4000, 1):
#         x_rand.append(data[i, 1] - data[i, 0])
#         y_rand.append(data[i, 3] - data[i, 2])
# x_nmpc, y_nmpc = [], []
# for fileNum in range(30):
#     data = np.loadtxt("data/nmpc{:02d}.txt".format(fileNum+1))
#     for i in range(3000, 4000, 1):
#         x_nmpc.append(data[i, 1] - data[i, 0])
#         y_nmpc.append(data[i, 3] - data[i, 2])
# plt.style.use('seaborn-white')
# kwargs = dict(histtype='step', stacked=True, fill=False, bins=5)
# print(len(x_rand), len(x_nmpc))
# plt.hist(x_rand, **kwargs, label="err_x_rand")
# plt.hist(y_rand, **kwargs, label="err_y_rand")
# plt.hist(x_nmpc, **kwargs, label="err_x_nmpc")
# plt.hist(y_nmpc, **kwargs, label="err_y_nmpc")
# plt.xlabel("Error (m)", fontsize=12)
# plt.ylabel("Number", fontsize=12)
# plt.margins(x=0)
# plt.legend(fontsize=12)
# plt.grid()
# plt.show()

# Plot the trajectory of formation flight
# f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
# plt.margins(x=0)
# for fileNum in range(30):
#     data = np.loadtxt("data/rand{:02d}.txt".format(fileNum+1))
#     timePlot = np.arange(0, len(data[:, 0]))/100
#     ax1.plot(timePlot, data[:, 1] - data[:, 0])
#     ax2.plot(timePlot, data[:, 3] - data[:, 2])
#     ax3.plot(timePlot, data[:, 5] - data[:, 4])
# ax1.axhline(y=0, color='r', linestyle='-')
# ax2.axhline(y=0, color='r', linestyle='-')
# ax3.axhline(y=0, color='r', linestyle='-')
# ax1.grid(True)
# ax2.grid(True)
# ax3.grid(True)
# ax1.set_ylabel(r"Err $x_{ij}$ (m)", fontsize=12)
# ax2.set_ylabel(r"Err $y_{ij}$ (m)", fontsize=12)
# ax3.set_ylabel(r"Err $\mathrm{\psi}_{ij}$ (rad)", fontsize=12)
# ax3.set_xlabel("Time (s)", fontsize=12)
# f.subplots_adjust(hspace=0)
# plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
# plt.show()

# Plot the world-frame trajectory of both robots
# data = np.loadtxt("data/nmpc_world.txt")
# data = data[1000:2000, :]
# timePlot = np.arange(0, len(data[:, 0]))/100
# plt.plot(data[:, 0], data[:, 2], label=r"$i^{\rm{th}}$ MAV position")
# plt.plot(data[:, 6], data[:, 8], label=r"$j^{\rm{th}}$ MAV position")
# plt.xlabel("X (m)", fontsize=12)
# plt.ylabel("Y (m)", fontsize=12)
# plt.legend(fontsize=12)
# plt.grid()
# plt.show()