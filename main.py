"""
Project: Nonlinear model predictive control for multiple aerial robots
         with observability constraint
Author: Shushuai Li, MAVLab, TUDelft
Reference: arxiv link

This file: simulation animation or plot of the relative position and yaw
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Simulation settings
show_animation = False
# seed the random number generator for reproducibility
np.random.seed(20210701)
border = {"xmin":-10, "xmax":10, "ymin":-10, "ymax":10, "zmin":0, "zmax":4}
numRob = 2
dt = 0.01
# simulation time [s]
simTime = 40.0
# maximum velocity [m/s]
maxVel = 2.0
# input deviation: vx[m/s], vy[m/s], yawRate[rad/s]
devInput = np.array([[0.25, 0.25, 0.01]]).T
# ranging deviation [m]
devObser = 0.1
# real interval is ekfStride*dt [s]
ekfStride = 1

# Variables being updated in simulation
# random initial ground-truth of state [x, y, yaw]' of numRob robots
xTrue = np.random.uniform(-3, 3, (3, numRob))
# [x_ij, y_ij, yaw_ij]' of the second robot in the first robot's view
relativeState = np.zeros((3, numRob, numRob))
velocity = np.zeros((3, numRob))

# Random control inputs for all robots
def calcInput_FlyRandom(step):
    global velocity
    if (step % 100) == 0:
        if (step % 200) == 0:
            velocity = -velocity
        else:
            velocity[0:2,:] = np.random.uniform(0, maxVel*2, (2, numRob)) - maxVel
            velocity[2,:] = np.random.uniform(0, 1, (1, numRob)) - 0.5
    if step == 1:
        velocity[0:2,:] = np.random.uniform(0, maxVel*2, (2, numRob)) - maxVel
        velocity[2,:] = np.random.uniform(0, 1, (1, numRob)) - 0.5
    velocity[2, 0:2] = np.zeros((1, 2))
    return velocity

# Relative motion model of two robots for state prediction
def motion_model(x, u):
    xPred = np.zeros((3, numRob))
    for i in range(numRob):
        # X_{k+1} = X_k + Ve * dt; e means earth, b means body
        # Ve = [[c(psi), -s(psi)],[s(psi),c(psi)]] * Vb
        F = np.array([[1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])
        B = np.array([[np.cos(x[2, i]), -np.sin(x[2, i]), 0],
                    [np.sin(x[2, i]),  np.cos(x[2, i]), 0],
                    [0.0, 0.0, 1]])*dt
        xPred[:,i] = F@x[:,i] + B@u[:,i]
    return xPred

# Update the ground-truth robot position and yaw; add noise to sensors
def update(xTrue, u):
    xTrue = motion_model(xTrue, u)
    dTrue = np.zeros((numRob, numRob))
    for i in range(numRob):
        for j in range(numRob):
            dx = xTrue[0, i] - xTrue[0, j]
            dy = xTrue[1, i] - xTrue[1, j]
            dTrue[i, j] = np.sqrt(dx**2 + dy**2)
    randNxN = np.random.randn(numRob, numRob)
    np.fill_diagonal(randNxN, 0)
    zNois = dTrue + randNxN * devObser
    rand3xN = np.random.randn(3, numRob)
    uNois = u + rand3xN * devInput
    return xTrue, zNois, uNois

# Initialization of covariance matrix
Pxy, Pr, Qxy, Qr, Rd = 10, 0.1, 0.25, 0.4, 0.1
Pmatrix = np.zeros((3, 3, numRob, numRob))
for i in range(numRob):
    for j in range(numRob):
        Pmatrix[0:2, 0:2, i, j] = np.eye(2)*Pxy
        Pmatrix[2, 2, i, j] = Pr
# covariance matrix
Q = np.diag([Qxy, Qxy, Qr, Qxy, Qxy, Qr])**2
R = np.diag([Rd])**2

# Extend Kalman filter for relative localization
def EKF(uNois, zNois, relativeState, ekfStride):
    # Calculate relative position between robot i and j in i's horizontal-frame
    dtEKF = ekfStride*0.01
    for i in range(1):
        for j in [jj for jj in range(numRob) if jj!=i]:
            # the relative state Xij = Xj - Xi
            uVix, uViy, uRi = uNois[:, i]
            uVjx, uVjy, uRj = uNois[:, j]
            xij, yij, yawij = relativeState[:, i, j]
            dotXij = np.array([np.cos(yawij)*uVjx - np.sin(yawij)*uVjy - uVix + uRi*yij,
                            np.sin(yawij)*uVjx + np.cos(yawij)*uVjy - uViy - uRi*xij,
                            uRj - uRi])
            statPred = relativeState[:, i, j] + dotXij * dtEKF
            jacoF = np.array([[1,         uRi*dtEKF,  (-np.sin(yawij)*uVjx-np.cos(yawij)*uVjy)*dtEKF],
                            [-uRi*dtEKF,  1,          (np.cos(yawij)*uVjx-np.sin(yawij)*uVjy)*dtEKF ],
                            [0,           0,          1]])
            jacoB = np.array([[-1,  0,  yij,  np.cos(yawij), -np.sin(yawij),  0],
                            [ 0,   -1, -xij,  np.sin(yawij),  np.cos(yawij),  0],
                            [ 0,    0,   -1,              0,              0,  1]])*dtEKF
            PPred = jacoF@Pmatrix[:, :, i, j]@jacoF.T + jacoB@Q@jacoB.T
            xij, yij, yawij = statPred
            zPred = dist = np.sqrt(xij**2 + yij**2)
            jacoH = np.array([[xij/dist, yij/dist, 0]])
            resErr = zNois[i, j] - zPred
            S = jacoH@PPred@jacoH.T + R
            K = PPred@jacoH.T@np.linalg.inv(S)
            relativeState[:, [i], [j]] = statPred.reshape((3,1)) + K@np.array([[resErr]])
            Pmatrix[:, :, i, j] = (np.eye(len(statPred)) - K@jacoH)@PPred
    # print(np.trace(Pmatrix[:, :, i, j]))
    return relativeState

def calcAbsPosUseRelaPosWRTRob0(posRob0, relaPos, xTrue, numRob):
    # Calculate the world-frame position of all robots to robot0 with the relative states
    xEsti = np.zeros((3, numRob))
    for i in range(numRob):
        # xj = R*x0j+x0
        xEsti[0,i] = relaPos[0,0,i]*np.cos(xTrue[2,0]) - relaPos[1,0,i]*np.sin(xTrue[2,0])
        xEsti[1,i] = relaPos[0,0,i]*np.sin(xTrue[2,0]) + relaPos[1,0,i]*np.cos(xTrue[2,0])
        xEsti[2,i] = relaPos[2,0,i]
        xEsti[:,i] = xEsti[:,i] + posRob0
    return xEsti

def calcRelaState(xTrue, numRob):
    # Calculate the relative states by using the position in world-frame
    xRelaGT = np.zeros((3, numRob))
    x0, y0, yaw0 = xTrue[0,0], xTrue[1,0], xTrue[2,0]
    for i in range(numRob):
        # xj = R*x0j+x0
        x0i = xTrue[0, i] - x0
        y0i = xTrue[1, i] - y0
        yaw0i = xTrue[2, i] - yaw0
        xRelaGT[0,i] = x0i*np.cos(yaw0)  + y0i*np.sin(yaw0)
        xRelaGT[1,i] = -x0i*np.sin(yaw0) + y0i*np.cos(yaw0)
        xRelaGT[2,i] = yaw0i
    return xRelaGT

# Acados and optimal problem initialization
from multiRobotModel import multiRobotModel
from multiRobotOpti import MultiRobotOptimizer
model, constraint = multiRobotModel()
tp, N = 1.0, 50
solver = MultiRobotOptimizer(model, constraint, tp, N)
xTrue[2, 0] = 0
xTrue[2, 1] = 1.7
for i in range(N):
    # solver.set(i, 'yref', np.array([0]))
    solver.set(i, 'yref', np.array([0, 1, 1]))
u_history = [[], [], [], []]
t_history = []

def nmpc(xCurrent, step):
    # given the initial states (clip is important for the solver convergence)
    if step > 1500:
        for i in range(N):
            solver.set(i, 'yref', np.array([0, 3*np.cos(step/100.0), 3*np.sin(step/100.0)]))
    xCurrent[0:2] = np.clip(xCurrent[0:2], -4, 4)
    xCurrent[2] = np.clip(xCurrent[2], -15, 15)
    solver.set(0, 'lbx', xCurrent)
    solver.set(0, 'ubx', xCurrent)
    # solve the optimal inputs
    status = solver.solve()
    if status != 0 :
        raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
    uOpti = solver.get(0, 'u')
    u_history[0].append(uOpti[0])
    u_history[1].append(uOpti[1])
    u_history[2].append(uOpti[2])
    u_history[3].append(uOpti[3])
    u = np.zeros((3, numRob))
    u[0:2,0] = uOpti[0:2]
    u[0:2,1] = uOpti[2:4]
    return u

def animate(step):
    global xTrue, relativeState
    u = nmpc(relativeState[:, 0, 1], step)
    # u = calcInput_FlyRandom(step)
    xTrue, zNois, uNois = update(xTrue, u)
    if step % ekfStride == 0:
        relativeState = EKF(uNois, zNois, relativeState, ekfStride)
    xEsti = calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
    pointsTrue.set_data(xTrue[0, :], xTrue[1, :])
    pointsEsti.set_data(xEsti[0, :], xEsti[1, :])
    pointsTrueHead.set_data(xTrue[0, :]+0.07*np.cos(xTrue[2, :]), xTrue[1, :]+0.07*np.sin(xTrue[2, :]))
    pointsEstiHead.set_data(xEsti[0, :]+0.07*np.cos(xEsti[2, :]), xEsti[1, :]+0.07*np.sin(xEsti[2, :]))
    circle.center = (xTrue[0, 0], xTrue[1, 0])
    circle.radius = zNois[0, 1]
    time_text.set_text("t={:.2f}s".format(step * dt))
    return pointsTrue, pointsEsti, circle, pointsTrueHead, pointsEstiHead, time_text

if show_animation:
    # Set up an animation
    fig = plt.figure()
    ax  = fig.add_subplot(111, aspect='equal')
    ax.set(xlim=(border["xmin"], border["xmax"]), ylim=(border["ymin"], border["ymax"]))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    title = ax.set_title('Simulated swarm')
    pointsTrue,  = ax.plot([], [], linestyle="", marker="o", color="b", label="GroundTruth")
    pointsEsti,  = ax.plot([], [], linestyle="", marker="o", color="r", label="Relative EKF")
    pointsTrueHead,  = ax.plot([], [], linestyle="", marker=".", color="g")
    pointsEstiHead,  = ax.plot([], [], linestyle="", marker=".", color="g")
    ax.legend()
    circle = plt.Circle((0, 0), 0.2, color='black', fill=False)
    ax.add_patch(circle)
    time_text = ax.text(0.01, 0.97, '', transform=ax.transAxes)
    time_text.set_text('')
    ani = animation.FuncAnimation(fig, animate, frames=None, interval=10, blit=True)
    # ani.save('particle_box.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
    plt.show()   
else:
    # relative states in robot0's body-frame
    xEsti = relativeState[:,0,:]
    # ground-ruth relative states
    xTrueRL = calcRelaState(xTrue, numRob)
    # data: x, xGT, y, yGT, yaw, yawGT
    dataForPlot = np.array([xEsti[0,1], xTrueRL[0,1], xEsti[1,1], xTrueRL[1,1], xEsti[2,1], xTrueRL[2,1]])
    step = 0
    while simTime >= dt*step:
        print(relativeState[:, 0, 1])
        step += 1
        u = nmpc(relativeState[:, 0, 1], step)
        # u = calcInput_FlyRandom(step)
        xTrue, zNois, uNois = update(xTrue, u)
        if step % ekfStride == 0:
            relativeState = EKF(uNois, zNois, relativeState, ekfStride)
        xEsti = relativeState[:,0,:]
        xTrueRL = calcRelaState(xTrue, numRob)
        dataForPlot = np.vstack([dataForPlot, np.array([xEsti[0,1], xTrueRL[0,1], xEsti[1,1],
            xTrueRL[1,1], xEsti[2,1], xTrueRL[2,1]])])
    dataForPlotArray = dataForPlot.T
    # np.savetxt("data/rand00.txt", dataForPlot, fmt="%s")
    timePlot = np.arange(0, len(dataForPlotArray[0]))/100
    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
    plt.margins(x=0)
    ax1.plot(timePlot, dataForPlotArray[0,:])
    ax1.plot(timePlot, dataForPlotArray[1,:])
    ax1.set_ylabel(r"$x_{ij}$ (m)", fontsize=12)
    ax1.grid(True)
    ax1.margins(x=0)
    ax2.plot(timePlot, dataForPlotArray[2,:])
    ax2.plot(timePlot, dataForPlotArray[3,:])
    ax2.set_ylabel(r"$y_{ij}$ (m)", fontsize=12)
    ax2.grid(True)
    ax2.margins(x=0)
    ax3.plot(timePlot, dataForPlotArray[4,:], label='Relative EKF')
    ax3.plot(timePlot, dataForPlotArray[5,:], label='Ground-truth')
    ax3.set_ylabel(r"$\mathrm{\psi_{ij}}$ (rad)", fontsize=12)
    ax3.set_xlabel("Time (s)", fontsize=12)
    ax3.grid(True)
    ax3.margins(x=0)
    ax3.legend(loc='upper center', bbox_to_anchor=(0.7, 0.7), shadow=True, ncol=1, fontsize=12)
    # Fine-tune figure; make subplots close to each other and hide x ticks for all but bottom plot.
    f.subplots_adjust(hspace=0)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    plt.show()

    # Plot of the optimal control inputs
    # timePlot = np.arange(0, len(u_history[0]))/100
    # f, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
    # plt.margins(x=0)
    # ax1.plot(timePlot, u_history[0])
    # ax1.set_ylabel(r"$v_{i}^x$", fontsize=12)
    # ax1.grid(True)
    # ax1.margins(x=0)
    # ax2.plot(timePlot, u_history[1])
    # ax2.set_ylabel(r"$v_{i}^y$", fontsize=12)
    # ax2.grid(True)
    # ax2.margins(x=0)
    # ax3.plot(timePlot, u_history[2])
    # ax3.set_ylabel(r"$v_{j}^x$", fontsize=12)
    # ax3.grid(True)
    # ax3.margins(x=0)
    # ax4.plot(timePlot, u_history[3])
    # ax4.set_ylabel(r"$v_{j}^y$", fontsize=12)
    # ax4.grid(True)
    # ax4.margins(x=0)
    # ax4.set_xlabel("Time (s)", fontsize=12)
    # # Fine-tune figure; make subplots close to each other and hide x ticks for all but bottom plot.
    # f.subplots_adjust(hspace=0)
    # plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    # plt.show()