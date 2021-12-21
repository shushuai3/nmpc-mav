'''
Python code to run NMPC on laptop to control two Crazyflies
Logging and command funtions are from basicLog.py
'''

import logging
import time
from threading import Timer

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

import ast
import sys
from NatNetClient import NatNetClient

uri0 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E5')
uri1 = uri_helper.uri_from_env(default='radio://1/80/2M/E7E7E7E7E6')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import numpy as np
relaState = np.array([0, 0, 0])
GT_xyz0 = np.array([0, 0, 0])
GT_xyz1 = np.array([0, 0, 0])
GT_q0 = np.array([0, 0, 0, 0])
GT_q1 = np.array([0, 0, 0, 0])

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('relative_pos.rlX1', 'float')
        self._lg_stab.add_variable('relative_pos.rlY1', 'float')
        self._lg_stab.add_variable('relative_pos.rlYaw1', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            if int(link_uri[-1])==5:
                self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        t = Timer(500, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()
        # print(data['relative_pos.rlX1'])
        relaState[0] = data['relative_pos.rlX1']
        relaState[1] = data['relative_pos.rlY1']
        relaState[2] = data['relative_pos.rlYaw1']
        pass

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

# Acados and optimal problem initialization
from multiRobotModel import multiRobotModel
from multiRobotOpti import MultiRobotOptimizer
model, constraint = multiRobotModel()
tp, N = 1.0, 30
solver = MultiRobotOptimizer(model, constraint, tp, N)
for i in range(N):
    # solver.set(i, 'yref', np.array([0]))
    solver.set(i, 'yref', np.array([0, 1, 1]))
u_history = [[], [], [], []]
t_history = []

def nmpc(xCurrent):
    # given the initial states (clip is important for the solver convergence)
    # if step > 1500:
    #     for i in range(N):
    #         solver.set(i, 'yref', np.array([0, 3*np.cos(step/100.0), 3*np.sin(step/100.0)]))
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
    u = np.zeros((3, 2))
    u[0:2,0] = uOpti[0:2]
    u[0:2,1] = uOpti[2:4]
    return u

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
	pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global GT_xyz0, GT_xyz1, GT_q0, GT_q1
    if id==1:
        GT_xyz0 = position
        GT_q0 = rotation
    if id==2:
        GT_xyz1 = position
        GT_q1 = rotation

if __name__ == '__main__':

    # streamingClient = NatNetClient() # Create a new NatNet client
    # streamingClient.newFrameListener = receiveNewFrame
    # streamingClient.rigidBodyListener = receiveRigidBodyFrame
    # streamingClient.run() # Run perpetually on a separate thread.

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    le0 = LoggingExample(uri0)
    le1 = LoggingExample(uri1)

    # file = open('./dat00.csv', 'w')
    # file.write('time, relaX, relaY, relaYaw, OTrelaX, OTrelaX, x0, y0, x1, y1, q00, q01, q02, q03, q10, q11, q12, q13\n')

    time0 = round(time.time()*1000)%1000000 # ms
    # while 1:
    #     t_ms = round(time.time()*1000)%1000000 - time0
    #     file.write('{}, {}, {}, {}, {}, {}, {}, {}, {}\n'.format(t_ms, relaState[0],
    #         relaState[1], relaState[2], GT_xyz1[2]-GT_xyz0[2], GT_xyz1[0]-GT_xyz0[0], GT_xyz0[2], GT_xyz0[0], GT_xyz1[2], GT_xyz1[0],
            # GT_q0[0], GT_q0[1], GT_q0[2], GT_q0[3], GT_q1[0], GT_q1[1], GT_q1[2], GT_q1[3]))

    for y in range(10):
        le0._cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
        le1._cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
        time.sleep(0.1)

    for _ in range(20):
        le0._cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
        le1._cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
        time.sleep(0.1)

    while le0.is_connected and le1.is_connected:
        try:
            # time.sleep(0.01)
            u = nmpc(relaState)
            le0._cf.commander.send_hover_setpoint(u[0, 0], u[1, 0], 0, 0.4)
            le1._cf.commander.send_hover_setpoint(u[0, 1], u[1, 1], 0, 0.4)
            # t_ms = round(time.time()*1000)%1000000 - time0
            # file.write('{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n'.format(t_ms, relaState[0],
            #     relaState[1], relaState[2], GT_xyz1[2]-GT_xyz0[2], GT_xyz1[0]-GT_xyz0[0], GT_xyz0[2], GT_xyz0[0], GT_xyz1[2], GT_xyz1[0],
            #     GT_q0[0], GT_q0[1], GT_q0[2], GT_q0[3], GT_q1[0], GT_q1[1], GT_q1[2], GT_q1[3]))

        except KeyboardInterrupt:
            # file.close()
            print("stop")
            ## landing procedure
            # for i in range(20):
            #     le[i]._cf.commander.send_hover_setpoint(0, 0, 0, 0.6-i*0.025)
            #     time.sleep(0.1)
            raise
