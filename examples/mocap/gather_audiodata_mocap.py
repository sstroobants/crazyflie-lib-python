# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2023 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Example of how to connect to a motion capture system and feed the position to a
Crazyflie, using the motioncapture library. The motioncapture library supports all major mocap systems and provides
a generalized API regardless of system type.
The script uses the high level commander to upload a trajectory to fly a figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
mocap setting matching your system.
"""
import time
from threading import Thread
import random

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import csv
import os
import pandas as pd
import numpy as np
from scipy.optimize import least_squares

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E700')

# The host name or ip address of the mocap system
host_name = '192.168.209.81'


# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'optitrack'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'cf'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 4.5e-3


# battery variables
batt_level = 0
batt_state = 0

# audio variables
audio_db = 0
audio_timestamp = 0
last_audio_timestamp = 0

# estimated position variables
x_est = 2
y_est = -2

# time variables
t_start = 0

# Find the next available file index
i = 0
while os.path.exists(f'data/audio_data_{i}.csv'):
    i += 1
audio_file = f'data/audio_data_{i}.csv'

# Ensure the data directory exists
os.makedirs('data', exist_ok=True)



class ConnectionLostError(Exception):
    pass

class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        print("Connecting to mocap system")
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        print("Connecting to optitrack successful")
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    # print(self.on_pose)
                    if self.on_pose:
                        pos = obj.position

                        # print(f"Position: ({pos[0] + 5}, {-pos[2] - 5}, {pos[1]})")     
                        # rotation = {"w": obj.rotation.w, "x": -obj.rotation.y, "y": obj.rotation.x, "z": obj.rotation.z}
                        # rotation = [obj.rotation.w, obj.rotation.y, -obj.rotation.x, obj.rotation.z]
                        # 0 = y, 1 = -x, 2 = z
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])
            # print(3)
    

def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """

    # Shift the coordinates to match the cyberzoo coordinate system with 0,0 in front left
    x = x + 5.0
    z = z + 5.0

    if send_full_pose:
        cf.extpos.send_extpose(x, -z, y, quat.x, -quat.z, quat.y, quat.w)
        # cf.extpos.send_extpose(-y, x, z, -quat.y, quat.x, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, -z, y)
        # print(-y, x, z)
        # cf.extpos.send_extpos(-y, x, z)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def start_onboard_logging(cf):
    cf.param.set_value("usd.logging", "1")

def stop_onboard_logging(cf):
    cf.param.set_value("usd.logging", "0")

def get_battery_level(cf):
    log_config = LogConfig(name='Battery', period_in_ms=5000)
    log_config.add_variable('pm.vbat', 'float')
    with SyncLogger(cf, log_config) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            return data["pm.vbat"]
        
def get_battery_state(cf):
    log_config = LogConfig(name='State', period_in_ms=500)
    log_config.add_variable('pm.state', 'int8_t')
    with SyncLogger(cf, log_config) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            return data["pm.state"]

def arm_fly_land(cf, x, y, z, yaw):
    global x_est, y_est, audio_db, audio_timestamp, last_audio_timestamp
    commander = cf.high_level_commander

    # Calculate reasonable time to reach target
    airspeed = 0.65
    distance =  ((x - x_est)**2 + (y - y_est)**2)**0.5
    time_to_target = max(1.5, distance / airspeed) # at least 
    print(f"Distance to target: {distance:0.2f} m, time to target: {time_to_target:0.2f} s")

    print("Arming")
    cf.platform.send_arming_request(True)
    time.sleep(1.0)
    commander.takeoff(z, 1.0)
    time.sleep(2.0)
    commander.go_to(x, y, z, yaw, time_to_target)
    time.sleep(time_to_target + 0.5)

    commander.go_to(x, y, 0.05, yaw, 1.2)
    time.sleep(1.8)
    commander.land(0.0, 0.5)
    time.sleep(1.5)
    cf.platform.send_arming_request(False)
    print("Disarmed")

    while True:
        if ((audio_db < 85) and (audio_timestamp - last_audio_timestamp > 9000)):
            last_audio_timestamp = audio_timestamp
            break
        time.sleep(0.1)
    print(f"Measurement at position x: {x_est:0.2f}, y: {y_est:0.2f} done.")
    print(f"Audio data: {audio_db:0.2f} dB at timestamp {audio_timestamp:d}")
    print(f"Battery level: {batt_level:0.2f}V")

    with open(audio_file, 'a', newline='') as f:
        writer = csv.writer(f)
        # Write header if file is empty
        if f.tell() == 0:
            writer.writerow(['x', 'y', 'db'])
        writer.writerow([x_est, y_est, audio_db])

def run_sequence(cf):
    global batt_level, batt_state, t_start, audio_db, audio_timestamp, last_audio_timestamp, x_est, y_est

    initial_measurement_positions = [
        (2, -2),
        (8, -6),
        (4, -8)
    ]

    # Starting position
    x = 2
    y = -2
    z = 0.6
    yaw = 0

    commander = cf.high_level_commander
    t_start = time.time()

    last_audio_timestamp = audio_timestamp

    for pos in initial_measurement_positions:
        x = pos[0]
        y = pos[1]
        print(f"Flying to position x: {x}, y: {y}, z: {z}")
        arm_fly_land(cf, x, y, z, yaw)
        # while True:
        #     if ((audio_db < 85) and (audio_timestamp - last_audio_timestamp > 9000)):
        #         last_audio_timestamp = audio_timestamp
        #         break
        #     time.sleep(0.1)
        # print(f"Measurement at position x: {x_est:0.2f}, y: {y_est:0.2f} done.")
        # print(f"Audio data: {audio_db:0.2f} dB at timestamp {audio_timestamp:d}")
        # print(f"Battery level: {batt_level:0.2f}V")

        # with open(audio_file, 'a', newline='') as f:
        #     writer = csv.writer(f)
        #     # Write header if file is empty
        #     if f.tell() == 0:
        #         writer.writerow(['x', 'y', 'db'])
        #     writer.writerow([x_est, y_est, audio_db])

def reconnect_and_land():
    start_time = time.time()
    duration = 10
    while time.time() - start_time < duration:
        print("Try to reconnect")
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print("Recovered connection and stopping propellors")
                cf = scf.cf
                cf.high_level_commander.land(0.0, 2.0)
        except Exception as e:
            print("Connection failed: ", e)
            time.sleep(1)

def connection_failed_link_error(link_uri, msg):
    print(f"Connection to {link_uri} failed: {msg}")
    reconnect_and_land()



def log_batt_callback(timestamp, data, logconf):
    global batt_level, batt_state, t_start, audio_db, audio_timestamp, x_est, y_est
    print(f"[{time.time() - t_start:.2f}s] Batt. level: {data['pm.vbat']:0.2f}V, " + \
    #         f"db: {data['teensy.audio_db']:0.2f}, " + \
    #         f"db timestamp: {data['teensy.audio_timestamp']:d}, " + \
    #     #   f"yaw: {data['stateEstimate.yaw']:0.2f}, " + \
    #     #   f"target y: {data['posCtl.targetVY']:0.2f}, " + \
    #     #   f"target: {data['posCtl.targetZ']:0.2f}, " + \
    #     #   f"target z: {data['posCtl.targetZ']:0.2f}, " + \
          f"est. y: {data['stateEstimate.y']:0.2f}, " + \
    #     #   f"target x: {data['posCtl.targetX']:0.2f}, " + \
          f"est. x: {data['stateEstimate.x']:0.2f}") # + \
    #     #   f"stateEstimate y: {data['stateEstimate.y']:0.2f}")
    #     #   f"stateEstimate: {data['stateEstimate.z']:0.2f}")
    batt_level = data["pm.vbat"]
    # batt_state = data["pm.state"]
    audio_db = data["teensy.audio_db"]
    audio_timestamp = data["teensy.audio_timestamp"]
    x_est = data["stateEstimate.x"]
    y_est = data["stateEstimate.y"]

def add_logconfig(cf):
    log_config = LogConfig(name='Battery', period_in_ms=1000)
    log_config.add_variable('pm.vbat', 'float')
    log_config.add_variable('teensy.audio_db', 'float')
    log_config.add_variable('teensy.audio_timestamp', 'uint32_t')
    # log_config.add_variable('posCtl.targetX', 'float')
    # log_config.add_variable('posCtl.targetZ', 'float')
    # log_config.add_variable('posCtl.targetVY', 'float')
    # log_config.add_variable('stateEstimate.yaw', 'float')
    # log_config.add_variable('posCtl.targetZ', 'float')
    # log_config.add_variable('locSrv.x', 'float')
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    # log_config.add_variable('stateEstimate.vy', 'float')
    # log_config.add_variable('stateEstimate.z', 'float')
    log_config.data_received_cb.add_callback(log_batt_callback)
    cf.log.add_config(log_config)
    log_config.start()
    return log_config

def stop_logconfig(logconfig):
    logconfig.stop()
    logconfig.data_received_cb.remove_callback(log_batt_callback)

if __name__ == '__main__':
    print("initializing drivers")
    cflib.crtp.init_drivers()

    print("Initializing MocapWrapper")
    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)

    print("Connect to the Crazyflie")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.connection_lost.add_callback(connection_failed_link_error)
        

        log_config = add_logconfig(cf)

        # Set up a callback to handle data from the mocap system
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

        reset_estimator(cf)
        wait_for_position_estimator(cf)

        # adjust_orientation_sensitivity(cf)
        # print("Activating the kalman estimator")
        # activate_kalman_estimator(cf)
        run_sequence(cf)


        for i in range(8):
            print("Finished sequence, fitting and flying to estimated source location")
            time.sleep(1)
            data = pd.read_csv(audio_file, skipinitialspace=True)

            x = data["x"].values
            y = data["y"].values
            L = data["db"].values

            def residuals(params):
                xs, ys, L0 = params
                r = np.sqrt((x - xs)**2 + (y - ys)**2)
                # avoid log(0)
                r = np.clip(r, 1e-3, None)
                pred = L0 - 20*np.log10(r)
                return L - pred  # residuals in dB

            # initial guess (e.g. center of your measurement area)
            x0 = np.mean(x)
            y0 = np.mean(y)
            L0_guess = np.max(L)
            res = least_squares(residuals, [x0, y0, L0_guess])

            xs, ys, L0_fit = res.x
            print(f"Estimated source: x={xs:.2f}, y={ys:.2f}, L0≈{L0_fit:.1f} dB at 1 m")
            print("RMS residual:", np.sqrt(np.mean(res.fun**2)), "dB")

            arm_fly_land(cf, xs, ys, 0.6, 0)

        stop_logconfig(log_config)
    mocap_wrapper.close()

