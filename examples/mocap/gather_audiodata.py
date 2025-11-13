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

from cf_utils import *
import matplotlib
# matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

send_anchors = False

# URI to the Crazyflie to connect to
uri_00 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E700')
uri_01 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')
uri_02 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
uri_03 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

forbidden_area = [
    [0.0, 0.0],
    [4.5, 0.0],
    [4.5, -5.3],
    [0.0, -5.3]
]

location_marker = None

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
visited_locations = []

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

def point_in_rect(point, rect):
    x, y = point
    lleft, uleft, uright, lright = rect
    return (lleft[0] <= x <= uright[0]) and (lleft[1] >= y >= uright[1])

def rect_max(rect):
    lleft, uleft, uright, lright = rect
    return lleft[0], lright[1]

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
        

def gather_audio_data(cf, x, y, z, yaw, do_yaw=True):
    global x_est, y_est, audio_db, audio_timestamp, last_audio_timestamp, visited_locations
    commander = cf.high_level_commander

    # Calculate reasonable time to reach target
    airspeed = 0.5
    distance =  ((x - x_est)**2 + (y - y_est)**2)**0.5
    time_to_target = max(1.5, distance / airspeed) # at least 
    print(f"Distance to target: {distance:0.2f} m, time to target: {time_to_target:0.2f} s")

    # Calculate yaw to fly towards target
    if do_yaw:
        yaw = np.arctan2(y - y_est, x - x_est)
    else:
        yaw = 0.0
    print(f"Calculated yaw: {yaw:0.2f} rad")

    arm_fly_land(cf, x, y, z, yaw)

    while True:
        # Only accept new measurement if audio level is below threshold and at least 9 seconds have passed since last measurement
        # This is to avoid the measurements taken during flight (high prop noise). This is probably a tuning parameter
        if ((audio_db < 85) and (audio_timestamp - last_audio_timestamp > 9000)):
            last_audio_timestamp = audio_timestamp
            break
        time.sleep(0.1)
    visited_locations.append((x_est, y_est))
    print(f"Measurement at position x: {x_est:0.2f}, y: {y_est:0.2f} done.")
    print(f"Audio data: {audio_db:0.2f} dB at timestamp {audio_timestamp:d}")
    print(f"Battery level: {batt_level:0.2f}V")

    with open(audio_file, 'a', newline='') as f:
        writer = csv.writer(f)
        # Write header if file is empty
        if f.tell() == 0:
            writer.writerow(['x', 'y', 'db'])
        writer.writerow([x_est, y_est, audio_db])
    # Update realtime plot with new measurement
    try:
        ax = plt.gca()
        # scatter current measurement (x_est, y_est) with color by audio_db
        sc = ax.scatter([y_est], [x_est], c=[audio_db], vmin=50, vmax=80, cmap='viridis', s=80, edgecolors='k')
        # update colorbar to reflect vmin/vmax if needed
        # keep a reference to last scatter in the axes for removal if desired
        plt.draw()
        plt.pause(0.001)
    except Exception:
        pass

    
def update_location_marker(offset=0.0):
    global x_est, y_est, location_marker
    # Create/update a single black cross for the estimated source
    try:
        if location_marker is None:
            (location_marker,) = ax.plot(
                [], [], marker='x', color='black', markersize=12,
                mew=3, linestyle='None', zorder=5
            )
        location_marker.set_data([y_est + offset], [x_est])
        plt.draw()
        plt.pause(0.01)
    except Exception:
        pass

def update_estimation_marker(xs, ys):
    global estimation_marker
    # Create/update a single black cross for the estimated source
    try:
        if estimation_marker is None:
            (estimation_marker,) = ax.plot(
                [], [], marker='x', color='black', markersize=12,
                mew=3, linestyle='None', zorder=5
            )
        estimation_marker.set_data([ys], [xs])
        plt.draw()
        plt.pause(0.01)
    except Exception:
        pass

def arm_fly_land(cf, x, y, z, yaw):
    global x_est, y_est, audio_db, audio_timestamp, last_audio_timestamp, visited_locations
    commander = cf.high_level_commander

    # Calculate reasonable time to reach target
    airspeed = 0.5
    distance =  ((x - x_est)**2 + (y - y_est)**2)**0.5
    time_to_target = max(1.5, distance / airspeed) # at least 1.5 seconds
    print(f"Target: {x}, {y}")
    print(f"Distance to target: {distance:0.2f} m, time to target: {time_to_target:0.2f} s")

    print("Arming")
    cf.platform.send_arming_request(True)
    time.sleep(1.0)
    commander.takeoff(z, 1.0)
    time.sleep(2.0)
    commander.go_to(x_est, y_est, z, yaw, 1.5) # go to current estimated position, with target yaw first IS THIS NECESSARY?
    time.sleep(1.5)
    update_location_marker()
    commander.go_to(x, y, z, yaw, time_to_target)
    for _ in range(int(time_to_target)):
        time.sleep(1)
        update_location_marker()
    time.sleep(time_to_target % 1)
    update_location_marker()

    wait_counter = 0
    while abs(x_est - x) > 0.1 or abs(y_est - y) > 0.1: # if we are further than 10 cm from target, wait
        print(f"Waiting to reach target position x: {x}, y: {y}. Current estimated position x: {x_est:0.2f}, y: {y_est:0.2f}")
        time.sleep(0.5)
        update_location_marker()
        wait_counter += 1
        if wait_counter > 6: # after 3 seconds, give up
            print("Taking too long to reach target, proceeding to landing.")
            break

    commander.go_to(x, y, z, 0.0, 1.5) # rotate back to zero yaw
    time.sleep(1.2)
    update_location_marker()
    commander.go_to(x, y, 0.07, 0.0, 1.2)
    time.sleep(1.8)
    update_location_marker()
    commander.land(0.0, 0.5)
    time.sleep(0.9)
    update_location_marker()
    cf.platform.send_arming_request(False)
    print("Disarmed")

def run_sequence(cf):
    global batt_level, batt_state, t_start, audio_db, audio_timestamp, last_audio_timestamp, x_est, y_est

    initial_measurement_positions = [
        (2, -1),
        (5.4, -1.2),
        (6.8, -3.8),
        (8, -6),
        (7, -8),
        (5.2, -7.5),
        (4.0, -6.0),
        (2.5, -8.0),
        (2.0, -7.0)
    ]

    # Starting position
    x = 0
    y = -1
    z = 0.6
    yaw = 0

    commander = cf.high_level_commander
    t_start = time.time()

    last_audio_timestamp = audio_timestamp

    n = 0
    for pos in initial_measurement_positions:
        x = pos[0]
        y = pos[1]
        print(f"Flying to position x: {x}, y: {y}, z: {z}")
        if n == 0:
            gather_audio_data(cf, x, y, z, yaw, do_yaw=False)
        else:
            gather_audio_data(cf, x, y, z, yaw, do_yaw=True)
        n += 1

        if n > 2:
            print(f"\nFinished {n} measurements, fitting and proceeding")
            time.sleep(0.1)
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

            # initial guess (center of room with reasonable L0)
            x0 = 5
            y0 = -5
            L0_guess = 78
            res = least_squares(residuals, [x0, y0, L0_guess])

            xs, ys, L0_fit = res.x

            xs = np.clip(xs, 1, 9)
            ys = np.clip(ys, -9, -1)

            print(f"Estimated source: x={xs:.2f}, y={ys:.2f}, L0≈{L0_fit:.1f} dB at 1 m")
            print(f"RMS residual: {np.sqrt(np.mean(res.fun**2)):.1f} dB")

            # Create/update a single red cross for the estimated source
            try:
                if estimated_marker is None:
                    (estimated_marker,) = ax.plot(
                        [], [], marker='x', color='red', markersize=12,
                        mew=3, linestyle='None', zorder=5
                    )
                estimated_marker.set_data([ys], [xs])
                plt.draw()
                plt.pause(0.01)
            except Exception:
                pass

    # Land close to whiteboard to work as beacon
    # arm_fly_land(cf, 7, -7, 0.6, yaw=0)
    # Land at the landing platform
    arm_fly_land(cf, 0.5, -7.4, 0.6, yaw=0)
    cf.param.set_value('loco.isAnchor', '1')
    
    



def log_batt_callback(timestamp, data, logconf):
    global batt_level, batt_state, t_start, audio_db, audio_timestamp, x_est, y_est, location_marker
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

    update_location_marker(offset=np.random.uniform(-1.0, 1.0))

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

def send_anchor(uri, x, y, z):
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.connection_lost.add_callback(connection_failed_link_error)
        # cf.console.receivedChar.add_callback(console_incoming)


        log_config = add_logconfig(cf)

        # cf.param.set_value('loco.isAnchor', '1')
        arm_fly_land(cf, x, y, z, 0)

        stop_logconfig(log_config)
        print("Connect to the Crazyflie")

        
def send_package(uri, x, y, z):
    global x_est, y_est, audio_db, audio_timestamp, last_audio_timestamp, visited_locations
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.connection_lost.add_callback(connection_failed_link_error)
        # cf.console.receivedChar.add_callback(console_incoming)


        log_config = add_logconfig(cf)

        commander = cf.high_level_commander

        cf.param.set_value('loco.isAnchor', '0')
        time.sleep(0.2)

        # Calculate reasonable time to reach target
        airspeed = 0.5
        distance =  ((x - x_est)**2 + (y - y_est)**2)**0.5
        time_to_target = max(1.5, distance / airspeed) # at least 1.5 seconds
        print(f"Distance to target: {distance:0.2f} m, time to target: {time_to_target:0.2f} s")

        # print("Arming")
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
        commander.takeoff(z, 1.0)
        time.sleep(2.0)
        commander.go_to(x_est, y_est, z, 0.0, 1.5) # go to current estimated position, with target yaw first IS THIS NECESSARY?
        time.sleep(1.5)
        update_location_marker()

        # Fly through free passage
        commander.go_to(5.5, -1.0, z, 0.0, time_to_target)
        for _ in range(int(time_to_target)):
            time.sleep(1)
            update_location_marker()
        time.sleep(time_to_target % 1)
        update_location_marker()

        wait_counter = 0
        while abs(x_est - 5.5) > 0.1 or abs(y_est - -1.0) > 0.1: # if we are further than 10 cm from target, wait
            print(f"Waiting to reach target position x: {5.5}, y: {-1.0}. Current estimated position x: {x_est:0.2f}, y: {y_est:0.2f}")
            time.sleep(0.5)
            update_location_marker()
            wait_counter += 1
            if wait_counter > 6: # after 3 seconds, give up
                print("Taking too long to reach target, proceeding to landing.")
                break

        distance =  ((x - x_est)**2 + (y - y_est)**2)**0.5
        time_to_target = max(1.5, distance / airspeed) # at least 1.5 seconds
        print(f"Distance to target: {distance:0.2f} m, time to target: {time_to_target:0.2f} s")
        yaw = np.arctan2(y - y_est, x - x_est)
        print(f"Calculated yaw: {yaw:0.2f} rad")

        commander.go_to(x_est, y_est, z, yaw, 1.0)
        time.sleep(1.0)
        commander.go_to(x, y, z, yaw, time_to_target)
        for _ in range(int(time_to_target)):
            time.sleep(1)
            update_location_marker()
        time.sleep(time_to_target % 1)
        update_location_marker()

        
        commander.go_to(x, y, z, yaw, 1.5) # rotate back to zero yaw
        time.sleep(1.2)
        update_location_marker()
        commander.go_to(x, y, 0.07, 0.0, 1.2)
        time.sleep(1.8)
        update_location_marker()
        commander.land(0.0, 0.5)
        time.sleep(0.9)
        update_location_marker()
        cf.platform.send_arming_request(False)
        print("Disarmed")

        stop_logconfig(log_config)


if __name__ == '__main__':
    # Initialize realtime plotting using the arena image as background
    img = mpimg.imread(os.path.join(os.path.dirname(__file__), '..', '..', 'images', 'arena.png'))

    estimated_marker = None

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))
    # Show arena image; extent matches x:0-10, y:-10-0 so that origin is top-left unless image oriented differently
    ax.imshow(img, extent=[0, -10, 0, 10])
    ax.set_xlim(0, -10)
    ax.set_ylim(0, 10)
    ax.set_xlabel('y (m)')
    ax.set_ylabel('x (m)')
    # create a colorbar mappable for consistent scaling
    from matplotlib.cm import ScalarMappable
    sm = ScalarMappable(cmap='viridis')
    sm.set_clim(50, 80)
    cbar = fig.colorbar(sm, ax=ax)
    cbar.set_label('Sound level (dB)')

    plt.show(block=False)
    plt.pause(0.005)

    update_location_marker()
    update_estimation_marker(5, -5)

    print("initializing drivers")
    cflib.crtp.init_drivers()

    print("Connect to the Crazyflie")

    if send_anchors:
        send_anchor(uri_01, x=1.4, y=-5, z=0.6)
        send_anchor(uri_02, x=7, y=-1, z=0.6)
        time.sleep(20)

    print("Connect to the Crazyflie")
    with SyncCrazyflie(uri_00, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.connection_lost.add_callback(connection_failed_link_error)
        # cf.console.receivedChar.add_callback(console_incoming)
        

        log_config = add_logconfig(cf)

        # adjust_orientation_sensitivity(cf)
        # print("Activating the kalman estimator")
        # activate_kalman_estimator(cf)


        run_sequence(cf)

        stop_logconfig(log_config)
    
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

    # initial guess (center of room with reasonable L0)
    x0 = 5
    y0 = -5
    L0_guess = 78
    res = least_squares(residuals, [x0, y0, L0_guess])

    xs, ys, L0_fit = res.x
    print(f"Estimated source: x={xs:.2f}, y={ys:.2f}, L0≈{L0_fit:.1f} dB at 1 m")
    print(f"RMS residual: {np.sqrt(np.mean(res.fun**2)):.1f} dB")

    # Create/update a single red cross for the estimated source
    try:
        if estimated_marker is None:
            (estimated_marker,) = ax.plot(
                [], [], marker='x', color='red', markersize=12,
                mew=3, linestyle='None', zorder=5
            )
        estimated_marker.set_data([ys], [xs])
        plt.draw()
        plt.pause(0.01)
    except Exception:
        pass
    
    send_package(uri_03, x=5.2, y=-1.0, z=0.6)


    plt.savefig("images/audio_data.png")
    plt.close()
    time.sleep(360)