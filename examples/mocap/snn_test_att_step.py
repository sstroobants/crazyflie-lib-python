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

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from cf_utils import *
from mocap_wrapper import MocapWrapper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E0')

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'cf'

# Using SNN control or not
snn_control = True

# battery variables
batt_level = 0
batt_state = 0

# time variables
t_start = 0

# Log config
logconfig = [{"name": "pm.vbat", "type": "float"}, 
              {"name": "pm.state", "type": "int8_t"}, 
              {"name": "posCtl.targetX", "type": "float"}, 
              {"name": "locSrv.x", "type": "float"}, 
              {"name": "ctrltarget.x", "type": "float"}]

def run_sequence(cf):
    global batt_level, batt_state, t_start

    # Starting position
    x = 0
    y = -2
    z = 1
    yaw = 0

    commander = cf.high_level_commander
    deactivate_snn_controller(cf)
    t_start = time.time()
    commander.takeoff(z, 2.0)
    time.sleep(2.5)
    commander.go_to(x, y, z, yaw, 1)
    time.sleep(2.0)
    if snn_control:
        print("Activating SNN controller")
        set_snn_I_gain(cf, 0.3)
        activate_snn_controller(cf)
    time.sleep(2.0)
    start_onboard_logging(cf)
    print("Setpoint change")
    for i in range(20):
        cf.commander.send_zdistance_setpoint(0, 0, 0, z)
        time.sleep(0.1)
    for i in range(15):
        cf.commander.send_zdistance_setpoint(-10, 0, 0, z)
        time.sleep(0.1)
    for i in range(15):
        cf.commander.send_zdistance_setpoint(10, 0, 0, z)
        time.sleep(0.1)
    for i in range(30):
        cf.commander.send_zdistance_setpoint(0, 0, 0, z)
        time.sleep(0.1)
    cf.commander.send_notify_setpoint_stop(remain_valid_milliseconds=0)
    time.sleep(0.05)
    stop_onboard_logging(cf)
    print("Back")
    commander.go_to(x, y, z, yaw, 2)
    time.sleep(2.0)
    deactivate_snn_controller(cf)
    time.sleep(2.0)
    commander.land(0.0, 2.0)
    time.sleep(4)
    commander.stop()

def connection_failed_link_error(link_uri, msg):
    print(f"Connection to {link_uri} failed: {msg}, trying to reconnect and land")
    reconnect_and_land(link_uri)

def log_callback(timestamp, data, logconf):
    global batt_level, batt_state, t_start
    print(f"[{time.time() - t_start:.2f}s] Batt. level: {data['pm.vbat']:0.2f}V, " + \
          f"state: {data['pm.state']}, " + \
          f"target: {data['posCtl.targetX']:0.3f}, " + \
          f"locSrv: {data['locSrv.x']:0.3f}, " + \
          f"ctrltarget: {data['ctrltarget.x']:0.3f}")
    batt_level = data["pm.vbat"]
    batt_state = data["pm.state"]

def stop_logconfig(logconfig):
    logconfig.stop()

if __name__ == '__main__':
    print("initializing drivers")
    cflib.crtp.init_drivers()

    print("Initializing MocapWrapper")
    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)

    print("Connect to the Crazyflie")
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            cf.connection_lost.add_callback(connection_failed_link_error)
            
            log_config = add_logconfig(cf, logconfig, log_callback, freq=2000)

            # Set up a callback to handle data from the mocap system
            mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

            print("Activating the kalman estimator")
            activate_kalman_estimator(cf)
            reset_estimator(cf)
            run_sequence(cf)
            time.sleep(1.0)
            stop_logconfig(log_config)
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            cf.high_level_commander.stop()

    mocap_wrapper.close()
