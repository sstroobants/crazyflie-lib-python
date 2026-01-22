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

from cf_utils import *

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E712')

# battery variables
batt_level = 0
batt_state = 0

# time variables
t_start = 0


class ConnectionLostError(Exception):
    pass

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf)

def set_drag_params(cf, dx, dy, dz, r_dx, r_dy, r_dz):
    cf.param.set_value('kalman.dragBx', dx)
    time.sleep(0.05)
    cf.param.set_value('kalman.dragBy', dy)
    time.sleep(0.05)
    cf.param.set_value('kalman.dragBz', dz)
    time.sleep(0.05)
    cf.param.set_value('kalman.drag_rx', r_dx)
    time.sleep(0.05)
    cf.param.set_value('kalman.drag_ry', r_dy)
    time.sleep(0.05)
    cf.param.set_value('kalman.drag_rz', r_dz)
    time.sleep(0.05)
    

def run_sequence(cf):
    global batt_level, batt_state, t_start

    # Starting position
    x = 0
    y = 0

    vx = 0.5
    vy = -0.5
    z = 0.8
    yaw = 0

    commander = cf.high_level_commander
    t_start = time.time()
    cf.platform.send_arming_request(True)

    start_onboard_logging(cf)
    time.sleep(1.0)
    # commander.takeoff(z, 1.0)
    # time.sleep(4.0)

    for i in range(40):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)

    for i in range(40):
        cf.commander.send_hover_setpoint(vx, 0, 0, z)
        time.sleep(0.1)

    for i in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)

    for i in range(40):
        cf.commander.send_hover_setpoint(0, vy, 0, z)
        time.sleep(0.1)

    for i in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)

    for i in range(40):
        cf.commander.send_hover_setpoint(-vx, 0, 0, z)
        time.sleep(0.1)

    for i in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)
    
    for i in range(40):
        cf.commander.send_hover_setpoint(0, -vy, 0, z)
        time.sleep(0.1)
    
    for i in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)

    cf.commander.send_notify_setpoint_stop(remain_valid_milliseconds=0)
    # time.sleep(0.05)

    print("Landing")

    # commander.land(0.0, 2.0)
    # time.sleep(2.2)

    # commander.go_to(x, y, 0.10, yaw, 1.5)
    # time.sleep(1.8)
    commander.land(0.03, 2.0)
    time.sleep(2.0)
    cf.platform.send_arming_request(False)
    stop_onboard_logging(cf)
    commander.stop()

def reconnect_and_land():
    start_time = time.time()
    duration = 10
    while time.time() - start_time < duration:
        print("Try to reconnect")
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print("Recovered connection and stopping propellors")
                cf = scf.cf
                cf.high_level_commander.stop()
        except Exception as e:
            print("Connection failed: ", e)
            time.sleep(1)

def connection_failed_link_error(link_uri, msg):
    print(f"Connection to {link_uri} failed: {msg}")
    reconnect_and_land()



def log_batt_callback(timestamp, data, logconf):
    global batt_level, batt_state, t_start
    print(f"[{time.time() - t_start:.2f}s] Batt. level: {data['pm.vbat']:0.2f}V, " + \
        #   f"target x: {data['posCtl.targetX']:0.2f}, " + \
        #   f"target y: {data['posCtl.targetY']:0.2f}, " + \
        # f"front range: {data['range.front']:0.2f}, " + \
        # f"oa.dirAxis: {data['oa.dirAxis']:d}, " + \
        # f"oa.dirSign: {data['oa.dirSign']:d}, " + \
        # f"oa.mode: {data['oa.mode']:d}, " + \
        #   f"target: {data['posCtl.targetZ']:0.2f}, " + \
          f"stateEstimate x: {data['stateEstimate.x']:0.2f}, " + \
          f"stateEstimate y: {data['stateEstimate.y']:0.2f}, " + \
          f"stateEstimate: {data['stateEstimate.z']:0.2f}")
    batt_level = data["pm.vbat"]
    # batt_state = data["pm.state"]

def add_logconfig(cf):
    log_config = LogConfig(name='Battery', period_in_ms=500)
    log_config.add_variable('pm.vbat', 'float')
    # log_config.add_variable('posCtl.targetX', 'float')
    # log_config.add_variable('posCtl.targetY', 'float')
    # log_config.add_variable('range.front', 'float')
    # log_config.add_variable('oa.dirAxis', 'uint8_t')
    # log_config.add_variable('oa.dirSign', 'int8_t')
    # log_config.add_variable('oa.mode', 'uint8_t')

    # log_config.add_variable('posCtl.targetY', 'float')
    # log_config.add_variable('posCtl.targetZ', 'float')
    # log_config.add_variable('locSrv.x', 'float')
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
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

    # print("Initializing MocapWrapper")
    # # Connect to the mocap system
    # mocap_wrapper = MocapWrapper(rigid_body_name)

    print("Connect to the Crazyflie")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.connection_lost.add_callback(connection_failed_link_error)
        cf.console.receivedChar.add_callback(console_incoming)
        
        log_config = add_logconfig(cf)

        # Set up a callback to handle data from the mocap system
        # mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

        # adjust_orientation_sensitivity(cf)
        # print("Activating the kalman estimator")
        # activate_kalman_estimator(cf)
        # reset_estimator(cf)

        set_drag_params(cf, 4.2, 1.8, 0.3, 0.0, 0.0, 0.06)

        reset_estimator(cf)

        run_sequence(cf)
        time.sleep(1.0)
        stop_logconfig(log_config)
