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

from cf_utils import *

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# battery variables
batt_level = 0
batt_state = 0

# time variables
t_start = 0


class ConnectionLostError(Exception):
    pass



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
    log_config = LogConfig(name='Battery', period_in_ms=2000)
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

    print("Connect to the Crazyflie")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.connection_lost.add_callback(connection_failed_link_error)
        cf.console.receivedChar.add_callback(console_incoming)
        
        log_config = add_logconfig(cf)

        while True:
            time.sleep(0.1)