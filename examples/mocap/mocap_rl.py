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
Example of how to connect to a motion capture system and relay optitrack position
data to a Crazyflie, using the motioncapture library. The motioncapture library supports
all major mocap systems and provides a generalized API regardless of system type.

Set the uri to the radio settings of the Crazyflie and modify the
mocap setting matching your system.
"""
import time
from threading import Thread, Event
from tkinter import Tk, Button, Label
import tkinter as tk

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from cf_utils import console_incoming

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E700')

# The host name or ip address of the mocap system
host_name = '192.168.209.81'

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'optitrack'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'Flapper'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3


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

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        cf.extpos.send_extpose(z, x, y, quat.z, quat.x, quat.y, quat.w)
    else:
        cf.extpos.send_extpos(z, x, y)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def log_state_estimate(timestamp, data, logconf):
    """Callback to print state estimate data."""
    print(f"State Estimate - x: {data['stateEstimate.x']:0.3f}, y: {data['stateEstimate.y']:0.3f}, z: {data['stateEstimate.z']:0.3f}")


def add_logging(cf):
    """Set up logging for state estimate."""
    log_config = LogConfig(name='State Estimate', period_in_ms=500)
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    log_config.data_received_cb.add_callback(log_state_estimate)
    cf.log.add_config(log_config)
    log_config.start()
    return log_config


def stop_logging(log_config):
    """Stop logging."""
    log_config.stop()
    log_config.data_received_cb.remove_callback(log_state_estimate)


def create_gui(cf, stop_event):
    """Create a GUI with buttons to control RL state."""
    root = Tk()
    root.title("RL State Control")
    root.geometry("400x550")    
    
    state_info = {0: "Landing/Landed", 1: "Hover", 2: "RL"}
    current_state = tk.StringVar(value="0")
    
    label = Label(root, text="RL State Control", font=("Arial", 14, "bold"))
    label.pack(pady=10)
    
    state_label = Label(root, text="Current State: Landing/Landed", font=("Arial", 12))
    state_label.pack(pady=5)
    
    def set_state(state):
        try:
            cf.param.set_value('rlapp.targetState', str(state))
            
            # Enable logging when hovering, disable when landing
            if state == 1:  # Hover
                cf.param.set_value('usd.logging', 1)
                print("USD logging enabled")
            elif state == 0:  # Landing/Landed
                cf.param.set_value('usd.logging', 0)
                print("USD logging disabled")
            
            current_state.set(str(state))
            state_label.config(text=f"Current State: {state_info[state]}")
            print(f"RL State changed to {state} ({state_info[state]})")
        except Exception as e:
            print(f"Error setting parameter: {e}")
    
    button_frame = tk.Frame(root)
    button_frame.pack(pady=20)
    
    Button(button_frame, text="0 - Landing/Landed", command=lambda: set_state(0), 
            width=20, height=2, bg="#ff6b6b", font=("Arial", 12)).pack(pady=5)
    Button(button_frame, text="1 - Hover", command=lambda: set_state(1), 
            width=20, height=2, bg="#4dabf7", font=("Arial", 12)).pack(pady=5)
    Button(button_frame, text="2 - RL", command=lambda: set_state(2), 
            width=20, height=2, bg="#51cf66", font=("Arial", 12)).pack(pady=5)

    # --- X / Y target controls ---
    target_x = [0.0]
    target_y = [0.0]

    xy_frame = tk.LabelFrame(root, text="Target Position", font=("Arial", 12, "bold"), padx=10, pady=10)
    xy_frame.pack(pady=10, padx=20, fill="x")

    x_label = Label(xy_frame, text="X: 0.00 m", font=("Arial", 12))
    x_label.grid(row=0, column=1, padx=5)

    y_label = Label(xy_frame, text="Y: 0.00 m", font=("Arial", 12))
    y_label.grid(row=1, column=1, padx=5, pady=5)

    def update_target_x(delta):
        target_x[0] += delta
        x_label.config(text=f"X: {target_x[0]:.2f} m")
        try:
            cf.param.set_value('rlapp.targetX', str(target_x[0]))
            print(f"rlapp.targetX set to {target_x[0]:.2f}")
        except Exception as e:
            print(f"Error setting rlapp.targetX: {e}")

    def update_target_y(delta):
        target_y[0] += delta
        y_label.config(text=f"Y: {target_y[0]:.2f} m")
        try:
            cf.param.set_value('rlapp.targetY', str(target_y[0]))
            print(f"rlapp.targetY set to {target_y[0]:.2f}")
        except Exception as e:
            print(f"Error setting rlapp.targetY: {e}")

    Button(xy_frame, text="X -0.5", command=lambda: update_target_x(-0.5),
           width=8, height=1, bg="#ffa94d", font=("Arial", 11)).grid(row=0, column=0, padx=5)
    Button(xy_frame, text="X +0.5", command=lambda: update_target_x(+0.5),
           width=8, height=1, bg="#ffa94d", font=("Arial", 11)).grid(row=0, column=2, padx=5)

    Button(xy_frame, text="Y -0.5", command=lambda: update_target_y(-0.5),
           width=8, height=1, bg="#cc5de8", font=("Arial", 11)).grid(row=1, column=0, padx=5)
    Button(xy_frame, text="Y +0.5", command=lambda: update_target_y(+0.5),
           width=8, height=1, bg="#cc5de8", font=("Arial", 11)).grid(row=1, column=2, padx=5)

    def on_closing():
        stop_event.set()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    while not stop_event.is_set():
        try:
            root.update()
            time.sleep(0.01)
        except:
            break


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    print("Initializing MocapWrapper")
    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)

    print("Connect to the Crazyflie")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Set up a callback to handle data from the mocap system
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

        cf.console.receivedChar.add_callback(console_incoming)
        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)
        reset_estimator(cf)

        # Set up logging
        log_config = add_logging(cf)

        # Start GUI thread
        stop_event = Event()
        gui_thread = Thread(target=create_gui, args=(cf, stop_event), daemon=True)
        gui_thread.start()

        print('Relaying optitrack data. Close the GUI or press Ctrl+C to stop.')
        try:
            # Keep the connection alive and relay mocap data
            while not stop_event.is_set():
                time.sleep(0.01)
        except KeyboardInterrupt:
            print('Stopping mocap relay')
        finally:
            stop_event.set()
            gui_thread.join(timeout=2)
            stop_logging(log_config)

    mocap_wrapper.close()
