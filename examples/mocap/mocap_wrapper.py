from threading import Thread

import motioncapture

# The host name or ip address of the mocap system
host_name = '192.168.209.81'

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'optitrack'


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

                        # print(f"Position: ({-pos[1]}, {pos[0]}, {pos[2]})")     
                        # rotation = {"w": obj.rotation.w, "x": -obj.rotation.y, "y": obj.rotation.x, "z": obj.rotation.z}
                        # rotation = [obj.rotation.w, obj.rotation.y, -obj.rotation.x, obj.rotation.z]
                        # 0 = y, 1 = -x, 2 = z
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])
    
