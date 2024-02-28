#!/usr/bin/env python
import json
import os.path
import signal
import sys
from copy import deepcopy

import numpy as np
import quaternion
from time import sleep
from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint, \
    SubDeviceRepresentation


class WallManager(AbstractVirtualCapability):

    def __init__(self, server):
        super().__init__(server)
        self.wall = []
        self.cars = []
        self.block_handler = None
        self.blocks = []

    def SetupWall(self, params: dict) -> dict:
        self.invoke_sync("InitializeSwarm", {"int": 2})
        self.block_handler = self.query_sync("BlockHandler")
        cnt = params["int"]
        for i in range(cnt - len(self.cars)):
            self.cars.append(self.query_sync("PlacerRobot", -1))
        self.wall = params["Vector3"]
        starting_point = params["ListOfPoints"]
        self.__assign_placer_to_wall()
        return {"DeviceList": self.cars}

    def WallTick(self, params: dict):
        copter = SubDeviceRepresentation(self.invoke_sync("GetAvaiableCopter", params)["Device"], self, None)
        stone = self.blocks.pop()
        formatPrint(self, stone)
        new_block = self.block_handler.invoke_sync("SpawnBlock", {"Vector3":stone["Vector3"]})

        copter.invoke_sync("SetPosition", {"Position3D":new_block["Position3D"]})
        copter.invoke_sync("TransferBlock", {"SimpleIntegerParameter": new_block["SimpleIntegerParameter"]})
        copter.invoke_sync("SetRotation", {"Quaternion": stone["Quaternion"]})
        copter.invoke_sync("SetPosition", {"Position3D": stone["Position3D"]})
        copter.invoke_sync("PlaceBlock", {"Position3D": stone["Position3D"]})

        self.invoke_sync("FreeCopter", {"Device": copter})
        return params

    def SetWall(self, params: dict):
        self.wall = params["Vector3"]
        self.__assign_placer_to_wall()
        return {}

    def GetBlocks(self, params: dict) -> dict:
        return {"ParameterList": self.blocks}

    def SetBlocks(self, params: dict) -> dict:
        self.blocks = params["ParameterList"]
        return self.GetBlocks({})

    def IsBlockOnWall(self, params: dict):
        if len(self.wall) > 0:
            p = np.sum(np.array(self.wall[:3]) * np.array(params["Vector3"]))
            return {"bool": float(np.abs(p - self.wall[3])) < 1e-3}
        else:
            raise ValueError("Wall has not been setup!")

    def __assign_placer_to_wall(self):
        if len(self.wall) > 0 and len(self.cars) > 0:
            for car in self.cars:
                car.invoke_sync("SetPosition", {"Position3D":self.wall[:3]})
                car.invoke_sync("SetRotation", {"Quaternion":self.wall[6:10]})

    def loop(self):
        sleep(.0001)


if __name__ == '__main__':
    # Needed for properly closing when process is being stopped with SIGTERM signal
    def handler(signum, frame):
        print("[Main] Received SIGTERM signal")
        listener.kill()
        quit(1)


    try:
        port = None
        ip = None
        if len(sys.argv[1:]) > 0:
            port = int(sys.argv[1])
        if len(sys.argv[2:]) > 0:
            ip = str(sys.argv[2])
        server = VirtualCapabilityServer(port, ip)
        listener = WallManager(server)
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
