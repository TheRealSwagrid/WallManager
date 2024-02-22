#!/usr/bin/env python
import json
import os.path
import signal
import sys
from copy import deepcopy

import numpy as np
import quaternion
from time import sleep
from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


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
        return {"DeviceList": self.cars}

    def WallTick(self, params: dict):
        copter = self.invoke_sync("GetAvaiableCopter", params)["Device"]
        stone = self.blocks.pop()
        formatPrint(self, stone)

        self.invoke_sync("FreeCopter", {"Device": copter})
        return params

    def SetWall(self, params: dict):
        self.wall = params["Vector3"]
        return {}

    def GetBlocks(self, params: dict) -> dict:
        return {"ParameterList": self.blocks}

    def SetBlocks(self, params: dict) -> dict:
        self.blocks = params["ParameterList"]
        return self.GetBlocks({})

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
