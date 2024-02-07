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
        self.copter_swarm = None
        self.block_handler = None
        self.blocks = []

    def tick(self, params: dict):
        copter = self.invoke_sync("GetAvaiableCopter", params)["Device"]


        return params

    def GetSwarm(self, params: dict):
        return {"Device": self.copter_swarm}

    def SetSwarm(self, params: dict):
        self.copter_swarm = json.loads(params["Device"])
        return self.GetSwarm({})

    def GetBlockHandler(self, params: dict):
        return {"Device": self.block_handler}

    def SetBlockHandler(self, params: dict):
        self.block_handler = json.loads(params["Device"])
        return self.GetBlockHandler({})

    def GetBlocks(self, params: dict) -> dict:
        return {"blocks": self.blocks}

    def SetBlocks(self, params: dict) -> dict:
        self.blocks = params["blocks"]
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
