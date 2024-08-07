#!/usr/bin/env python
import json
import os.path
import signal
import sys
from copy import deepcopy
from threading import Thread, Lock
from typing import List

import numpy as np
import quaternion
from time import sleep
from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint, \
    SubDeviceRepresentation, CapabilityErrorException


class WallManager(AbstractVirtualCapability):

    def __init__(self, server):
        super().__init__(server)
        self.wall = []
        self.cars: List[SubDeviceRepresentation] = []
        self.charging_station = None
        self.car_lock: List[Lock] = []
        self.block_handler = None
        self.blocks = []
        # Matches BuildPlan integer id to blockhandler integer id
        self.fitted_blocks = {}
        self.f = f"/home/z/Desktop/Trash/{self.server.connectionPort}.log"
        self.id = -1

    def SyncAlreadyFitted(self, params: dict):
        #  This comes from another WallManager
        if params is not None:
            self.fitted_blocks.update(params["FittedBlocks"])
            if params["int"] != self.id:
                with open(self.f, mode='a') as logger:
                    logger.write(f"in@" + str(params["int"]) + "  -  " + str(dict(params["FittedBlocks"]).values()))
                    logger.write(f"\n{self.fitted_blocks.values()}\n")
            if params["FittedBlocks"] != self.fitted_blocks and len(params["FittedBlocks"]) < len(self.fitted_blocks):
                # Syncing further
                with open(self.f, mode='a') as logger:
                    logger.write("-------------------------------------------------------------------------\n")
                    logger.write(f"START{self.id}: {self.fitted_blocks.values()}\n")
                self.invoke_sync("SyncAlreadyFitted", {"FittedBlocks": self.fitted_blocks, "int": self.id})

        return {"FittedBlocks": self.fitted_blocks}

    def SetupWall(self, params: dict) -> dict:
        self.invoke_sync("InitializeSwarm", {"int": 3})
        self.charging_station = self.query_sync("ChargingStation", 0)
        self.block_handler = self.query_sync("BlockHandler")
        cnt = 1 # params["int"]
        self.id = params["int"]
        self.f = f"/home/z/Desktop/Trash/{self.id}.log"
        for i in range(cnt - len(self.cars)):
            self.cars.append(self.query_sync("PlacerRobot", -1))
            self.car_lock.append(Lock())
        self.wall = params["Vector3"]
        starting_point = params["ListOfPoints"]
        self.__assign_placer_to_wall(starting_point)
        return {"DeviceList": self.cars}

    def acquire_stone(self, copter, new_stone):
        # Copter takes stone
        stone_id = copter.invoke_sync("GetBlock", {})["SimpleIntegerParameter"]
        if stone_id == -1:
            new_block = self.block_handler.invoke_sync("SpawnBlock", {"Vector3": new_stone["Vector3"]})
            copter.invoke_sync("SetPosition", {"Position3D": new_block["Position3D"]})
            copter.invoke_sync("TransferBlock", {"SimpleIntegerParameter": new_block["SimpleIntegerParameter"]})
            stone_id = new_block["SimpleIntegerParameter"]
        copter.invoke_sync("SetRotation", {"Quaternion": new_stone["Quaternion"]})
        return stone_id

    def WallTick(self, params: dict):
        stone = self.__get_next_block()
        if stone is None:
            if set([b["int"] for b in self.blocks]) <= self.fitted_blocks.keys():
                raise ValueError(f"All Stones are placed in this wallsection")
        while stone is None:
            with open(self.f, mode='a') as logger:
                logger.write("-------------------------------------------------------------------------\n")
                logger.write(f"START{self.id}: {self.fitted_blocks.values()}\n")
            self.invoke_sync("SyncAlreadyFitted", {"FittedBlocks": self.fitted_blocks, "int": self.id})
            #raise ValueError(f"I finished {self.fitted_blocks}")
            sleep(.5)
            stone = self.__get_next_block()
            while self.car_lock[0].locked():
                # Dont need to query blocks while car is unable to move
                sleep(1)
        formatPrint(self, f"Setting new stone: {stone}")

        copter = SubDeviceRepresentation(self.invoke_sync("GetAvaiableCopter", params)["Device"], self, None)
        new_block = self.acquire_stone(copter, stone)

        if self.car_lock[0].locked():
            self.invoke_sync("FreeCopter", {"Device": copter})
            while self.car_lock[0].locked():
                sleep(.001)
            copter = SubDeviceRepresentation(self.invoke_sync("GetAvaiableCopter", params)["Device"], self, None)
            new_block = self.acquire_stone(copter, stone)

        with self.car_lock[0]:
            pos = self.cars[0].invoke_sync("SetPosition", {"Position3D": stone["Position3D"]})
            copter.invoke_sync("SetPosition", pos)
            self.cars[0].invoke_sync("Transferblock", {"SimpleIntegerParameter": new_block})
            copter.invoke_sync("TransferBlock", {"SimpleIntegerParameter": -1})
            self.invoke_sync("FreeCopter", {"Device": copter})
            self.cars[0].invoke_sync("PlaceBlock", {"Position3D": stone["Position3D"]})
            self.fitted_blocks[stone["int"]] = new_block
            return params

    def GetBlocks(self, params: dict) -> dict:
        return {"ParameterList": self.blocks}

    def SetBlocks(self, params: dict) -> dict:
        self.blocks = params["ParameterList"]
        id_list = [b["int"] for b in self.blocks]
        formatPrint(self, f"MYBLOKS size={len(id_list)}: " + str(id_list))
        return self.GetBlocks({})

    def IsBlockOnWall(self, params: dict):
        if len(self.wall) > 0:
            p = np.sum(np.array(self.wall[:3]) * np.array(params["Vector3"]))
            return {"bool": float(np.abs(p - self.wall[3])) < 1e-3}
        else:
            raise ValueError("Wall has not been setup!")

    def __assign_placer_to_wall(self, starting_point):
        if len(self.wall) > 0 and len(self.cars) > 0:
            for car in self.cars:
                car.invoke_sync("SetPosition", {"Position3D": np.array(starting_point[:3])})
                car.invoke_sync("SetRotation", {"Quaternion": starting_point[6:10]})

    def __get_all_available_blocks(self):
        available_blocks = []
        for block in self.blocks:
            key = str(block["int"])
            if key not in self.fitted_blocks:
                dependency_resolved = True
                for dependency in block["depends_on"]:
                    dependency_resolved &= str(dependency) in self.fitted_blocks
                if not dependency_resolved:
                    continue
                available_blocks += [deepcopy(block)]
        return available_blocks

    def __get_next_block(self):
        next_block = None
        avaialable_blocks = self.__get_all_available_blocks()
        if len(avaialable_blocks) <= 0:
            return next_block
        car_pos = self.cars[0].invoke_sync("GetPosition", {})["Position3D"]
        shortest_path_length = np.finfo(float).max
        for block in avaialable_blocks:
            # euclid distance car - (future)block
            dist = np.sum(np.sqrt((np.array(car_pos) - np.array(block["Position3D"])) ** 2))
            if dist < shortest_path_length:
                shortest_path_length = dist
                next_block = deepcopy(block)
        return next_block

    def loop(self):
        if len(self.cars) > 0:
            for i, car in enumerate(self.cars):
                battery_lvl = car.invoke_sync("GetBatteryChargeLevel", {})["BatteryChargeLevel"]
                if battery_lvl < 15.:
                    formatPrint(self, f"Loading Car: {car.ood_id}")
                    self.car_lock[0].acquire()
                    car.invoke_sync("SetPosition", self.charging_station.invoke_sync("GetPosition", {}))
                    self.charging_station.invoke_async("ChargeDevice", {"Device": car}, lambda *args: self.car_lock[0].release())
        sleep(10)

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
        listener.uri = "WallManager"
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
