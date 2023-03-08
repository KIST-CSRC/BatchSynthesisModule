#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [BatchPlatformServer] control hardware for batch synthesis
# @author   Hyuk Jun Yoo (yoohj9475@kist.re.kr)   
# TEST 2021-10-18

from Chemical_Storage.Vial_Storage import VialStorage
from Linear_Actuator.Linear_Actator_Class import Science_Town_Class, ParameterLA
from Stirrer.hotplate import Hotplate_custom
from Syringe_Pump.Syringe_Class import Centris, ParameterPump
from Log.Logging_Class import NodeLogger
from BaseUtils.TCP_Node import BaseTCPNode
import socket
import json
import time
import multiprocessing

SIZE = 1048576

# # TCP/IP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('161.122.22.146', 54009))  # ip address, port
server_socket.listen() # wait status requirement of client connection

NodeLogger_obj = NodeLogger(platform_name="Batch Synthesis", setLevel="DEBUG",
                            SAVE_DIR_PATH="C:/Users/KIST/PycharmProjects/BatchPlatform")

STORAGE_obj = VialStorage(NodeLogger_obj)
LA_obj = Science_Town_Class(NodeLogger_obj)
STIRRER_obj = Hotplate_custom(NodeLogger_obj)

base_tcp_node_obj = BaseTCPNode()

def callPump(solution_name, volume, flow_rate=2000, mode_type="virtual"):
    param_pump_obj = ParameterPump()
    pump_info = param_pump_obj.pump_info[solution_name]
    if pump_info["deviceName"] == "Centris":
        Centris_obj = Centris(logger_obj=NodeLogger_obj, device_name=pump_info["deviceName"],
                              solution_name=solution_name, tecan_addr=pump_info["pumpAddress"],
                              ser_port=pump_info["pumpUsbAddr"], baud=0, syringe_volume=5000, max_attempts=10)
        res_msg = Centris_obj.add(volume=volume, flow_rate=flow_rate, mode_type=mode_type)
    elif pump_info["deviceName"] == "XCaliburD":
        XCaliburD_obj = XCaliburD(logger_obj=NodeLogger_obj, device_name=pump_info["deviceName"],
                                  solution_name=solution_name, tecan_addr=pump_info["pumpAddress"],
                                  ser_port=pump_info["pumpUsbAddr"], baud=0, syringe_volume=5000, max_attempts=10)
        res_msg = XCaliburD_obj.add(volume=volume, flow_rate=flow_rate, mode_type=mode_type)

    return res_msg

try:
    while True:
        client_socket, addr = server_socket.accept()  # accept connection. return ip, port 
        data = client_socket.recv(SIZE)  # recieve data from client. print buffer size

        packet_info = str(data.decode()).split(sep="/")
        print("packet information list : ", packet_info)

        if packet_info[0] == "LA":
            if packet_info[1] == "home":  # go home
                hardware_name, action_type, mode_type = packet_info
                res_msg = LA_obj.moveHome(mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type)
            elif packet_info[1] == "down" or "up":  # go stirrer's specific location
                hardware_name, action_type, stirrer_address, location_number, mode_type = packet_info
                res_msg = LA_obj.move2Stirrer(action_type, stirrer_address, location_number, mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type)
            else:
                raise ValueError("[{}] Packet Error : Packet length is different".format(hardware_name))

        elif packet_info[0] == "STIRRER":
            if packet_info[1] == 'heat':
                hardware_name, action_type, stirrer_address, action_info, mode_type = packet_info
                res_msg = STIRRER_obj.controlTemperature(set_temp=action_info, mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type)
            elif packet_info[1] == 'stir':
                hardware_name, action_type, stirrer_address, action_info, mode_type = packet_info
                res_msg = STIRRER_obj.controlStirrer(set_stir_rate=action_info, mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type)
            elif packet_info[1] == 'stop':
                hardware_name, action_type, mode_type = packet_info
                res_msg = STIRRER_obj.controlStop(mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type)
            else:
                raise ValueError("[{}] Packet Error : Packet value is different".format(hardware_name))

        elif packet_info[0] == "PUMP":
            hardware_name, action_type, solution_name, volume, flow_rate, mode_type = packet_info
            if action_type == "single":
                res_msg = callPump(solution_name, volume, flow_rate, mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type)
            elif action_type == "multi":
                total_status = 0
                solution_name_list = list(map(str, solution_name.split(sep=",")))
                volume_list = list(map(float, volume.split(sep=",")))
                flow_rate_list = list(map(float, flow_rate.split(sep=",")))
                for solution_idx in range(len(solution_name_list)):  # matching 1 vial --> 1 action
                    res_msg = callPump(solution_name_list[solution_idx], volume_list[solution_idx],
                                    flow_rate_list[solution_idx], mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type)
            else:
                raise ValueError("[{}] Packet Error : action_type is wrong".format(action_type))

        elif packet_info[0] == "STORAGE":
            hardware_name, entrance_num, mode_type = packet_info
            res_msg = STORAGE_obj.openEntrance(entrance_num, mode_type=mode_type)
            base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type="Open & Close")

        elif packet_info[0] == "BATCH":
            hardware_name, action_type, _ = packet_info
            total_dict = {}
            param_pump_obj = ParameterPump()
            param_LA_obj = ParameterLA()
            LA_obj.hello()
            STIRRER_obj.hello()
            STORAGE_obj.hello()
            total_dict["PUMP"]=param_pump_obj.pump_info
            total_dict["LA"]=param_LA_obj.LA_info
            base_tcp_node_obj.checkSocketStatus(client_socket, total_dict, hardware_name, action_type)
        else:
            raise ValueError("[{}] Packet Error : hardware_name is wrong".format(hardware_name))

except KeyboardInterrupt:
    print('Ctrl + C, interrupt message')