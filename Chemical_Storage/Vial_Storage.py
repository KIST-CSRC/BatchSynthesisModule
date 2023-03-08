#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [Vial Storage] Vial Storage file
# @author   Hyuk Jun Yoo (yoohj9475@kist.re.kr)   
# @version  1_1   
# TEST 2021-09-11

import os, sys
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from Log.Logging_Class import NodeLogger
from serial_labware import SerialDevice, command
import serial
import time

class ParamterVialStorage(object):
    def __init__(self):
        # serial settings
        self.storage_info={
            "PORT":"COM13",
            "BAUD_RATE":9600
        }

class VialStorage(ParamterVialStorage):
    """
    [VialStorage] VialStorage Class for controlling in another computer (windows)

    # Variable
    :param logger_obj (obj): set logging object (from Logging_class import Loggger) 
    :param device_name="Vial_Storage" (string): set Storage model name (log name)
    :param PORT="COM9" (str): set LA model name (log name)
    :param BAUD_RATE=9600 (int) 
    :param mode_type="virtual" (str): set virtual or real mode

    # function
    1. vial_Entrance(entrance_num, mode_type="virtual")
    """
    def __init__(self, logger_obj, device_name="Vial_Storage"):
        ParamterVialStorage.__init__(self,)
        self.logger_obj=logger_obj
        self.device_name=device_name
        self.arduinoData = serial.Serial(self.storage_info["PORT"], self.storage_info["BAUD_RATE"])

    def hello(self,):
        self.arduinoData.write("1".encode())
        debug_msg="Hello World!! Succeed to connection to main computer!"
        self.logger_obj.debug(device_name=self.device_name, debug_msg=debug_msg)

        return_res_msg="[{}] : {}".format(self.device_name, debug_msg)
        return return_res_msg

    def openEntrance(self, entrance_num, mode_type="virtual"):
        """
        control vial storage's Entrance (1,2,3,4,5 : Bottom Site (Empty Vial)) (6,7,8,9,10 : Top Site (Full Vial))

        :param entrance_num (int): Entrance Number
        :param mode_type="virtual" (str): set virtual or real mode
        
        :return: return_res_msg => [Vial Storage ({mode_type})] : ~~
        """
        device_name="Vial Storage ({})".format(mode_type)
        if mode_type=="real":
            debug_msg="open {} entrance".format(entrance_num)
            self.logger_obj.debug(device_name=device_name, debug_msg=debug_msg)

            time.sleep(2)
            value = entrance_num
            self.arduinoData.write(value.encode())
            if int(entrance_num) <=5:
                time.sleep(2)
            else:
                time.sleep(5)
            value = "-"+entrance_num
            self.arduinoData.write(value.encode())
            time.sleep(2)

            debug_msg="Finished to open {} entrance".format(entrance_num)
            self.logger_obj.debug(device_name=device_name, debug_msg=debug_msg)
            
            return_res_msg="[{}] : {}".format(device_name, debug_msg)
            return return_res_msg

        elif mode_type=="virtual":
            debug_msg="open {} entrance".format(entrance_num)
            self.logger_obj.debug(device_name=device_name, debug_msg=debug_msg)
            debug_msg="Finished to open {} entrance".format(entrance_num)
            self.logger_obj.debug(device_name=device_name, debug_msg=debug_msg)

            return_res_msg="[{}] : {}".format(device_name, debug_msg)
            return return_res_msg