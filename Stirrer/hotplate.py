# !usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief [Hotplate] Hotplate class using IKA_RET_Visc in Autonomous System Laboratory
# @author Na Yeon Kim (kny@kist.re.kr), Hyuk Jun Yoo (yoohj9475@kist.re.kr)
# TEST

import os, sys
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))  # get import path : logger.py
from Stirrer.IKA_RET_Control_Visc import IKARETControlVisc
from Log.Logging_Class import NodeLogger
import time
import socket
from multiprocessing import Process

SIZE = 4096


class Hotplate_custom(IKARETControlVisc):
    '''
    This class is Controling Hotplate for Heating or Stirring with IKA_RET.

    :logger_obj (obj) : port number of each Hotplate. (ex. NodeLogger)
    :device_name (int) : device name of each Hotplate. (ex. "IKA_RET")
    :device_address (int) : port number of each Hotplate. (ex. 0)
    '''
    def __init__(self, logger_obj, device_name="IKA_RET", device_address=0):
        super().__init__(port='COM5', device_name=device_name, connect_on_instantiation=True)
        self.logger_obj=logger_obj
        self.device_name=device_name
        self.device_address = device_address
    
    def hello(self,):
        res_msg, _ = self.temperature_hot_plate_pv
        debug_msg="Hello World!! Succeed to connection to main computer!"
        self.logger_obj.debug(device_name=self.device_name, debug_msg=debug_msg)

        return res_msg

    def controlTemperature(self, set_temp=0, mode_type="virtual", *set_waiting_time):
        '''
        This function is Controling temperature of hotplate.

        :param set_temp (int) : target temperature. (Limitation : roomtemp~340 celcius)
        :param mode_type="virtual" (str) : set mode type
        :param set_waiting_time (int) : Retention Time(sec)

        return return_res_msg
        '''
        debug_device_name="{} ({})".format(self.device_name, mode_type)
        self.logger_obj.debug(device_name=debug_device_name, debug_msg="Start controlling temperature in Stirrer")
        if mode_type=="real":
            self.temperature_sp = set_temp  # Temperature setting
            self.start_heater()  # Turn on the heater
            if set_waiting_time:
                time.sleep(set_waiting_time[0])  # Waiting
                self.stop_heater()  # Turn off the heater
        elif mode_type=="virtual":
            pass
        self.logger_obj.debug(device_name=debug_device_name, debug_msg="Finish controlling temperature in Stirrer")
        return_res_msg="[{}] : {}".format(self.device_name, "Finish controlling temperature in Stirrer")
        return return_res_msg

    def controlStirrer(self, set_stir_rate=0, mode_type="virtual", *set_waiting_time):
        '''
        This function is Controling stirring rate of hotplate.

        :param set_stir_rate (int) : target stirring rate. (Limitation : 0~1700 rpm)
        :param mode_type="virtual" (str) : set mode type
        :param set_waiting_time(int) : Retention Time(sec)

        return return_res_msg
        '''

        debug_device_name="{} ({})".format(self.device_name, mode_type)
        self.logger_obj.debug(device_name=debug_device_name, debug_msg="Start controlling stirring in Stirrer")
        if mode_type=="real":
            self.stir_rate_sp = set_stir_rate  # stirring setting
            self.start_stirrer()  # Turn on the heater
            if set_waiting_time:
                time.sleep(set_waiting_time[0])  # Waiting
                self.stop_stirrer()  # Turn off the stirrer
        elif mode_type=="virtual":
            pass
        self.logger_obj.debug(device_name=debug_device_name, debug_msg="Finish controlling stirring in Stirrer")
        return_res_msg="[{}] : {}".format(self.device_name, "Finish controlling stirring in Stirrer")
        return return_res_msg

    def controlStop(self, mode_type):
        '''
        This function is stop stirring rate & temperature of hotplate.

        :param mode_type="virtual" (str) : set mode type

        return return_res_msg
        '''
        debug_device_name="{} ({})".format(self.device_name, mode_type)
        self.logger_obj.debug(device_name=debug_device_name, debug_msg="Stop controlling stirring & temperature in Stirrer")
        if mode_type=="real":
            self.stop_stirrer()
            self.stop_heater()
        elif mode_type=="virtual":
            pass
        self.logger_obj.debug(device_name=debug_device_name, debug_msg="Finish Stop stirring & temperature in Stirrer")
        return_res_msg="[{}] : {}".format(self.device_name, "Finish Stop stirring & temperature in Stirrer")
        return return_res_msg