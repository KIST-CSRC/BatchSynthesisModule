#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##s
# @brief    [Linear Actuator] motion basic class for linear actuator
# @author   Hyuk Jun Yoo (yoohj9475@kist.re.kr)   
# @version  1_1   
# TEST 2021-09-23

import socket
import os, sys
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))
from BaseUtils.TCP_Node import BaseTCPNode

class ParameterLA:
    """Linear Actuator IP, PORT, location dict, move_z"""
    def __init__(self, ):
        self.LA_info={
            "HOST_LA":"127.0.0.1",
            "PORT_LA":54010,
            "Stirrer_location_dict" : 
            {
                "Stirrer_0" : [
                    {"x" : 875300, "y" : 110690},
                    {"x" : 804600, "y" : 111100},
                    {"x" : 715065, "y" : 109950},
                    {"x" : 643846, "y" : 109021},

                    {"x" : 875260, "y" : 179467},
                    {"x" : 805400, "y" : 178102},
                    {"x" : 713550, "y" : 180304},
                    {"x" : 643169, "y" : 177497},

                    {"x" : 873275, "y" : 271356},
                    {"x" : 804918, "y" : 271356},
                    {"x" : 713797, "y" : 270526},
                    {"x" : 641801, "y" : 271707},

                    {"x" : 872730, "y" : 343790},
                    {"x" : 803036, "y" : 341760},
                    {"x" : 712200, "y" : 341760},
                    {"x" : 642916, "y" : 341760}
                ]
            },
            "move_z":{
                "up" : 125000,
                "down" : 145000
            }
        }

class Science_Town_Class(ParameterLA, BaseTCPNode):
    """
    [Linear Actuator] Science Town's linear actuator Class for controlling in another computer (windows)

    # Variable
    :param logger_obj (obj): set logging object (from Logging_class import Loggger) 
    :param device_name="Science_town" (string): set LA model name (log name)
    

    # function
    1. _callServer_LA(command_byte)
    2. hello()
    3. moveHome()
    4. move_to_stirrer(stirrer_address, location_number, mode="down")
    """
    def __init__(self, logger_obj, device_name="Science_town"):
        ParameterLA.__init__(self)
        BaseTCPNode.__init__(self)
        self.logger_obj=logger_obj
        self.device_name=device_name

    def _callServer_LA(self, command_byte):
        res_msg=self.callServer(self.LA_info["HOST_LA"], self.LA_info["PORT_LA"], command_byte)
        return res_msg

    def hello(self):
        """
        get connection status using TCP/IP (socket)
        
        :return res_msg (str): "Hello World!! Succeed to connection to main computer!"
        """
        debug_device_name="{} ({})".format(self.device_name, "virtual")
        hello_command_byte = str.encode("{}".format("hello"))
        res_msg=self._callServer_LA(command_byte=hello_command_byte)
        
        self.logger_obj.debug(device_name=debug_device_name, debug_msg=res_msg)
            
        return res_msg

    def moveHome(self, mode_type="virtual"):
        """
        move home location (location information in cpp files)

        :param mode_type="virtual" (str): set virtual or real mode

        :return res_msg (str): response_message
        """
        debug_device_name="{} ({})".format(self.device_name, mode_type)
        self.logger_obj.debug(device_name=debug_device_name,debug_msg="start moving to home")
        if mode_type=="real":
            home_command_byte = str.encode("{}".format("home"))
            res_msg=self._callServer_LA(home_command_byte)
        elif mode_type=="virtual":
            res_msg=self.hello()
        self.logger_obj.debug(device_name=debug_device_name,debug_msg=res_msg)

        return_res_msg="[{}] : {}".format(debug_device_name, res_msg)
        return return_res_msg

    def move2Stirrer(self, action_type, stirrer_address, location_number, mode_type="virtual"):
        """
        previous : 8,0,1 --> 8 : stirring number, 0,1 --> down
        move stirring stirrer point depending on stirring_number

        :param action_type (str) : input z position mode (ex. "down" or "up")
        :param stirrer_address (str) : input stirrer name (ex. "Stirrer_0", "Stirrer_1") 
        :param location_number (int) : input location name (ex. 0, 1, 2, 3, 4, ....) 
        :param mode_type="virtual" (str): set virtual or real mode

        :return res_msg (str): response_message
        """
        debug_device_name="{} ({})".format(self.device_name, mode_type)
        """
            Stirrer_0 --> 0 으로 할당하는 function
        """
        self.logger_obj.debug(device_name=debug_device_name,debug_msg="start moving to {}, specific location : {}, mode : {}".format(stirrer_address, location_number, action_type))
        res_msg=""
        if mode_type=="real":
            if action_type == "down":
                command_byte = str.encode('{},{},{}'.format(self.LA_info["Stirrer_location_dict"][stirrer_address][int(location_number)]["x"], self.LA_info["Stirrer_location_dict"][stirrer_address][int(location_number)]["y"], self.LA_info["move_z"][action_type]))
            elif action_type == "up":
                command_byte = str.encode('{},{},{}'.format(self.LA_info["Stirrer_location_dict"][stirrer_address][int(location_number)]["x"], self.LA_info["Stirrer_location_dict"][stirrer_address][int(location_number)]["y"], self.LA_info["move_z"][action_type]))
            res_msg = self._callServer_LA(command_byte=command_byte)
        elif mode_type=="virtual":
            res_msg = "{}, location {}, mode {}".format(stirrer_address, str(location_number), action_type)
        
        self.logger_obj.debug(device_name=self.device_name, debug_msg=res_msg)
            
        return_res_msg="[{}] : {}".format(debug_device_name, res_msg)
        return return_res_msg