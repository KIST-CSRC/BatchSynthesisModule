#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [Syringe_Pump] Tecan Cavro Pump Class for controlling Syringe Pump (Centris, XCaliburD)
# @author   Hyuk Jun Yoo (yoohj9475@kist.re.kr)   
# TEST 2021-09-24

import os
import sys
import time
import copy
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "Syringe_Pump_Package")))  # get import path : TecanAPISerial class in transport.py
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))
from transport import TecanAPISerial
from Log import Logging_Class


class ParameterPump:
    """
    solution_type_addr_dict
    pump_addr_dict
    pump_device_address
    dispense_init
    """
    def __init__(self):
        self.pump_info={
            "AgNO3":
                {"solutionType":"Metal",
                "pumpAddress":0,
                "pumpUsbAddr":"COM8",
                "deviceName":"Centris"},
            "H2O":
                {"solutionType":"Solvent",
                "pumpAddress":1,
                "pumpUsbAddr":"COM8",
                "deviceName":"Centris"},
            "H2O2":
                {"solutionType":"Reductant",
                "pumpAddress":2,
                "pumpUsbAddr":"COM8",
                "deviceName":"Centris"}
                ,
            "NaBH4":
                {"solutionType":"Oxidant",
                "pumpAddress":3,
                "pumpUsbAddr":"COM7",
                "deviceName":"Centris"}
                ,
            "Citrate":
                {"solutionType":"CA",
                "pumpAddress":4,
                "pumpUsbAddr":"COM7",
                "deviceName":"Centris"},
            "PVP55":
                {"solutionType":"CA",
                "pumpAddress":5,
                "pumpUsbAddr":"COM7",
                "deviceName":"Centris"}
        }

    def preprocessPumpInformation(self):
        """
        preprocess Pump Information in dict

        make object variable via this function

        ex) self.solution_name_list=list(self.pump_info.keys()) # ["AgNO3", "H2O", "PVP55"... ]
            self.solution_addr_list=list(self.pump_info.values()) # [{"PumpAddress":2,"PumpUsbAddr":"/dev/ttyUSB2","DeviceName":"Centris"},...]
            self.solution_type_list=[] # ["Metal", "Solvent", "Salt", "CA", "Reductant"]

        :return preprocess_pump_info_dict (dict): 
        ex) {
            'solutionNameList': ['AgNO3', 'H20', 'H2O2', 'NaBH4', 'Citrate', 'PVP55'], 
            'solutionAddrList': [
                    {'solutionType': 'Metal', 'pumpAddress': 0, 'pumpUsbAddr': 'COM8', 'deviceName': 'Centris'}, 
                    {'solutionType': 'Solvent', 'pumpAddress': 1, 'pumpUsbAddr': 'COM8', 'deviceName': 'Centris'}, 
                    {'solutionType': 'Oxidant', 'pumpAddress': 2, 'pumpUsbAddr': 'COM8', 'deviceName': 'Centris'}, 
                    {'solutionType': 'Reductant', 'pumpAddress': 3, 'pumpUsbAddr': 'COM7', 'deviceName': 'Centris'}, 
                    {'solutionType': 'CA', 'pumpAddress': 4, 'pumpUsbAddr': 'COM7', 'deviceName': 'Centris'}, 
                    {'solutionType': 'CA', 'pumpAddress': 5, 'pumpUsbAddr': 'COM7', 'deviceName': 'Centris'}
                ], 
            'solutionTypeList': ['Metal', 'Solvent', 'Oxidant', 'Reductant', 'CA', 'CA']
        }
        """
        self.solution_name_list=list(self.pump_info.keys()) # ["AgNO3", "H2O", "PVP55"... ]
        self.solution_addr_list=list(self.pump_info.values()) # [{"PumpAddress":2,"PumpUsbAddr":"/dev/ttyUSB2","DeviceName":"Centris"},...]
        self.solution_type_list=[] # ["Metal", "Solvent", "Salt", "CA", "Reductant"]
        for solution_addr_dict in self.solution_addr_list:
            self.solution_type_list.append(self.solution_addr_dict["solutionType"])
        preprocess_pump_info_dict={}
        preprocess_pump_info_dict["solutionNameList"]=self.solution_name_list
        preprocess_pump_info_dict["solutionAddrList"]=self.solution_addr_list
        preprocess_pump_info_dict["solutionTypeList"]=self.solution_type_list

        return preprocess_pump_info_dict

class Centris(object):
    """
    [Centris] Centris Class to control Cavro Centris in tecan cavro

    This class input logger_obj, so that our class use logging class in this pump class.
    if you want to consider the law of logging function, please refer Log/Logging_Class.py

    # Variable
    :param logger_obj (obj): set logging object (from Logging_class import Loggger)
    :param device_name (str): set pump name ex) Centris (Log name)
    :param tecan_addr (int): physical address in syringe pump (back of pump, seems like clock) ex) 0,1,2,3...
    :param ser_port (int): COM8 or COM7 depending on Centris' Multi Hub ex) COM8
    :param baud (int): port number which has own number ex) 0,1,2....
    :param syringe_volume (int): glass or plastic syringe's volume (dimension : ul) ex) 500 or 5000 (==5ml)
    :param ini (bool): choose type of action (initialize or not)
    :param port_num (int or None): set the number of port (3 or 9, default=3)
    :param max_attempts (int): maximum attempts is 10. 
    :param mode_type="virtual" (str): set virtual or real mode

    # function
    - initialize(flow_rate=1000)
    - add(volume, speed)
    - getStatus()
    - getConfiguration()
    - updateSpeeds()
    - getPlungerPos()
    - getCurrentPlunger()
    - getEncoderPos()
    - getInitializationGapSteps()
    - getBackSlashIncrements()
    - getStartSpeed()
    - getTopSpeed()
    - getCutoffSpeed()
    - getSyringeVolume()
    - getPlungerPosInSyringe()
    - getCurPort()
    - terminateCmd()
    - moveMaintainencePoint()
    - moveInputPoint()
    """
    ERROR_DICT = {
        0: "Non Error",
        1: 'Initialization Error',
        2: 'Invalid Command',
        3: 'Invalid Operand',
        7: 'Device Not Initialized',
        8: 'Invalid Valve Configuration',
        9: 'Plunger Overload',
        10: 'Valve Overload',
        11: 'Plunger Move Not Allowed',
        12: 'Extended Error Present',
        13: 'Nvmem Access Failure',
        14: 'Command Buffer Empty or Not Ready',
        15: 'Command Buffer Overflow'
    }

    def __init__(self, logger_obj, device_name="Centris", solution_name="", tecan_addr=0, ser_port="COM8", baud=0, syringe_volume=5000, port_num=None, ini=False, max_attempts=10):
        # logger object
        self.logger_obj=logger_obj

        # device debugrmation
        self.device_name_only=device_name
        self.solution_name=solution_name
        self.base_increment=181490
        self.device_name = "{} pump {}".format(device_name, str(tecan_addr))
        self.tecan_addr = tecan_addr
        self.ser_port = ser_port
        self.baud = 9600+baud
        self.syringe_volume = syringe_volume
        self.ini = ini
        self.max_attempts = max_attempts
        self.cmd_string = ""
        self.syringe_pump = TecanAPISerial(self.tecan_addr, self.ser_port, self.baud, max_attempts=self.max_attempts)
        self.state = {
            'plunger_pos': None,
            'port': None,
            'start_speed': None,
            'top_speed': None,
            'cutoff_speed': None,
        }
        self.port_num=port_num
        self.initialize()

    #########################################################################
    # Report commands (cannot be chained)                                   #
    #########################################################################

    def _divideSpeed(self,initSpeed):
        return int(initSpeed)

    def getStatus(self):
        """
        check syringe pump's status
        status_dict --> 01000000 or 01100000 (work done == not busy) or 0(working == busy)

        :return: status_dict
        """
        cmd_string = "?29"
        status_dict = self.syringe_pump.sendRcv(cmd_string)
        return status_dict

    def getConfiguration(self):
        """
        check syringe pump's Configuration

        :return: status_dict
        """
        cmd_string = "?76"
        configuration_dict = self.syringe_pump.sendRcv(cmd_string)
        return configuration_dict

    def updateSpeeds(self):
        self.getStartSpeed()
        self.getTopSpeed()
        self.getCutoffSpeed()

    def getPlungerPos(self):
        """ 
        Returns the absolute plunger position as an int (0-3000) 
        
        return --> 1600
        """
        cmd_string = '?'
        data = self.syringe_pump.sendRcv(cmd_string)
        self.state['plunger_pos'] = int(data["data"])
        return self.state['plunger_pos']

    def getCurrentPlunger(self):
        """ 
        Returns the Current Plunger
        
        return --> b'0'
        """
        cmd_string = '?1'
        data = self.syringe_pump.sendRcv(cmd_string)
        return data["data"]

    def getEncoderPos(self):
        """ Returns the current encoder count on the plunger axis """
        cmd_string = '?2'
        data = self.syringe_pump.sendRcv(cmd_string)
        return float(data["data"])

    def getInitializationGapSteps(self):
        """ Returns the plunger Initialization Gap Steps """
        cmd_string = '?3'
        data = self.syringe_pump.sendRcv(cmd_string)
        return data["data"]
        
    def getBackSlashIncrements(self):
        """ Returns the start speed as an int (in pulses/sec) """
        cmd_string = '?4'
        data = self.syringe_pump.sendRcv(cmd_string)
        return data["data"]

    def getStartSpeed(self):
        """ Returns the start speed as an int (in ul/sec) """
        cmd_string = '?6'
        data = self.syringe_pump.sendRcv(cmd_string)
        self.state['start_speed'] = int(data["data"])
        return self.state['start_speed']

    def getTopSpeed(self):
        """ Returns the top speed as an int (in pulses/sec) """
        cmd_string = '?7'
        data = self.syringe_pump.sendRcv(cmd_string)
        self.state['top_speed'] = int(data["data"])
        return self.state['top_speed']

    def getCutoffSpeed(self):
        """ Returns the cutoff speed as an int (in pulses/sec) """
        cmd_string = '?8'
        data = self.syringe_pump.sendRcv(cmd_string)
        self.state['cutoff_speed'] = int(data["data"])
        return self.state['cutoff_speed']

    def getSyringeVolume(self):
        """ Returns the Syringe Volume (5000 or 500)"""
        cmd_string = '?17'
        data = self.syringe_pump.sendRcv(cmd_string)
        return int(data["data"])

    def getPlungerPosInSyringe(self):
        """ Returns the Plunger Pos In Syringe depending on 5000 or 500"""
        cmd_string = '?18'
        data = self.syringe_pump.sendRcv(cmd_string)
        return int(data["data"])

    def getCurPort(self):
        """ Returns the current port position (1-num_ports) """
        cmd_string = '?20'
        data = self.syringe_pump.sendRcv(cmd_string)
        return data["data"]

    def _checkStatus(self, status_byte):
        """
        Parses a bit string representation of a Tecan API status byte for
        potential error codes (and subsequently raises `SyringeError`) and
        returns the status code as a boolean (True = ready, False = busy).

        Defaults to the error code dictionary (`ERROR_DICT`) defined in the
        `Syringe` class; however, this can be overridden in a subclass.

        """
        status_byte=status_byte["status_byte"]
        error_code = int(status_byte[4:8], 2)
        ready = int(status_byte[2])
        if ready == 1:
            self._ready = True
        else:
            self._ready = False
        error_dict = self.__class__.ERROR_DICT[error_code]
        if error_code != 0:
            raise ConnectionError("{} : {}".format(error_code, error_dict))
        return ready, error_dict

    def terminateCmd(self):
        cmd_string = 'T'
        return self.syringe_pump.sendRcv(cmd_string)    

    def initialize(self, flow_rate=3000):
        """
        initialize syringe pump before we start

        :param flow_rate: syringe pulling flow_rate (dimension : ul/s)

        :return: None
        """
        if self.ini == True:
            cmd_syringe_volume=0
            cmd_port_num=1
            if self.syringe_volume == 5000:
                cmd_syringe_volume=92
            elif self.syringe_volume == 500:
                cmd_syringe_volume=95
            if self.port_num == 9:
                cmd_port_num=8
            self.cmd_string = "ZV{0},1U{1}U{2}R".format(flow_rate, cmd_syringe_volume, cmd_port_num)

            status_dic = self.syringe_pump.sendRcv(self.cmd_string)
            ready, error_msg = self._checkStatus(status_dic)

            msg = "Initializing... Status : {}".format(error_msg)
            self.logger_obj.debug(self.device_name, msg)
            while True:
                status_dic=self.getStatus()
                ready, error_msg = self._checkStatus(status_dic)
                if ready == True:
                    break
                else:
                    time.sleep(5)
            msg = "Initialization completed! : {}".format(status_dic)
            ready, error_msg=self._checkStatus(status_dic)
            msg = "Ready: {}, Status: {}".format(bool(ready), error_msg)
            self.logger_obj.debug(self.device_name, msg)
        else:   
            msg = "Intialization already finished!"
            status_dic = self.syringe_pump.sendRcv("?17R")
            ready, error_msg = self._checkStatus(status_dic)
            self.logger_obj.debug(self.device_name, "msg:{}, data:{}".format(msg, error_msg))

    """
        Fill the volume based on the syringe pump case
    """

    def calibrateVolume(self, volume):
        """
        add solution and inject syringe pump

        :param volume: [ul : 5000, 500]
        :param flow_rate: [ul/s]
        :param mode_type="virtual" (str): set virtual or real mode

        :return: res_msg --> (str)
        """

        # calibrateVolumeValue=volume*1.004+0.0017
        calibrateVolumeValue=volume
        return calibrateVolumeValue

    def add(self, volume, flow_rate, mode_type="virtual"):
        """
        add solution and inject syringe pump

        :param volume (str): [ul : 5000, 500]
        :param flow_rate (str): [ul/s]
        :param mode_type="virtual" (str): set virtual or real mode

        :return: res_msg --> (str)
        """
        int_volume=int(float(volume))
        calibrateVolumeValue = self.calibrateVolume(int_volume)
        flow_rate=int(float(flow_rate))
        temp_device_name=self.device_name+" ({})".format(mode_type)
        flow_rate = self._divideSpeed(flow_rate)
        msg = "Injecting... Solution: {}, Volume:{}, flow_rate:{}".format(self.solution_name,volume,flow_rate)
        self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
        """
        not overflow our basic syringe volume
        """
        if mode_type=="real":
            if calibrateVolumeValue <= self.syringe_volume:
                step = round((calibrateVolumeValue/self.syringe_volume) * self.base_increment)
                self.cmd_string = "gV12000IA{0}V{1},1OA{2}G{3}R".format(step, flow_rate, 0, 1)
                status_dic = self.syringe_pump.sendRcv(self.cmd_string)
                ready, error_msg = self._checkStatus(status_dic)
                msg = "Receive signal! --> Status : {}" .format(error_msg)
                self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)

                while True:
                    status_dic = self.getStatus()
                    ready, error_msg = self._checkStatus(status_dic)
                    if ready == True:
                        msg = "Injection done: {}!".format(self.solution_name)
                        self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                        break
                    else:
                        time.sleep(5)
                status_dic = self.getStatus()
                ready, error_msg = self._checkStatus(status_dic)
                msg = "Pump Ready? Receive signal! --> Ready : {}, Status : {}" .format(bool(ready), error_msg)
                self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                self.cmd_string = ""

            # If value overflow our basic syringe volume, calculate how many cycle i        
            else:
                cycle = int(calibrateVolumeValue/self.syringe_volume)
                last_volume = calibrateVolumeValue - (self.syringe_volume * cycle)
                cycle_step = round((self.syringe_volume / self.syringe_volume) * self.base_increment)
                last_step = round((last_volume / self.syringe_volume) * self.base_increment)
                """
                    g : start cycle
                    I : input direction
                    A : Absolute result of volume
                    O : Output direction
                    A : Absolute result of volume
                    G : How many cycle do we run?
                    R : command's end
                """
                self.cmd_string = "gV12000IA{0}V{1},1OA{2}G{3}V15000IA{4}V{1},1OA{2}R".format(cycle_step, flow_rate, 0, cycle, last_step)
                status_dic = self.syringe_pump.sendRcv(self.cmd_string)
                ready, error_msg = self._checkStatus(status_dic)
                msg = "Receive signal! --> Status : {}" .format(error_msg)
                self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                while True:
                    status_dic = self.getStatus()
                    ready, error_msg = self._checkStatus(status_dic)
                    if ready == True:
                        msg = "Injection done: {}!".format(self.solution_name)
                        self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                        break
                    else:
                        time.sleep(5)
                status_dic = self.getStatus()
                ready, error_msg = self._checkStatus(status_dic)
                msg = "Pump Ready? Receive signal! --> Ready : {}, Status : {}" .format(bool(ready), error_msg)
                self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                self.cmd_string = ""
            msg= "Injection done, Solution: {}, Volume:{}, flow_rate:{}".format(self.solution_name,volume,flow_rate)
            self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
            res_msg=temp_device_name + " : " + msg

            return res_msg

        elif mode_type=="virtual":
            msg = "Injecting... Solution: {}, Volume:{}, flow_rate:{}".format(self.solution_name,volume,flow_rate)
            self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
            msg= "Injection done, Solution: {}, Volume:{}, flow_rate:{}".format(self.solution_name,volume,flow_rate)
            self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
            res_msg=temp_device_name + " : " + msg

            return res_msg

    def moveMaintainencePoint(self,):
        """
        If we start our maintainence process, please run this function and remove our syringe, wash our syringe next.
        """
        self.cmd_string = "A8000R"
        status_dic = self.syringe_pump.sendRcv(self.cmd_string)

        return str(status_dic['status_byte'][2])

    def moveMaintainencePoint(self,):
        """
        If we start our maintainence process, please run this function and remove our syringe, wash our syringe next.
        """
        self.cmd_string = "A8000R"
        status_dic = self.syringe_pump.sendRcv(self.cmd_string)

        return str(status_dic['status_byte'][2])

    def moveInputPoint(self,):
        """ 
        move input point in 3-port valve system.
        If system change to n-port valve, should be make more function.
        """
        self.cmd_string = "V1000IR"
        status_dic = self.syringe_pump.sendRcv(self.cmd_string)

        return str(status_dic['status_byte'][2])

    def test(self, cmd_string):
        status_dic = self.syringe_pump.sendRcv(cmd_string)
        print(self.getCurrentPlunger())
        print(self.getEncoderPos())
        print(self.getPlungerPos())

        return str(status_dic['status_byte'][2])


class XCaliburD(object):
    """
    [XCaliburD] XCaliburD to control Cavro Centris, XCaliburD in tecan cavro

    This class input logger_obj, so that our class use logging class in this pump class.
    if you want to consider the law of logging function, please refer Log/Logging_Class.py

    # Variable
    :param logger_obj (obj): set logging object (from Logging_class import Loggger)
    :param device_name: set pump name ex) XCaliburD (Log name)
    :param tecan_addr: physical address in syringe pump (back of pump, seems like clock) ex) 0,1,2,3...
    :param ser_port: COM8 or COM7 depending on Centris' Multi Hub ex) COM8
    :param baud: port number which has own number ex) 0,1,2....
    :param syringe_volume: glass or plastic syringe's volume (dimension : ul) ex) 500 or 5000 (==5ml)
    :param ini: choose type of action (initialize or not)
    :param max_attempts: maximum attempts is 10. 

    # function
    - initialize(flow_rate=1000)
    - add(volume, speed)
    - getStatus()
    - getConfiguration()
    - updateSpeeds()
    - getPlungerPos()
    - getStartSpeed()
    - getTopSpeed()
    - getCutoffSpeed()
    - getEncoderPos()
    - getCurPort()
    - getBufferStatus()
    - getBackSlashIncrements()
    - getNumberOfInitialization()
    - getNumberOfPlungerMovements()
    - getNumberOfValveMovements()
    - getNumberOfLastValveMovements()
    - setMicrostep
    - terminateCmd()
    - moveMaintainencePoint()
    - moveInputPoint()
    """
    ERROR_DICT = {
        0: "Non Error",
        1: 'Initialization Error',
        2: 'Invalid Command',
        3: 'Invalid Operand',
        4: 'Invalid Command Sequence',
        6: 'EEPROM Failure',
        7: 'Device Not Initialized',
        9: 'Plunger Overload',
        10: 'Valve Overload',
        11: 'Plunger Move Not Allowed',
        15: 'Command Overflow'
    }

    def __init__(self, logger_obj, device_name="XCaliburD", solution_name="", tecan_addr=0, ser_port="COM6", baud=0, syringe_volume=5000, ini=False, max_attempts=10):
        # logger object
        self.logger_obj=logger_obj

        # device debugrmation
        self.device_name_only=device_name
        self.solution_name=solution_name
        self.base_increment=3000
        self.tecan_addr = tecan_addr
        self.ser_port = ser_port
        self.baud = 9600+baud
        self.syringe_volume = syringe_volume
        self.ini = ini
        self.max_attempts = max_attempts
        self.cmd_string = ""
        self.syringe_pump = TecanAPISerial(self.tecan_addr, self.ser_port, self.baud, max_attempts=self.max_attempts)
        self.microstep=False
        self.setMicrostep(self.microstep)
        self.state = {
            'plunger_pos': None,
            'port': None,
            'microstep': self.microstep,
            'start_speed': None,
            'top_speed': None,
            'cutoff_speed': None,
        }
        self.initialize()
        self.updateSpeeds()

    #########################################################################
    # Report commands (cannot be chained)                                   #
    #########################################################################

    def updateSpeeds(self):
        self.getStartSpeed()
        self.getTopSpeed()
        self.getCutoffSpeed()
        self.getPlungerPos()
        self.getCurPort()

    def getStatus(self):
        """
        check syringe pump's status

        status_dict --> 01000000 or 01100000 (work done == not busy) or 0(working == busy)

        :return: status_dict
        """
        cmd_string = "?29"
        status_dict = self.syringe_pump.sendRcv(cmd_string)

        return status_dict

    def getConfiguration(self):
        """
        check syringe pump's configuration

        configuration_dict --> 01000000 or 01100000 (work done == not busy) or 0(working == busy)

        :return: configuration_dict
        """
        cmd_string = "?76"
        configuration_dict = self.syringe_pump.sendRcv(cmd_string)

        return configuration_dict

    def getPlungerPos(self):
        """ Returns the absolute plunger position as an int (0-3000) """
        cmd_string = '?'
        data = self.syringe_pump.sendRcv(cmd_string)
        self.state['plunger_pos'] = int(data["data"])
        return self.state['plunger_pos']

    def getStartSpeed(self):
        """ Returns the start speed as an int (in pulses/sec) """
        cmd_string = '?1'
        data = self.syringe_pump.sendRcv(cmd_string)
        self.state['start_speed'] = int(data["data"])
        return self.state['start_speed']

    def getTopSpeed(self):
        """ Returns the top speed as an int (in pulses/sec) """
        cmd_string = '?2'
        data = self.syringe_pump.sendRcv(cmd_string)
        self.state['top_speed'] = int(data["data"])
        return self.state['top_speed']

    def getCutoffSpeed(self):
        """ Returns the cutoff speed as an int (in pulses/sec) """
        cmd_string = '?3'
        data = self.syringe_pump.sendRcv(cmd_string)
        self.state['cutoff_speed'] = int(data["data"])
        return self.state['cutoff_speed']

    def getEncoderPos(self):
        """ Returns the current encoder count on the plunger axis """
        cmd_string = '?4'
        data = self.syringe_pump.sendRcv(cmd_string)
        return int(data["data"])

    def getCurPort(self):
        """ Returns the current port position (1-num_ports) """
        cmd_string = '?6'
        data = self.syringe_pump.sendRcv(cmd_string)
        return data["data"]

    def getBufferStatus(self):
        """ Returns the current cmd buffer status """
        cmd_string = '?10'
        data = self.syringe_pump.sendRcv(cmd_string)
        return data["data"]

    def getBackSlashIncrements(self):
        """ Returns the current cmd Backslash increments status """
        cmd_string = '?12'
        data = self.syringe_pump.sendRcv(cmd_string)
        return data["data"]

    def getNumberOfInitialization(self):
        """ Returns the Number Of Initialization"""
        cmd_string = '?15'
        data = self.syringe_pump.sendRcv(cmd_string)
        return int(data["data"])

    def getNumberOfPlungerMovements(self):
        """ Returns the Number Of Plunger Movements """
        cmd_string = '?16'
        data = self.syringe_pump.sendRcv(cmd_string)
        return int(data["data"])

    def getNumberOfValveMovements(self):
        """ Returns the Number Of Valve Movements """
        cmd_string = '?17'
        data = self.syringe_pump.sendRcv(cmd_string)
        return int(data["data"])

    def getNumberOfLastValveMovements(self):
        """ Returns the Number Of Valve Movements until Last movements """
        cmd_string = '?18'
        data = self.syringe_pump.sendRcv(cmd_string)
        return int(data["data"])

    #########################################################################
    # Config commands                                                       #
    #########################################################################

    def setMicrostep(self, on=False):
        """ Turns microstep mode on or off """
        cmd_string = 'N{0}'.format(int(on))
        self.microstep = on
        if on == True:
            self.base_increment==24000
        else:
            self.base_increment==3000
        self.syringe_pump.sendRcv(cmd_string)
      
        return self.syringe_pump.sendRcv(cmd_string)

    #########################################################################
    # Control commands                                                      #
    #########################################################################

    def terminateCmd(self):
        cmd_string = 'T'
        return self.syringe_pump.sendRcv(cmd_string)    

    def _ulToSteps(self, volume_ul, microstep=None):
        """
        Converts a volume in microliters (ul) to encoder steps.
        Args:
            `volume_ul` (int) : volume in microliters
        Kwargs:
            `microstep` (bool) : whether to convert to standard steps or
                                 microsteps
        """
        if microstep is None:
            microstep = self.state['microstep']
        if microstep:
            steps = int(volume_ul * (24000 / self.syringe_volume))
        else:
            steps = int(volume_ul * (3000 / self.syringe_volume))
        return steps

    def _checkStatus(self, status_byte):
        """
        Parses a bit string representation of a Tecan API status byte for
        potential error codes (and subsequently raises `SyringeError`) and
        returns the status code as a boolean (True = ready, False = busy).

        Defaults to the error code dictionary (`ERROR_DICT`) defined in the
        `Syringe` class; however, this can be overridden in a subclass.

        """
        status_byte=status_byte["status_byte"]
        error_code = int(status_byte[4:8], 2)
        ready = int(status_byte[2])
        if ready == 1:
            self._ready = True
        else:
            self._ready = False
        error_dict = self.__class__.ERROR_DICT[error_code]
        if error_code != 0:
            raise ConnectionError("{} : {}".format(error_code, error_dict))
        return ready, error_dict

    """
        Fill the volume based on the syringe pump case
    """
    def initialize(self, flow_rate=50):
        """
        initialize syringe pump before we start

        :param flow_rate: syringe pulling flow_rate (dimension : step/s)

        :return: None
        """
        self.device_name = "{} pump {} ({})".format(self.device_name_only, self.tecan_addr, mode_type)
        if self.ini == True:
            self.cmd_string = "ZV{0}BR".format(flow_rate)
            status_dic = self.syringe_pump.sendRcv(self.cmd_string)
            ready, error_msg=self._checkStatus(status_dic)
            msg = "Initializing..."
            self.logger_obj.debug(self.device_name, msg)

            while True:
                status_dic=self.getStatus()
                ready, error_msg=self._checkStatus(status_dic)
                if ready == 1:
                    break
                else:
                    time.sleep(5)
            msg = "Initialization completed! : {}".format(status_dic)
            ready, error_msg=self._checkStatus(status_dic)
            msg = "Ready: {}, Status: {}".format(bool(ready), error_msg)
            self.logger_obj.debug(self.device_name, msg)
        else:   
            msg = "Intialization already finished!"
            data = self.syringe_pump.sendRcv("?17R")
            self.logger_obj.debug(self.device_name, "msg:{}, data:{}".format(msg, data))

    def _divideSpeed(self,initSpeed):
        return int(round(initSpeed/20))

    def add(self, volume, flow_rate, mode_type="virtual"):
        """
        add solution and inject syringe pump

        :param volume: [ul : 5000, 500]
        :param flow_rate: [ul/s]
        :param mode_type="virtual" (str): set virtual or real mode

        :return: res_msg --> (str)
        """
        flow_rate = self._divideSpeed(flow_rate)
        temp_device_name=self.device_name+" ({})".format(mode_type)
        msg = "Injecting... Solution: {}, Volume:{}, Speed:{}".format(self.solution_name,volume,flow_rate)
        self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
        """
        not overflow our basic syringe volume
        """
        if mode_type=="real":
            if volume <= self.syringe_volume:
                step = self._ulToSteps(volume)
                self.cmd_string = "V1000IA{0}V{1}OA{2}R".format(step, flow_rate, 0)
                status_dic = self.syringe_pump.sendRcv(self.cmd_string)
                ready, error_msg=self._checkStatus(status_dic)
                msg = "Receive signal! --> Ready: {}, Status: {}" .format(bool(ready), error_msg)
                self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)

                while True:
                    status_dic = self.getStatus()
                    ready, error_msg=self._checkStatus(status_dic)
                    if ready == True:
                        msg = "Injection done: {}!".format(self.solution_name)
                        self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                        break
                    else:
                        time.sleep(5)
                status_dic = self.getStatus()
                ready, error_msg=self._checkStatus(status_dic)
                msg = "Pump Ready? Receive signal! --> Ready: {}, Status: {}" .format(bool(ready), error_msg)
                self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                self.cmd_string = ""

            # If value overflow our basic syringe volume, calculate how many cycle i        
            else:
                cycle = int(volume/self.syringe_volume)
                last_volume = volume - (self.syringe_volume * cycle)
                cycle_step = self._ulToSteps(self.syringe_volume)
                last_step = self._ulToSteps(last_volume)
                """
                    g : start cycle
                    I : input direction
                    A : Absolute result of volume
                    O : Output direction
                    A : Absolute result of volume
                    G : How many cycle do we run?
                    R : command's end
                """
                self.cmd_string = "gV1000IA{0}V{1}OA{2}G{3}IA{4}OA{2}R".format(flow_rate, cycle_step, 0, cycle, last_step)
                status_dic = self.syringe_pump.sendRcv(self.cmd_string)
                ready, error_msg=self._checkStatus(status_dic)
                msg = "Receive signal! --> Ready: {}, Status: {}" .format(bool(ready), error_msg)
                self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                # time.sleep(45)
                while True:
                    status_dic = self.getStatus()
                    ready, error_msg=self._checkStatus(status_dic)
                    if ready == True:
                        msg = "Injection done: {}!".format(self.solution_name)
                        self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                        break
                    else:
                        time.sleep(5)
                status_dic = self.getStatus()
                msg = "Pump Ready? Receive signal! --> Ready: {}, Status: {}" .format(bool(ready), error_msg)
                self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
                self.cmd_string = ""
            msg= "Injection done, Solution: {}, Volume:{}, flow_rate:{}".format(self.solution_name,volume,flow_rate)
            self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
            res_msg=temp_device_name + " : " + msg
            return res_msg

        elif mode_type=="virtual":
            msg = "Injecting... Solution: {}, Volume:{}, Speed:{}".format(self.solution_name,volume,flow_rate)
            self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
            msg= "Injection done, Solution: {}, Volume:{}, flow_rate:{}".format(self.solution_name,volume,flow_rate)
            self.logger_obj.debug(device_name=temp_device_name, debug_msg=msg)
            res_msg=temp_device_name + " : " + msg

            return res_msg

    def moveMaintainencePoint(self):
        """ 
        If we start our maintainence process, please run this function and remove our syringe, wash our syringe next.
        """
        self.cmd_string = "A3000R"
        status_dic = self.syringe_pump.sendRcv(self.cmd_string)

        return str(status_dic['status_byte'][2])

    def moveInputPoint(self):
        """ 
        move input point in 3-port valve system.
        If system change to n-port valve, should be make more function.
        """
        self.cmd_string = "V1000IR"
        status_dic = self.syringe_pump.sendRcv(self.cmd_string)

        return str(status_dic['status_byte'][2])

    def test(self):
        self.cmd_string = "V1000IA1000V20OA0R"
        status_dic = self.syringe_pump.sendRcv(self.cmd_string)

        return str(status_dic['status_byte'][2])