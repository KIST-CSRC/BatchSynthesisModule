#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] Robot Arm motion for doosan robot
# @author    Nayeon Kim (kny@kist.re.kr) // Hyuk Jun Yoo (yoohj9475@kist.re.kr)
# @version 1_2
# TEST 2021-09-23
# Test 2022-04-13

# from bitarray import test
# from dxl_def import *
# from dynamixel_sdk import *
import rospy
import os
import threading, time
import sys
# from BaseUtils.Preprocess import PreprocessJSON
# from BaseUtils.TCP_Node import BaseTCPNode

sys.dont_write_bytecode = True
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "common/imp")))  # get import path : DSR_ROBOT.py
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../Robot_Arm")))
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))) 

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *
from robot_teaching import location_dict
# from TCP_Connection.TCP import TCP_Class
# Fundermental componant--------------------
# 
# ---------------------------------------------------------------------------
def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0


def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if 0 == (msgRobotState_cb.count % 100000):
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state           : %d" % msg.robot_state)
        print("  robot_state_str       : %s" % msg.robot_state_str)
        print("  actual_mode           : %d" % msg.actual_mode)
        print("  actual_space          : %d" % msg.actual_space)
        print("  current_posj          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.current_posj[0], msg.current_posj[1], msg.current_posj[2], msg.current_posj[3], msg.current_posj[4],
            msg.current_posj[5]))
        print("  current_velj          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.current_velj[0], msg.current_velj[1], msg.current_velj[2], msg.current_velj[3], msg.current_velj[4],
            msg.current_velj[5]))
        print("  joint_abs             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.joint_abs[0], msg.joint_abs[1], msg.joint_abs[2], msg.joint_abs[3], msg.joint_abs[4], msg.joint_abs[5]))
        print("  joint_err             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.joint_err[0], msg.joint_err[1], msg.joint_err[2], msg.joint_err[3], msg.joint_err[4], msg.joint_err[5]))
        print("  target_posj           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.target_posj[0], msg.target_posj[1], msg.target_posj[2], msg.target_posj[3], msg.target_posj[4],
            msg.target_posj[5]))
        print("  target_velj           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.target_velj[0], msg.target_velj[1], msg.target_velj[2], msg.target_velj[3], msg.target_velj[4],
            msg.target_velj[5]))
        print("  current_posx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.current_posx[0], msg.current_posx[1], msg.current_posx[2], msg.current_posx[3], msg.current_posx[4],
            msg.current_posx[5]))
        print("  current_velx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.current_velx[0], msg.current_velx[1], msg.current_velx[2], msg.current_velx[3], msg.current_velx[4],
            msg.current_velx[5]))
        print("  task_err              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.task_err[0], msg.task_err[1], msg.task_err[2], msg.task_err[3], msg.task_err[4], msg.task_err[5]))
        print("  solution_space        : %d" % msg.solution_space)
        sys.stdout.write("  rotation_matrix       : ")
        for i in range(0, 3):
            sys.stdout.write("dim : [%d]" % i)
            sys.stdout.write("  [ ")
            for j in range(0, 3):
                sys.stdout.write("%d " % msg.rotation_matrix[i].data[j])
            sys.stdout.write("] ")
        print  ##end line
        print("  dynamic_tor           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.dynamic_tor[0], msg.dynamic_tor[1], msg.dynamic_tor[2], msg.dynamic_tor[3], msg.dynamic_tor[4],
            msg.dynamic_tor[5]))
        print("  actual_jts            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.actual_jts[0], msg.actual_jts[1], msg.actual_jts[2], msg.actual_jts[3], msg.actual_jts[4],
            msg.actual_jts[5]))
        print("  actual_ejt            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.actual_ejt[0], msg.actual_ejt[1], msg.actual_ejt[2], msg.actual_ejt[3], msg.actual_ejt[4],
            msg.actual_ejt[5]))
        print("  actual_ett            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.actual_ett[0], msg.actual_ett[1], msg.actual_ett[2], msg.actual_ett[3], msg.actual_ett[4],
            msg.actual_ett[5]))
        print("  sync_time             : %7.3f" % msg.sync_time)
        print("  actual_bk             : %d %d %d %d %d %d" % (
            msg.actual_bk[0], msg.actual_bk[1], msg.actual_bk[2], msg.actual_bk[3], msg.actual_bk[4], msg.actual_bk[5]))
        print("  actual_bt             : %d %d %d %d %d " % (
            msg.actual_bt[0], msg.actual_bt[1], msg.actual_bt[2], msg.actual_bt[3], msg.actual_bt[4]))
        print("  actual_mc             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.actual_mc[0], msg.actual_mc[1], msg.actual_mc[2], msg.actual_mc[3], msg.actual_mc[4], msg.actual_mc[5]))
        print("  actual_mt             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.actual_mt[0], msg.actual_mt[1], msg.actual_mt[2], msg.actual_mt[3], msg.actual_mt[4], msg.actual_mt[5]))

        # print digital i/o
        sys.stdout.write("  ctrlbox_digital_input : ")
        for i in range(0, 16):
            sys.stdout.write("%d " % msg.ctrlbox_digital_input[i])
        print  ##end line
        sys.stdout.write("  ctrlbox_digital_output: ")
        for i in range(0, 16):
            sys.stdout.write("%d " % msg.ctrlbox_digital_output[i])
        print 
        sys.stdout.write("  flange_digital_input  : ")
        for i in range(0, 6):
            sys.stdout.write("%d " % msg.flange_digital_input[i])
        print
        sys.stdout.write("  flange_digital_output : ")
        for i in range(0, 6):
            sys.stdout.write("%d " % msg.flange_digital_output[i])
        print
        # print modbus i/o
        sys.stdout.write("  modbus_state          : ")
        if len(msg.modbus_state) > 0:
            for i in range(0, len(msg.modbus_state)):
                sys.stdout.write("[" + msg.modbus_state[i].modbus_symbol)
                sys.stdout.write(", %d] " % msg.modbus_state[i].modbus_value)
        print

        print("  access_control        : %d" % msg.access_control)
        print("  homming_completed     : %d" % msg.homming_completed)
        print("  tp_initialized        : %d" % msg.tp_initialized)
        print("  mastering_need        : %d" % msg.mastering_need)
        print("  drl_stopped           : %d" % msg.drl_stopped)
        print("  disconnected          : %d" % msg.disconnected)

def Robot_initialize():
    rospy.init_node('single_robot_simple_py', log_level=rospy.ERROR)
    rospy.on_shutdown(shutdown)
    global set_robot_mode
    set_robot_mode = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/system/set_robot_mode', SetRobotMode)
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True
    t1.start()

    global pub_stop
    pub_stop = rospy.Publisher('/' + ROBOT_ID + ROBOT_MODEL + '/stop', RobotStop, queue_size=10)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # global parameter
    # set_velx(2500, 800.63)  # set global task speed: 30(mm/sec), 20(deg/sec)
    # set_accx(100, 300)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    # openGripper()
    # closeGripper(action_type="Vial")

msgRobotState_cb.count = 0

def get_status():
    robot_position = get_current_posx()
    return robot_position

def thread_subscriber():
    rospy.Subscriber('/' + ROBOT_ID + ROBOT_MODEL + '/state', RobotState, msgRobotState_cb)
    rospy.spin()
    # rospy.spinner(2)


def openGripper(action_type="Vial"):
    """
    ONROBOT_TARGET_FORCE_1: 0
    ONROBOT_TARGET_WIDTH_1: 0 (600 == 60mm)
    ONROBOT_CONTROL_1: 1 (1 == action)
    ONROBOT_ACTUAL_DEPTH_1: 211
    ONROBOT_ACTUAL_REL_DEPTH_1: 211
    ONROBOT_ACTUAL_WIDTH_1: 918
    ONROBOT_STATUS_1: 0
    """
    if action_type == "Vial":
        set_modbus_output("ONROBOT_TARGET_WIDTH_1", 500) 
        set_modbus_output("ONROBOT_TARGET_FORCE_1", 400)
        set_modbus_output("ONROBOT_CONTROL_1", 1)

    elif action_type == "Cuvette":
        set_modbus_output("ONROBOT_TARGET_WIDTH_1", 400)
        set_modbus_output("ONROBOT_TARGET_FORCE_1", 200)
        set_modbus_output("ONROBOT_CONTROL_1", 1)


def closeGripper(action_type="Vial"):
    if action_type == "Vial":
        set_modbus_output("ONROBOT_TARGET_WIDTH_1", 350)
        set_modbus_output("ONROBOT_TARGET_FORCE_1", 400)
        set_modbus_output("ONROBOT_CONTROL_1", 1)

    elif action_type == "Cuvette":
        set_modbus_output("ONROBOT_TARGET_WIDTH_1", 200)
        set_modbus_output("ONROBOT_TARGET_FORCE_1", 200)
        set_modbus_output("ONROBOT_CONTROL_1", 1)

def pick(approach, leave, velocity, acceleration, object_type):
    '''
    pick (bird coordination(before pick) -> approach -> close gripper -> leave -> bird coordination)
    
    :param approach (int): approch distance 
    :param leave (int): leave distance  
    :param object_type (str): object type for gripper distance 
        ex) Vial, Cuvette
    '''
    movel(approach, vel = velocity, acc = acceleration, ref = DR_TOOL)
    closeGripper(action_type = object_type)
    movel(leave, vel = velocity, acc = acceleration, ref = DR_TOOL)

def place(approach, leave, velocity, acceleration, object_type):
    '''
    place (bird coordination(before pick) -> approach -> open gripper -> leave -> bird coordination)
    
    :param approach (int): approch distance 
    :param leave (int): leave distance  
    :param object_type (str): object type for gripper distance 
        ex) Vial, Cuvette
    '''
    movel(approach, vel = velocity, acc = acceleration, ref = DR_TOOL)
    openGripper(action_type = object_type)
    movel(leave, vel = 500, acc = 50, ref = DR_TOOL)

def move_HOME(temp_info_dict):
    temp_info_dict=location_dict["move_HOME"]
    movel(temp_info_dict["location_list"][0], vel=temp_info_dict['vel']['general'], acc=temp_info_dict["acc"]["general"])
    
def move_before_pick_point(temp_location_dict):
    '''
    move before pick point

    :temp_location_dict (list): sequence of each action
        ex) storage_empty_to_stirrer_list = [[pos_XYZ, pos_storage, storage_empty_center], 
                            storage_empty_pick_1,
                            storage_empty_pick_2, 
                            [pos_storage, pos_XYZ, stirrer_center],
                            stirrer_place_1,
                            stirrer_place_2, 
                            [stirrer_center, pos_XYZ]
                            ]
        -> ONLY USE [pos_XYZ, pos_storage, storage_empty_center] part.
    '''
    for i in range(len(temp_location_dict["location_list"][0])):
        movel(temp_location_dict["location_list"][0][i], vel = temp_location_dict["vel"]["general"], acc = temp_location_dict["acc"]["general"])
    
def move_place_point_during_picking_vial(temp_location_dict):
    '''
    move place point during picking vial 

    :temp_location_dict (list): sequence of each action
        ex) storage_empty_to_stirrer_list = [[pos_XYZ, pos_storage, storage_empty_center], 
                            storage_empty_pick_1,
                            storage_empty_pick_2, 
                            [pos_storage, pos_XYZ, stirrer_center],
                            stirrer_place_1,
                            stirrer_place_2, 
                            [stirrer_center, pos_XYZ]
                            ]
        -> ONLY USE [pos_storage, pos_XYZ, stirrer_center] part.
    '''
    for i in range(len(temp_location_dict["location_list"][3])):
        movel(temp_location_dict["location_list"][3][i], vel=temp_location_dict['vel']['general'], acc = temp_location_dict["acc"]['general'])

def move_ending_point(temp_location_dict):
    '''
    move ending point to finish action 

    :temp_location_dict (list): sequence of each action
        ex) storage_empty_to_stirrer_list = [[pos_XYZ, pos_storage, storage_empty_center], 
                            storage_empty_pick_1,
                            storage_empty_pick_2, 
                            [pos_storage, pos_XYZ, stirrer_center],
                            stirrer_place_1,
                            stirrer_place_2, 
                            [stirrer_center, pos_XYZ]
                            ]
        -> ONLY USE [stirrer_center, pos_XYZ] part.
    '''
    for i in range(len(temp_location_dict["location_list"][6])):
        movel(temp_location_dict["location_list"][6][i], vel=temp_location_dict['vel']['general'], acc = temp_location_dict["acc"]['general'])

def pick_and_place(NodeLogger_obj, pick_num, place_num, action_type='storage_empty_to_stirrer', mode_type="virtual"):
    """
        Pick and place function for moving object with DOOSAN robot

        :param NodeLogger_obj (NodeLogger): NodeLogger object to control log message
        :param pick_num (int) : pick number of each hardware location (0-7) 
        :param place_num (int) : place number of each hardware loaction (0-7)
        :param action_type (str) : Key of input_location_dict. 
            ex) storage_empty_to_stirrer
                stirrer_to_holder
                holder_to_storage_filled
                cuvette_storage_to_cuvette_holder
                cuvette_holder_to_UV
                UV_to_cuvette_storage
        :return: None
        
        temp_info_dict = 
        {
            "location_list" : Storage_empty_to_Stirrer_list,
            "pick_loc_list" : storage_empty_list,
            "place_loc_list" : stirrer_list,
            "object_type" : 'Vial',
            "ref" : "DR_BASE",
            "vel":{
                "linear":250,
                "radious":80
            },
            "acc":{
                "linear":1,
                "radious":332
            }
        }
    """
    temp_info_dict=location_dict[action_type]
    device_name = "{} ({})".format("Doosan Arm m0609",mode_type)
    # change later
    if mode_type=="real":
        openGripper(action_type = temp_info_dict['object_type'])
    msg = "Pick and place : Gripper is opened."
    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)

    if mode_type=="real":
        move_before_pick_point(temp_info_dict) # move before pick point
    msg = "Pick and place : Move before pick point."
    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)

    if mode_type=="real":
        movel(temp_info_dict["pick_loc_list"][pick_num], vel=temp_info_dict['vel']['general'], acc = temp_info_dict["acc"]['general'])
    msg = "Pick and place : Move pick point."
    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)

    if mode_type=="real":
        pick(temp_info_dict["location_list"][1], temp_info_dict["location_list"][2], velocity=temp_info_dict['vel']['pick'], acceleration=temp_info_dict["acc"]['pick'], object_type=temp_info_dict["object_type"]) # go below -> pick -> go up
    msg = "Pick and place : Pick."
    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)

    if mode_type=="real":
        move_place_point_during_picking_vial(temp_info_dict) # move place point during picking vial
    msg = "Pick and place : Move before place point during picking vial."
    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)

    if mode_type=="real":
        movel(temp_info_dict["place_loc_list"][place_num], vel=temp_info_dict['vel']['general'], acc = temp_info_dict["acc"]['general'])
    msg = "Pick and place : Move place point during picking vial."
    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)

    if mode_type=="real":
        place(temp_info_dict["location_list"][4], temp_info_dict["location_list"][5],velocity=temp_info_dict['vel']['place'],acceleration=temp_info_dict['acc']['place'], object_type=temp_info_dict["object_type"]) # go below -> place -> go up
    msg = "Pick and place : Place."
    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)

    if mode_type=="real":
        move_ending_point(temp_info_dict) # move ending point
    msg = "Pick and place : Move home"
    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)

    msg = "{0} : {1}_{2}  -> {3}_{4}".format(action_type, temp_info_dict['msg']['from'], pick_num, temp_info_dict['msg']['to'], place_num)
    res_msg = "Success"

    NodeLogger_obj.debug(device_name=device_name, debug_msg=msg)
    

    return res_msg


 
# real motion -------------------------------------------------------------------------------------------------------------------------------------



def move_Storage_empty_to_Stirrer(NodeLogger_obj, cycle_num, place_num, mode_type="virtual"):
    '''
    move (storage -> Strrier) with DOOSAN robot

    :param NodeLogger_obj (NodeLogger): NodeLogger object to control log message
    :param cycle_num (int): cycle number of batch system(chemical storage, stirrer, cuvette storage)
    :param place_num (int): place number of each vessel loaction (0-7)
    :param mode_type (str): "virtual" (str): set mode type --> real, virtual
    '''

    pick_and_place(NodeLogger_obj, (cycle_num)//2, place_num, action_type='storage_empty_to_stirrer', mode_type=mode_type)

def move_Stirrer_to_Holder(NodeLogger_obj, place_num, mode_type="virtual"):
    '''
    move (Stirrer -> Holder) with DOOSAN robot

    :param NodeLogger_obj (NodeLogger): NodeLogger object to control log message
    :param place_num (int): place number of each vessel loaction (0-7)
    :param mode_type (str): "virtual" (str): set mode type --> real, virtual
    '''
    pick_and_place(NodeLogger_obj, place_num, place_num, action_type='stirrer_to_holder', mode_type=mode_type)

def move_Holder_to_Storage_filled(NodeLogger_obj, cycle_num, pick_num, mode_type="virtual"):
    '''
    move (Holder -> Stroage filled) with DOOSAN robot

    :param NodeLogger_obj (NodeLogger): NodeLogger object to control log message
    :param cycle_num (int): cycle number of batch system(chemical storage, stirrer, cuvette storage)
    :param pick_num_num (int): pick number of each vessel loaction (0-7)
    :param mode_type (str): "virtual" (str): set mode type --> real, virtual
    '''
    pick_and_place(NodeLogger_obj, pick_num, (cycle_num)//2, action_type='holder_to_storage_filled', mode_type=mode_type)
    
def move_Cuvette_storage_to_Cuvette_holder(NodeLogger_obj, cycle_num, pick_num, mode_type="virtual"):
    """
    This function has 8 action which move 8 cuvette to each cuvette holder.

    :param NodeLogger_obj (NodeLogger): NodeLogger object to control log message
    :param cycle_num (int): cycle number of batch system(chemical storage, stirrer, cuvette storage)
    :param pick_num_num (int): pick number of each vessel loaction (0-7)   
    :param mode_type (str): "virtual" (str): set mode type --> real, virtual
    """
    pick_and_place(NodeLogger_obj, (cycle_num)*8+pick_num, pick_num, action_type='cuvette_storage_to_cuvette_holder',mode_type=mode_type)
    
def move_Cuvette_holder_to_UV(NodeLogger_obj, pick_num, mode_type="virtual"):
    """
    This function has 1 action which move 1 cuvette to each UV holder.

    :param NodeLogger_obj (NodeLogger): NodeLogger object to control log message
    :param pick_num_num (int): pick number of each vessel loaction (0-7)   
    :param mode_type (str): "virtual" (str): set mode type --> real, virtual
    """
    pick_and_place(NodeLogger_obj, pick_num, 0, action_type='cuvette_holder_to_UV',mode_type=mode_type)
   
def move_UV_to_Cuvette_storage(NodeLogger_obj, cycle_num, place_num, mode_type="virtual"):
    """
    This function has 1 action which move 1 cuvette to each cuvette storage.

    :param NodeLogger_obj (NodeLogger): NodeLogger object to control log message
    :param cycle_num (int): cycle number of batch system(chemical storage, stirrer, cuvette storage)
    :param place_num_num (int): place number of each vessel loaction (0-7)   
    :param mode_type (str): "virtual" (str): set mode type --> real, virtual
    """
    pick_and_place(NodeLogger_obj, 0, (cycle_num)*8+place_num, action_type='UV_to_cuvette_storage',mode_type=mode_type)

def TEST_move_one_cycle(NodeLogger_obj,cycle_num, mode_type="virtual"):
    # tcp_obj=TCP_Class()

    # msg = 'Testing whole action (DOOSAN robot + vial stroage) is started : cycle number is {}.'.format(cycle_num)
    # NodeLogger_obj.info(part_name='SelfdrivingLab', info_msg="Start Robot Queue : "+msg)

    # command_bytes=str.encode("{}/{}/{}".format("STORAGE",(cycle_num)//2+1, mode_type))
    # res_msg=tcp_obj.callServer_STORAGE(command_byte=command_bytes)
    time.sleep(2)
    for action_idx in range(8,16):
        move_Storage_empty_to_Stirrer(NodeLogger_obj, cycle_num=cycle_num, place_num=action_idx, mode_type=mode_type)
    # for action_idx in range(8):
    #     move_Stirrer_to_Holder(NodeLogger_obj, cycle_num=cycle_num, place_num=action_idx, mode_type=mode_type)


    # for action_idx in range(8):
    #     move_Cuvette_storage_to_Cuvette_holder(NodeLogger_obj, cycle_num=cycle_num, pick_num=action_idx, mode_type=mode_type)
    # for each_vial_loc in range(8):    
    #     move_Cuvette_holder_to_UV(NodeLogger_obj, pick_num=4, mode_type=mode_type)
    #     move_UV_to_Cuvette_storage(NodeLogger_obj, cycle_num=cycle_num, place_num=each_vial_loc, mode_type=mode_type)

    # time.sleep(2)
    # command_bytes=str.encode("{}/{}/{}".format("STORAGE",(cycle_num)//2+6,mode_type))
    # res_msg=tcp_obj.callServer_STORAGE(command_byte=command_bytes)  
    # for action_idx in range(8):
    #     move_Holder_to_Storage_filled(NodeLogger_obj, cycle_num=cycle_num, pick_num=action_idx, mode_type=mode_type)

def test_cuvette_action(start_location,final_location,repeat_num):
    for k in range(start_location,final_location):
        for j in range(repeat_num):
            print("cycle : {}".format(j))
            for i in range(0,8):
                time.sleep(1)
                print('cuvette_storage : {}, cuvette_holder : {}'.format(i+8*k, i))
                pick_and_place(NodeLogger, i+8*k, i, action_type='cuvette_storage_to_cuvette_holder',mode_type="real")
            for i in range(0,8):
                time.sleep(1)
                print('cuvette_holder : {}, UV : {}'.format(i, 0))
                pick_and_place(NodeLogger, i, 0, action_type='cuvette_holder_to_UV',mode_type="real")
                time.sleep(1)
                print('UV : {}, cuvette_storage : {}'.format(0, i+8*k))
                pick_and_place(NodeLogger, 0, i+8*k, action_type='UV_to_cuvette_storage',mode_type="real")

def test_storage_to_stirrer_to_storageFilled():
    from BaseUtils.TCP_Node import BaseTCPNode
    tcp_object=BaseTCPNode()
    for i in range(0,16):
        tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/1/real')
        pick_and_place(NodeLogger, 0, i, action_type='storage_empty_to_stirrer',mode_type="real")
    for i in range(0,16):
        if i==0 or i==8:
            tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/6/real')
        pick_and_place(NodeLogger, i, 0, action_type='stirrer_to_storage_filled',mode_type="real")

    for i in range(0,16):
        tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/2/real')
        pick_and_place(NodeLogger, 1, i, action_type='storage_empty_to_stirrer',mode_type="real")
    for i in range(0,16):
        if i==0 or i==8:
            tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/7/real')
        pick_and_place(NodeLogger, i, 1, action_type='stirrer_to_storage_filled',mode_type="real")

    for i in range(0,16):
        tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/3/real')
        pick_and_place(NodeLogger, 2, i, action_type='storage_empty_to_stirrer',mode_type="real")
    for i in range(0,16):
        if i==0 or i==8:
            tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/8/real')
        pick_and_place(NodeLogger, i, 2, action_type='stirrer_to_storage_filled',mode_type="real")

    for i in range(0,16):
        tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/4/real')
        pick_and_place(NodeLogger, 3, i, action_type='storage_empty_to_stirrer',mode_type="real")
    for i in range(0,16):
        if i==0 or i==8:
            tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/9/real')
        pick_and_place(NodeLogger, i, 3, action_type='stirrer_to_storage_filled',mode_type="real")

    for i in range(0,16):
        tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/5/real')
        pick_and_place(NodeLogger, 4, i, action_type='storage_empty_to_stirrer',mode_type="real")
    for i in range(0,16):
        if i==0 or i==8:
            tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/10/real')
        pick_and_place(NodeLogger, i, 4, action_type='stirrer_to_storage_filled',mode_type="real")
   
def test_vialStorage_entrance(): 
    from BaseUtils.TCP_Node import BaseTCPNode
    tcp_object=BaseTCPNode()
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/1/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/2/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/3/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/4/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/5/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/6/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/7/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/8/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/9/real')
    time.sleep(1)
    tcp_object.callServer(host="161.122.22.146", port=54009, command_byte=b'STORAGE/10/real')
    time.sleep(1)

if __name__ == "__main__":
    # robot center setting-----------------------
    openGripper(action_type = 'Vial')
    pos_XYZ = posx(8.240, 258.440, 353.510, 90, -180, -90)
    pos_storage = posx(-277.030, 115.450, 487.380, 0, -180, -270)
    pos_anaylsis = posx(257.620, 15.140, 359.070, 180, -180, 90)
    
    meta = {
            "metadata":
                {
                    "subject":"Find lambda_max",
                    "researcherGroup":"KIST_CSRC",
                    "researcherName":"HJ",
                    "researcherId":"yoohj9475@kist.re.kr",
                    "researcherPwd":"1234",
                    "element":"Ag",
                    "experimentType":"robot",
                    "logLevel":"INFO",
                    "modeType":"real",
                    "saveDirPath":"/home/sdl-pc/catkin_ws/src/doosan-robot",
                    "totalIterNum":4,
                    "batchSize":8
                }
            }

    from Log.Logging_Class import NodeLogger
    NodeLogger_obj=NodeLogger(platform_name="Robot Arm Server", setLevel="INFO", SAVE_DIR_PATH="Log")

    Robot_initialize()
    openGripper()
    # move_Holder_to_Storage_filled(NodeLogger, cycle_num=0, pick_num=1, mode_type="real")
    for cycle_num in range(1):
        TEST_move_one_cycle(NodeLogger_obj, cycle_num, "real")
    # test_vialStorage_entrance()

# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

# # Open port
# if portHandler.openPort():
#     print("Succeeded to open the port")
# else:
#     print("Failed to open the port")
#     print("Press any key to terminate...")
#     getch()
#     quit()

# # Set port baudrate
# if portHandler.setBaudRate(BAUDRATE):
#     print("Succeeded to change the baudrate")
# else:
#     print("Failed to change the baudrate")
#     print("Press any key to terminate...")
#     getch()
#     quit()

# # Enable Dynamixel#1 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))
# else:
#     print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# # Enable Dynamixel#2 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))
# else:
#     print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# # ppt_test()

# # Disable Dynamixel#1 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE,
#                                                           TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Disable Dynamixel#2 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE,
#                                                           TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))


# # Close port
# portHandler.closePort()