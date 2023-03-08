from Robot_Arm.Robot_class import *
from Log.Logging_Class import NodeLogger
from BaseUtils.TCP_Node import BaseTCPNode
import socket

SIZE = 104857600

# # TCP/IP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('161.122.22.174', 54010))  # ip주소, 포트번호 지정
server_socket.listen()  # 클라이언트의 연결요청을 기다리는 상태

NodeLogger_obj = NodeLogger(platform_name="Robot Platform", setLevel="DEBUG",
                            SAVE_DIR_PATH="/home/sdl/catkin_ws/src/doosan-robot")
Robot_obj = Robot_Class(NodeLogger)
base_tcp_node_obj = BaseTCPNode()
"""
roslaunch dsr_launcher single_robot_gazebo.launch model:=m0609 host:=192.168.137.100 mode:=real
"""
try:
    while True:
        client_socket, addr = server_socket.accept()  # 연결 요청을 수락함. 그러면 아이피주소, 포트등 데이터를 return
        data = client_socket.recv(SIZE)  # 클라이언트로 부터 데이터를 받음. 출력되는 버퍼 사이즈. (만약 2할 경우, 2개의 데이터만 전송됨)

        packet_info = str(data.decode()).split(sep="/")
        print("packet information list : ", packet_info)

        hardware_name,  pick_num, place_num,action_type, mode_type = packet_info
        if packet_info[0] == "DS_B":
            if packet_info[3]== "stirrer_to_holder":
                res_msg = pick_and_place(NodeLogger_obj,pick_num=int(pick_num),place_num= int(place_num),action_type="stirrer_to_holder",mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type=action_type)
            
            elif packet_info[3]== "storage_empty_to_stirrer":
                res_msg = pick_and_place(NodeLogger_obj, pick_num=int(pick_num), place_num=int(place_num),action_type='storage_empty_to_stirrer',mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type=action_type)

            elif packet_info[3] == "holder_to_storage_filled":       
                res_msg = pick_and_place(NodeLogger_obj,pick_num=int(pick_num),place_num= int(place_num),action_type="holder_to_storage_filled",mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type=action_type)

            elif packet_info[3] == "cuvette_storage_to_cuvette_holder":
                res_msg = pick_and_place(NodeLogger_obj,pick_num=int(pick_num),place_num= int(place_num),action_type="cuvette_storage_to_cuvette_holder",mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type=action_type)
          
            elif packet_info[3] == "cuvette_holder_to_UV":
                res_msg = pick_and_place(NodeLogger_obj,pick_num=int(pick_num),place_num= int(place_num),action_type="cuvette_holder_to_UV",mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type=action_type)
            
            elif packet_info[3] == "UV_to_cuvette_storage":
                res_msg = pick_and_place(NodeLogger_obj,pick_num=int(pick_num),place_num= int(place_num),action_type="UV_to_cuvette_storage",mode_type=mode_type)
                base_tcp_node_obj.checkSocketStatus(client_socket, res_msg, hardware_name, action_type=action_type)
     
        else:
            raise ValueError("[{}] Packet Error : hardware_name is wrong".format(hardware_name))
        
except KeyboardInterrupt:
    print('Ctrl + C 중지 메시지 출력')