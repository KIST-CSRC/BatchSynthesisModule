#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] Robot Arm motion for doosan robot
# @author   Nayeon Kim (kny@kist.re.kr)
# TEST 2021-10-12

import rospy
import os
import threading, time
import sys

sys.dont_write_bytecode = True
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "common/imp")))  # get import path : DSR_ROBOT.py
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../Hardware/Robot_Arm"))) 
# for single robot
from DR_common import *
from DSR_ROBOT import *
import DR_init

# from cuvette_information import cuvette_storage_information

# center point of each process
pos_anaylsis = posx(257.620, 15.140, 359.070, 180, -180, 90)
pos_XYZ = posx(8.240, 258.440, 353.510, 90, -180, -90)
pos_storage = posx(-277.030, 115.450, 487.380, 0, -180, -270)
pos_HOME = posx(0, 0, 0, 90, -180, -90) # modified later

# pick and place z point (distance)
storage_empty_pick_1 = posx(0, 0, 84.5, 0, 0, 0)
storage_empty_pick_2 = posx(0, 0, -85.5, 0, 0, 0)
stirrer_place_1 = posx(0, 0, 71, 0, 0, 0)
stirrer_place_2 = posx(0, 0, -75, 0, 0, 0)
stirrer_pick_1 = posx(0, 0, 75, 0, 0, 0)
stirrer_pick_2 = posx(0, 0, -75, 0, 0, 0)
holder_place_1 = posx(0, 0, 75, 0, 0, 0)
holder_place_2 = posx(0, 0, -75, 0, 0, 0)
holder_pick_1 = posx(0, 0, 75, 0, 0, 0)
holder_pick_2 = posx(0, 0, -75, 0, 0, 0)
storage_filled_place_1 = posx(0, 0, 24, 0, 0)
storage_filled_place_2 = posx(0, 0, -45, 0, 0, 0)
cuvette_storage_pick_1 = posx(0, 0, 66, 0, 0, 0)
cuvette_storage_pick_2 = posx(0, 0, -66, 0, 0, 0)
cuvette_holder_place_1 = posx(0, 0, 64, 0, 0, 0)
cuvette_holder_place_2 = posx(0, 0, -66, 0, 0, 0)
cuvette_holder_pick_1 = posx(0, 0, 66, 0, 0, 0)
cuvette_holder_pick_2 = posx(0, 0, -66, 0, 0, 0)
UV_place_1 = posx(0, 0, 99, 0, 0, 0)
UV_place_2 = posx(0, 0, -100, 0, 0, 0)
UV_pick_1 = posx(0, 0, 100, 0, 0, 0)
UV_pick_2 = posx(0, 0, -100, 0, 0, 0)
cuvette_storage_place_1 = posx(0, 0, 62, 0, 0, 0)
cuvette_storage_place_2 = posx(0, 0, -66, 0, 0, 0)

# storage empty point
storage_empty_center = posx(-362.000, 128.000, 269.740, 0, -167, 90)
storage_empty_1 = posx(-352.430, 9.000, 238.740, 0, -166, 90)
storage_empty_2 = posx(-351.230, 67.000, 238.740, 0, -166, 90)
storage_empty_3 = posx(-350.000, 128.000, 238.740, 0, -166, 90)
storage_empty_4 = posx(-349.000, 189.000, 238.740, 0, -166, 90)
storage_empty_5 = posx(-348.000, 246.000, 238.740, 0, -166, 90)

# holder point
holder_center = posx(505.050, 52.060, 300.000, 180, -180, 90)
holder_1 = posx(720.700, 74.500, 230.000, 0, -180, -90)
holder_2 = posx(718.500, 19.000, 230.000, 0, -180, -90)
holder_3 = posx(665.000, 76.500, 230.000, 0, -180, -90) 
holder_4 = posx(662.000, 19.000, 230.000, 0, -180, -90) 
holder_5 = posx(606.000, 78.000, 230.000, 0, -180, -90)
holder_6 = posx(606.000, 21.000, 230.000, 0, -180, -90)
holder_7 = posx(719.000, -38.000, 230.000, 0, -180, -90)
holder_8 = posx(660.000, -38.000, 230.000, 0, -180, -90)

# Stirrer
stirrer_center = posx(-245.510, 460.000, 316.000, 0, -180, 135) 

stirrer_1 = posx(-249.000, 628.500, 315.000, 0, -180, 135)
stirrer_2 = posx(-214.500, 628.500, 315.000, 0, -180, 135)
stirrer_3 = posx(-168.000, 628.000, 315.000, 0, -180, 135)
stirrer_4 = posx(-133.000, 628.000, 315.000, 0, -180, 135)

stirrer_5 = posx(-249.000, 594.000, 315.000, 0, -180, 135)
stirrer_6 = posx(-214.500, 594.000, 315.000, 0, -180, 135)
stirrer_7 = posx(-168.000, 593.500, 315.000, 0, -180, 135) 
stirrer_8 = posx(-133.000, 593.500, 315.000, 0, -180, 135) 

stirrer_9 = posx(-249.500, 548.000, 315.000, 0, -180, 135) 
stirrer_10 = posx(-215.000, 548.000, 315.000, 0, -180, 135) 
stirrer_11 = posx(-169.000, 547.500, 315.000, 0, -180, 135) 
stirrer_12 = posx(-134.000, 547.500, 315.000, 0, -180, 135) 

stirrer_13 = posx(-249.000, 513.000, 315.000, 0, -180, 135)
stirrer_14 = posx(-215.000, 513.000, 315.000, 0, -180, 135)
stirrer_15 = posx(-169.000, 512.500, 315.000, 0, -180, 135)
stirrer_16 = posx(-134.000, 512.500, 315.000, 0, -180, 135)

# 2022-1019 이전
# stirrer_1 = posx(-251.000, 628.500, 315.000, 0, -180, 135)
# stirrer_2 = posx(-216.500, 628.500, 315.000, 0, -180, 135)
# stirrer_3 = posx(-171.000, 628.500, 315.000, 0, -180, 135)
# stirrer_4 = posx(-135.500, 628.500, 315.000, 0, -180, 135)

# stirrer_5 = posx(-251.000, 593.500, 315.000, 0, -180, 135)
# stirrer_6 = posx(-216.500, 593.500, 315.000, 0, -180, 135)
# stirrer_7 = posx(-171.000, 593.500, 315.000, 0, -180, 135) 
# stirrer_8 = posx(-135.500, 593.500, 315.000, 0, -180, 135) 

# stirrer_9 = posx(-251.000, 547.500, 315.000, 0, -180, 135) 
# stirrer_10 = posx(-216.500, 547.500, 315.000, 0, -180, 135) 
# stirrer_11 = posx(-171.000, 547.500, 315.000, 0, -180, 135) 
# stirrer_12 = posx(-135.500, 547.500, 315.000, 0, -180, 135) 

# stirrer_13 = posx(-251.000, 512.500, 315.000, 0, -180, 135)
# stirrer_14 = posx(-216.500, 512.500, 315.000, 0, -180, 135)
# stirrer_15 = posx(-171.000, 512.500, 315.000, 0, -180, 135)
# stirrer_16 = posx(-135.500, 512.500, 315.000, 0, -180, 135)
# storage filled point
storage_filled_center = posx(-540.500, 133.940, 495.000, 0, 168, -270)
storage_filled_1 = posx(-574.000, 8.000 ,437.470, 0 ,168, -270)
storage_filled_2 = posx(-571.500, 67.990 ,439.470, 180, -168, -90)
storage_filled_3 = posx(-569.000, 127.990, 437.460, 180, -168, -90)
storage_filled_4 = posx(-568.000, 187.980, 437.400, 180, -168, -90)
storage_filled_5 = posx(-567.000, 248.710, 437.410, 180, -168, -90)

# cuvette storage point
cuvette_storage_center = posx(489.000, -205.500, 260.000, 0, -180, -90)

cuvette_storage_1 = posx(638.000, -294.500, 220.000, 0, -180, -90)
cuvette_storage_2 = posx(618.500, -294.500, 220.000, 0, -180, -90)
cuvette_storage_3 = posx(599.000, -293.700, 220.000, 0, -180, -90)
cuvette_storage_4 = posx(579.100, -293.400, 220.000, 0, -180, -90)
cuvette_storage_5 = posx(558.800, -293.200, 220.000, 0, -180, -90)
cuvette_storage_6 = posx(538.600, -292.700, 220.000, 0, -180, -90)
cuvette_storage_7 = posx(518.600, -292.600, 220.000, 0, -180, -90)
cuvette_storage_8 = posx(498.200, -291.700, 220.000, 0, -180, -90)
# cuvette_storage_8 = posx(498.800, -291.700, 220.000, 0, -180, -90)

cuvette_storage_9 = posx(478.000, -291.700, 220.000, 0, -180, -90)
cuvette_storage_10 = posx(457.900, -291.400, 220.000, 0, -180, -90)
cuvette_storage_11 = posx(438.000, -291.100, 220.000, 0, -180, -90)
cuvette_storage_12 = posx(417.900, -291.150, 220.000, 0, -180, -90)
cuvette_storage_13 = posx(397.500, -290.700, 220.000, 0, -180, -90)
cuvette_storage_14 = posx(377.700, -290.700, 220.000, 0, -180, -90)
cuvette_storage_15 = posx(357.700, -290.500, 220.000, 0, -180, -90)
cuvette_storage_16 = posx(337.600, -290.500, 220.000, 0, -180, -90)

cuvette_storage_17 = posx(638.700, -250.700, 220.000, 0, -180, -90)
cuvette_storage_18 = posx(618.800, -250.700, 220.000, 0, -180, -90)
cuvette_storage_19 = posx(599.000, -250.000, 220.000, 0, -180, -90)
cuvette_storage_20 = posx(579.200, -249.800, 220.000, 0, -180, -90)
cuvette_storage_21 = posx(559.200, -249.500, 220.000, 0, -180, -90)
cuvette_storage_22 = posx(539.200, -248.900, 220.000, 0, -180, -90)
cuvette_storage_23 = posx(519.000, -248.900, 220.000, 0, -180, -90)
cuvette_storage_24 = posx(498.800, -248.600, 220.000, 0, -180, -90)

cuvette_storage_25 = posx(479.200, -248.600, 220.000, 0, -180, -90)
cuvette_storage_26 = posx(459.200, -248.100, 220.000, 0, -180, -90)
cuvette_storage_27 = posx(439.900, -247.900, 220.000, 0, -180, -90)
cuvette_storage_28 = posx(418.700, -247.700, 220.000, 0, -180, -90)
cuvette_storage_29 = posx(398.500, -247.000, 220.000, 0, -180, -90)
cuvette_storage_30 = posx(378.100, -247.000, 220.000, 0, -180, -90)
cuvette_storage_31 = posx(358.300, -246.900, 220.000, 0, -180, -90)
cuvette_storage_32 = posx(337.900, -246.900, 220.000, 0, -180, -90)

cuvette_storage_33 = posx(638.800, -207.500, 220.000, 0, -180, -90)
cuvette_storage_34 = posx(619.200, -207.100, 220.000, 0, -180, -90)
cuvette_storage_35 = posx(599.300, -207.000, 220.000, 0, -180, -90)
cuvette_storage_36 = posx(579.350, -206.600, 220.000, 0, -180, -90)
cuvette_storage_37 = posx(559.450, -206.300, 220.000, 0, -180, -90)
cuvette_storage_38 = posx(539.600, -206.300, 220.000, 0, -180, -90)
cuvette_storage_39 = posx(519.600, -206.000, 220.000, 0, -180, -90)
cuvette_storage_40 = posx(499.700, -205.800, 220.000, 0, -180, -90)

cuvette_storage_41 = posx(479.400, -205.400, 220.000, 0, -180, -90)
cuvette_storage_42 = posx(459.400, -205.200, 220.000, 0, -180, -90)
cuvette_storage_43 = posx(439.300, -205.200, 220.000, 0, -180, -90)
cuvette_storage_44 = posx(419.500, -204.900, 220.000, 0, -180, -90)
cuvette_storage_45 = posx(399.500, -204.900, 220.000, 0, -180, -90)
cuvette_storage_46 = posx(379.500, -204.600, 220.000, 0, -180, -90)
cuvette_storage_47 = posx(359.300, -204.400, 220.000, 0, -180, -90)
cuvette_storage_48 = posx(339.300, -204.100, 220.000, 0, -180, -90)

cuvette_storage_49 = posx(639.800, -164.400, 220.000, 0, -180, -90) 
cuvette_storage_50 = posx(620.100, -163.900, 220.000, 0, -180, -90) 
cuvette_storage_51 = posx(599.700, -163.400, 220.000, 0, -180, -90) 
cuvette_storage_52 = posx(580.200, -163.300, 220.000, 0, -180, -90) 
cuvette_storage_53 = posx(559.900, -162.950, 220.000, 0, -180, -90) 
cuvette_storage_54 = posx(539.800, -162.700, 220.000, 0, -180, -90) 
cuvette_storage_55 = posx(519.900, -162.500, 220.000, 0, -180, -90) 
cuvette_storage_56 = posx(500.000, -162.200, 220.000, 0, -180, -90) 

cuvette_storage_57 = posx(480.000, -161.950, 220.000, 0, -180, -90)
cuvette_storage_58 = posx(460.000, -161.550, 220.000, 0, -180, -90)
cuvette_storage_59 = posx(439.900, -161.500, 220.000, 0, -180, -90)
cuvette_storage_60 = posx(419.900, -161.350, 220.000, 0, -180, -90)
cuvette_storage_61 = posx(399.800, -161.200, 220.000, 0, -180, -90)
cuvette_storage_62 = posx(379.700, -160.900, 220.000, 0, -180, -90)
cuvette_storage_63 = posx(359.400, -160.700, 220.000, 0, -180, -90)
cuvette_storage_64 = posx(339.300, -160.600, 220.000, 0, -180, -90)

cuvette_storage_65 = posx(639.800, -121.600, 220.000, 0, -180, -90) 
cuvette_storage_66 = posx(619.900, -120.800, 220.000, 0, -180, -90) 
cuvette_storage_67 = posx(599.950, -120.500, 220.000, 0, -180, -90) 
cuvette_storage_68 = posx(580.300, -120.100, 220.000, 0, -180, -90) 
cuvette_storage_69 = posx(560.500, -119.700, 220.000, 0, -180, -90) 
cuvette_storage_70 = posx(540.600, -119.700, 220.000, 0, -180, -90) 
cuvette_storage_71 = posx(520.800, -119.600, 220.000, 0, -180, -90) 
cuvette_storage_72 = posx(500.800, -119.100, 220.000, 0, -180, -90) 

cuvette_storage_73 = posx(480.800, -118.700, 220.000, 0, -180, -90)
cuvette_storage_74 = posx(460.600, -118.700, 220.000, 0, -180, -90)
cuvette_storage_75 = posx(440.600, -118.200, 220.000, 0, -180, -90)
cuvette_storage_76 = posx(420.900, -118.200, 220.000, 0, -180, -90)
cuvette_storage_77 = posx(400.900, -118.000, 220.000, 0, -180, -90)
cuvette_storage_78 = posx(381.000, -117.800, 220.000, 0, -180, -90)
cuvette_storage_79 = posx(360.800, -117.700, 220.000, 0, -180, -90)
cuvette_storage_80 = posx(340.800, -117.500, 220.000, 0, -180, -90)

# cuvette holder point
cuvette_holder_center = posx(477.300, 19.000, 260.000, -1, -180, -90)
cuvette_holder_1 = posx(495.300, -35.000, 254.000, -1, -180, -90)
cuvette_holder_2 = posx(457.250, -34.800, 254.000, -1, -180, -90) 
# cuvette_holder_3 = posx(495.700, 0.500, 254.000, -1, -180, -90) # 20221015 전 포지션
cuvette_holder_3 = posx(495.580, 0.500, 254.000, -1, -180, -90)
cuvette_holder_4 = posx(457.800, 1.500, 254.000, -1, -180, -90)
# cuvette_holder_5 = posx(496.100, 35.400, 254.000, -1, -180, -90) # 20221015 전 포지션
cuvette_holder_5 = posx(496.080, 35.400, 254.000, -1, -180, -90)
cuvette_holder_6 = posx(458.180, 36.500, 254.000, -1, -180, -90)
cuvette_holder_7 = posx(496.620, 70.500, 254.000, -1, -180, -90)
cuvette_holder_8 = posx(458.600, 71.700, 254.000, -1, -180, -90)

# 2022-10-08 이전 나연 좌표
# cuvette_holder_1 = posx(495.200, -35.000, 254.000, -1, -180, -90)
# cuvette_holder_2 = posx(456.900, -34.800, 254.000, -1, -180, -90) 
# cuvette_holder_3 = posx(495.850, 0.100, 254.000, -1, -180, -90)
# cuvette_holder_4 = posx(457.550, 1.500, 254.000, -1, -180, -90)
# cuvette_holder_5 = posx(495.900, 35.800, 254.000, -1, -180, -90)
# cuvette_holder_6 = posx(457.800, 36.600, 254.000, -1, -180, -90)
# cuvette_holder_7 = posx(496.500, 70.300, 254.000, -1, -180, -90)
# cuvette_holder_8 = posx(458.500, 71.700, 254.000, -1, -180, -90)

# 2022-09-28 좌표
# cuvette_holder_1 = posx(495.400, -33.500, 254.000, -1, -180, -90)
# cuvette_holder_2 = posx(457.000, -33.500, 254.000, -1, -180, -90) 
# cuvette_holder_3 = posx(495.800, 1.500, 254.000, -1, -180, -90)
# cuvette_holder_4 = posx(457.900, 2.500, 254.000, -1, -180, -90)
# cuvette_holder_5 = posx(496.100, 36.700, 254.000, -1, -180, -90)
# cuvette_holder_6 = posx(458.000, 37.500, 254.000, -1, -180, -90)
# cuvette_holder_7 = posx(496.600, 72.000, 254.000, -1, -180, -90)
# cuvette_holder_8 = posx(458.500, 73.400, 254.000, -1, -180, -90)


# UV point
# pos_UV = posx(663.990, 153.600, 267.000, -2.4, -180, -90)
# pos_UV = posx(663.990, 152.600, 266.500, -2.4, -180, -90)

pos_UV = posx(675.500, 140.100, 266.500, -2.4, -180, -90)
pos_UV_2 = posx(676.100, 140.100, 266.500, -2.4, -180, -90)

#2022-10-08 나연 좌표
# pos_UV = posx(675.500, 140.100, 266.500, -3.6, -180, -90)
# pos_UV_2 = posx(676.100, 140.100, 266.500, -3.6, -180, -90)

# total grid point group list
pos_HOME_list = [pos_HOME]
pos_UV_list = [pos_UV]
pos_UV_2_list = [pos_UV_2]
grid_storage_empty_list = [storage_empty_1, storage_empty_2, storage_empty_3, storage_empty_4, storage_empty_5]
grid_holder_list = [holder_1, holder_2, holder_3, holder_4, holder_5, holder_6, holder_7, holder_8]
grid_stirrer_list = [stirrer_1, stirrer_2, stirrer_3, stirrer_4,stirrer_5, stirrer_6, stirrer_7, stirrer_8, 
                     stirrer_9, stirrer_10, stirrer_11, stirrer_12, stirrer_13, stirrer_14, stirrer_15, stirrer_16]
grid_storage_filled_list = [storage_filled_1, storage_filled_2, storage_filled_3,storage_filled_4, storage_filled_5]
cuvette_storage_list = [cuvette_storage_1,cuvette_storage_2, cuvette_storage_3, cuvette_storage_4,
                        cuvette_storage_5,cuvette_storage_6, cuvette_storage_7, cuvette_storage_8,
                        cuvette_storage_9,cuvette_storage_10, cuvette_storage_11, cuvette_storage_12,
                        cuvette_storage_13,cuvette_storage_14, cuvette_storage_15, cuvette_storage_16,
                        cuvette_storage_17,cuvette_storage_18, cuvette_storage_19, cuvette_storage_20,
                        cuvette_storage_21,cuvette_storage_22, cuvette_storage_23, cuvette_storage_24,
                        cuvette_storage_25,cuvette_storage_26, cuvette_storage_27, cuvette_storage_28,
                        cuvette_storage_29,cuvette_storage_30, cuvette_storage_31, cuvette_storage_32,
                        cuvette_storage_33,cuvette_storage_34, cuvette_storage_35, cuvette_storage_36,
                        cuvette_storage_37,cuvette_storage_38, cuvette_storage_39, cuvette_storage_40,
                        cuvette_storage_41,cuvette_storage_42, cuvette_storage_43, cuvette_storage_44,
                        cuvette_storage_45,cuvette_storage_46, cuvette_storage_47, cuvette_storage_48,
                        cuvette_storage_49,cuvette_storage_50, cuvette_storage_51, cuvette_storage_52,
                        cuvette_storage_53,cuvette_storage_54, cuvette_storage_55, cuvette_storage_56,
                        cuvette_storage_57,cuvette_storage_58, cuvette_storage_59, cuvette_storage_60,
                        cuvette_storage_61,cuvette_storage_62, cuvette_storage_63, cuvette_storage_64,
                        cuvette_storage_65,cuvette_storage_66, cuvette_storage_67, cuvette_storage_68,
                        cuvette_storage_69,cuvette_storage_70, cuvette_storage_71, cuvette_storage_72,
                        cuvette_storage_73,cuvette_storage_74, cuvette_storage_75, cuvette_storage_76,
                        cuvette_storage_77,cuvette_storage_78, cuvette_storage_79, cuvette_storage_80]

cuvette_holder_list = [cuvette_holder_1, cuvette_holder_2, cuvette_holder_3, cuvette_holder_4,
                        cuvette_holder_5, cuvette_holder_6, cuvette_holder_7, cuvette_holder_8]


storage_empty_to_stirrer_list = [[pos_XYZ, pos_storage, storage_empty_center], 
                            storage_empty_pick_1,
                            storage_empty_pick_2, 
                            [pos_storage, pos_XYZ, stirrer_center],
                            stirrer_place_1,
                            stirrer_place_2, 
                            [stirrer_center, pos_XYZ]
                            ]

stirrer_to_grid_holder_list = [[pos_XYZ, stirrer_center],
                            stirrer_pick_1, 
                            stirrer_pick_2, 
                            [stirrer_center, pos_XYZ, pos_anaylsis, holder_center], 
                            holder_place_1, 
                            holder_place_2, 
                            [holder_center, pos_anaylsis]]

stirrer_to_storage_filled_list = [[pos_XYZ, stirrer_center],
                            stirrer_pick_1, 
                            stirrer_pick_2, 
                            [stirrer_center, pos_XYZ, pos_storage, storage_filled_center], 
                            storage_filled_place_1, 
                            storage_filled_place_2, 
                            [storage_filled_center, pos_storage, pos_XYZ]]

holder_to_storage_filled_list = [[pos_anaylsis, holder_center], 
                            holder_pick_1, 
                            holder_pick_2, 
                            [holder_center, pos_anaylsis, pos_XYZ, pos_storage, storage_filled_center],
                            storage_filled_place_1, 
                            storage_filled_place_2, 
                            [storage_filled_center, pos_storage, pos_XYZ, pos_anaylsis]]

cuvette_storage_to_cuvette_holder_list = [
                            [pos_anaylsis, cuvette_storage_center],
                            cuvette_storage_pick_1,
                            cuvette_storage_pick_2,
                            [cuvette_storage_center, cuvette_holder_center],
                            cuvette_holder_place_1,
                            cuvette_holder_place_2,
                            [cuvette_holder_center, pos_anaylsis]
                            ]

cuvette_holder_to_UV_list = [[cuvette_holder_center],
                            cuvette_holder_pick_1,
                            cuvette_holder_pick_2,
                            [cuvette_holder_center],
                            UV_place_1,
                            UV_place_2,
                            [pos_anaylsis]]

UV_to_cuvette_storage_list = [[pos_anaylsis],
                            UV_pick_1,
                            UV_pick_2,
                            [cuvette_storage_center],
                            cuvette_storage_place_1,
                            cuvette_storage_place_2,
                            [cuvette_storage_center, pos_anaylsis]
                            ]

location_dict = {
    
    "move_HOME":{
        "location_list" : pos_HOME_list,
        "vel":{
            "general" : [900, 300]
        },
        "acc":{
            "general" : [500, 300]
        }
    },

    "storage_empty_to_stirrer":{
        "location_list" : storage_empty_to_stirrer_list,
        "pick_loc_list" : grid_storage_empty_list,
        "place_loc_list" : grid_stirrer_list,
        "object_type" : 'Vial',
        "ref" : "DR_BASE",
        "vel":{
            "general" : [900, 300],
            "pick" : [500, 50],
            "place" : [400, 200]
        },
        "acc":{
            "general" : [500, 300],
            "pick" : [50, 30],
            "place" : [50, 30]
        },
        "msg":{
            "from" : "Storage_emtpy",
            "to" : "Stirrer"
        }
    },

    "stirrer_to_holder":{
        "location_list" : stirrer_to_grid_holder_list,
        "pick_loc_list" : grid_stirrer_list,
        "place_loc_list" : grid_holder_list,
        "object_type" : 'Vial',
        "ref" : "DR_BASE",
        "vel":{
            "general" : [900, 300],
            "pick" : [500, 50],
            "place" : [400, 200]
        },
        "acc":{
            "general" : [500, 300],
            "pick" : [50, 30],
            "place" : [50, 30]
        },
        "msg":{
            "from" : "Stirrer",
            "to" : "Holder"
        }
    },

    "stirrer_to_storage_filled":{
        "location_list" : stirrer_to_storage_filled_list,
        "pick_loc_list" : grid_stirrer_list,
        "place_loc_list" : grid_storage_filled_list,
        "object_type" : 'Vial',
        "ref" : "DR_BASE",
        "vel":{
            "general" : [900, 300],
            "pick" : [500, 50],
            "place" : [400, 200]
        },
        "acc":{
            "general" : [500, 300],
            "pick" : [50, 30],
            "place" : [50, 30]
        },
        "msg":{
            "from" : "Stirrer",
            "to" : "Storage_filled"
        }
    },

    "holder_to_storage_filled":{
        "location_list" : holder_to_storage_filled_list,
        "pick_loc_list" : grid_holder_list,
        "place_loc_list" : grid_storage_filled_list,
        "object_type" : 'Vial',
        "ref" : "DR_BASE",
        "vel":{
            "general" : [900, 300],
            "pick" : [500, 50],
            "place" : [400, 200]
        },
        "acc":{
            "general" : [500, 300],
            "pick" : [50, 30],
            "place" : [50, 30]
        },
        "msg":{
            "from" : "Holder",
            "to" : "Storage_filled"
        }
    },

    "cuvette_storage_to_cuvette_holder":{
        "location_list" : cuvette_storage_to_cuvette_holder_list,
        "pick_loc_list" : cuvette_storage_list,
        "place_loc_list" : cuvette_holder_list,
        "object_type" : 'Cuvette',
        "ref" : "DR_BASE",
        "vel":{
            "general" : [900, 300],
            "pick" : [500, 50],
            "place" : [400, 200]
        },
        "acc":{
            "general" : [500, 300],
            "pick" : [50, 30],
            "place" : [50, 30]
        },
        "msg":{
            "from" : "Cuvette_storage",
            "to" : "Cuvette_holder"
        }
    },

    "cuvette_holder_to_UV":{
        "location_list" : cuvette_holder_to_UV_list,
        "pick_loc_list" : cuvette_holder_list,
        "place_loc_list" : pos_UV_list,
        "object_type" : 'Cuvette',
        "ref" : "DR_BASE",
        "vel":{
            "general" : [900, 300],
            "pick" : [500, 50],
            "place" : [250, 150]
        },
        "acc":{
            "general" : [500, 300],
            "pick" : [50, 30],
            "place" : [25, 15]
        },
        "msg":{
            "from" : "Cuvette_holder",
            "to" : "UV"
        }
    },

    "UV_to_cuvette_storage":{
        "location_list" : UV_to_cuvette_storage_list,
        "pick_loc_list" : pos_UV_2_list,
        "place_loc_list" : cuvette_storage_list,
        "object_type" : 'Cuvette',
        "ref" : "DR_BASE",
        "vel":{
            "general" : [900, 300],
            "pick" : [500, 50],
            "place" : [400, 200]
        },
        "acc":{
            "general" : [500, 300],
            "pick" : [50, 30],
            "place" : [50, 30]
        },
        "msg":{
            "from" : "UV",
            "to" : "Cuvette_storage"
        }
    }
}