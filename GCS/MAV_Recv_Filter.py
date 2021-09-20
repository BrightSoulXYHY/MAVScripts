#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2019-06-30 21:20:44
# @Author  : BrightSoul (653538096@qq.com)

'''
作为MAV接受来说GCS的一些控制指令
'''


import os
os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil
import threading
import time
start_time = time.time()

L_Undesire = ["HEARTBEAT","STATUSTEXT","PARAM_VALUE","TIMESYNC",
                "COMMAND_ACK",

                "REQUEST_DATA_STREAM",

                "RC_CHANNELS","SERVO_OUTPUT_RAW","MISSION_CURRENT","POWER_STATUS",
                "SYS_STATUS","GLOBAL_POSITION_INT","ATTITUDE","BATTERY_STATUS","VIBRATION",
                "LOCAL_POSITION_NED","EKF_STATUS_REPORT","HWSTATUS","AHRS3","AHRS2","AHRS",

                "VFR_HUD","RC_CHANNELS_SCALED","GPS_RAW_INT","SCALED_IMU3","SCALED_IMU2","RAW_IMU",
                "MEMINFO","SYSTEM_TIME","SCALED_PRESSURE","SENSOR_OFFSETS",

                "SIMSTATE"
]
def print_time():
    print("[%.2f]"%(time.time()-start_time),end='')


the_connection = mavutil.mavlink_connection('udpin:192.168.1.128:20101')
# the_connection = mavutil.mavlink_connection('com18',baud=57600)
# the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
# the_connection = mavutil.mavlink_connection('udpin:192.168.91.1:14550')
# the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
# the_connection = mavutil.mavlink_connection('udpout:127.0.0.1:14550')
# the_connection = mavutil.mavlink_connection('udpin:192.168.91.1:14000')
# the_connection = mavutil.mavlink_connection('udp:localhost:16200')
# the_connection = mavutil.mavlink_connection('tcp:localhost:5887')
# the_connection = mavutil.mavlink_connection('tcpin:192.168.2.100:5887')

the_connection.wait_heartbeat()

# MAV发给GCS的HB
# HEARTBEAT {type : 10, autopilot : 3, base_mode : 65, custom_mode : 0, system_status : 4, mavlink_version : 3}
# the_connection.mav.heartbeat_send(10, 3, 65, 0,4)

# GCS发给MAV的hb
# HEARTBEAT {type : 6, autopilot : 8, base_mode : 0, custom_mode : 0, system_status : 0, mavlink_version : 3}
# the_connection.mav.heartbeat_send(6, 8, 0, 0,0)
print("Start Listen!")
print("Heartbeat from system (system %u component %u)"% (the_connection.target_system, the_connection.target_component))
while True:
    # 接收数据:
    msg = the_connection.recv_match(blocking=True)
    # print(msg)

    # if msg.get_type() not in L_Undesire:
    #     print_time()
    #     print(msg)

    # print(msg.get_type())
    if msg.get_type() == "LOCAL_POSITION_NED":
        print_time()
        print(msg)





