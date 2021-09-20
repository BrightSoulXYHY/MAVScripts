#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2019-06-30 21:20:44
# @Author  : BrightSoul (653538096@qq.com)

'''
利用pymavlink解析一些消息并且可以发送一些消息

pymavlink              2.4.15
pymavlink升级了，有些指令大概是变了
'''
import time,codecs,os
import readline
os.environ['MAVLINK20'] = '1'



from pymavlink import mavutil
import threading

# 用于切换模式
dict_custom_mode = {"stb":0, "althd":2, "guided":4, "fuck":24, }



PI = 3.1415926
rad = PI/180
deg = 180/PI
# 不想显示的消息
L_Undesire = ["HEARTBEAT","STATUSTEXT","PARAM_VALUE","TIMESYNC",
              "COMMAND_ACK",
              "BAD_DATA",


            "RC_CHANNELS","SERVO_OUTPUT_RAW","MISSION_CURRENT","POWER_STATUS",
            "SYS_STATUS","GLOBAL_POSITION_INT","ATTITUDE","BATTERY_STATUS","VIBRATION",
            "LOCAL_POSITION_NED","EKF_STATUS_REPORT","HWSTATUS","AHRS3","AHRS2","AHRS",

            "VFR_HUD","RC_CHANNELS_SCALED","GPS_RAW_INT","SCALED_IMU3","SCALED_IMU2","RAW_IMU",
            "MEMINFO","SYSTEM_TIME","SCALED_PRESSURE","SENSOR_OFFSETS",

            "SIMSTATE", "RC_CHANNELS_RAW"
]


# txt_time = time.strftime('%Y%m%d-%H%M', time.localtime(time.time()))
# fo = codecs.open("_Log/"+txt_time, 'w', 'utf-8')




# Start a connection listening to a UDP port
# the_connection = mavutil.mavlink_conardunection('tcpin:localhost:5763')
# the_connection = mavutil.mavlink_conardunection('udpin:localhost:14550')
# 
# 
# 
# the_connection = mavutil.mavlink_connection('tcp:localhost:5760')
# the_connection = mavutil.mavlink_connection('tcp:localhost:5760')
# the_connection = mavutil.mavlink_connection('tcp:localhost:5887')
# 
# the_connection = mavutil.mavlink_connection('udpin:192.168.2.100:14552')
# the_connection = mavutil.mavlink_connection('udpin:192.168.91.1:14550')
# the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
# 
# the_connection = mavutil.mavlink_connection('udpout:192.168.1.246:14580')
# the_connection = mavutil.mavlink_connection('udpin:192.168.1.192:14550')
the_connection = mavutil.mavlink_connection('udpin:192.168.1.192:14541')

# the_connection = mavutil.mavlink_connection('com4',baud=57600)
# the_connection = mavutil.mavlink_connection('com10',baud=57600)
# the_connection = mavutil.mavlink_connection('com15',baud=57600)
# the_connection = mavutil.mavlink_connection('com38',baud=57600)
print("connection start")
print("waitiing for hb")
the_connection.wait_heartbeat()

start_time = time.time()

def print_time():
    print("[%.2f]"%(time.time()-start_time),end='')


def recv_msg():
    wt_hb_flag = 0
    print("Start Listen!")
    while True:
        # 接arducopter_3.6.9_Fuck模式加了控制接口收数据:
        msg = the_connection.recv_match(blocking=True)
        if msg.get_type() == "HEARTBEAT" and wt_hb_flag:
            print(msg)        

        # if msg.get_type() == "COMMAND_LONG":
        #     print(msg)        

        # if msg.get_type() == "GOLF_TEST":
        #     print(msg)


        # if msg.get_type() == "SET_POSITION_TARGET_LOCAL_NED" :
        #     print_time()
        #     print(msg)
            
        if msg.get_type() == "COMMAND_ACK":
            print("command {} is {}".format(msg.command, msg.result))

        # if msg.get_type() == "ATTITUDE":
            # print(msg)        


        if msg.get_type() == "STATUSTEXT":
            if msg.severity == 4 or msg.severity == 7:
                print(str(msg.text).strip('\x00'))
                # print(type(msg.text))
                # print(str(msg.text, encoding = "utf-8").strip('\x00'))
                # fo.write(str(msg.text, encoding = "utf-8").strip('\x00'))
                # fo.write("\r\n")
                
                
        # print("the msg type is {}".format(msg.get_type()))
        
        debug_msg = 0
        if msg.get_type() not in L_Undesire and debug_msg:
            print_time()
            print(msg)
         
        # print_time()
        # print(msg)


def main():
    print("Heartbeat from system (system %u component %u)"
          % (the_connection.target_system, the_connection.target_component))
    t = threading.Thread(target=recv_msg, args=())
    t.start()

    my_cmd = CmdlongModule(the_connection)
    while True:
        str_cdmd = input(">")
        L = str_cdmd.split(" ")
        if(L[0] == ""):
            continue
        else:
            cmd = L.pop(0)
            try:
                my_cmd.decode(cmd, L)
            except Exception as e:
                print("KeyError support cmd:")
                for key, value in my_cmd.cmdmap.items() :
                    print(key,end=' ')
                print()
                print(e)





class CmdlongModule():
    def __init__(self, the_connection):
        self.cnct = the_connection
        self.cmdmap = {}
        
        self.cmdmap["mode"] = self.moded
        self.cmdmap["arm"] = self.arm
        self.cmdmap["disarm"] = self.disarm
        
        self.cmdmap["fdisarm"] = self.fdisarm
        self.cmdmap["land"] = self.land

        self.cmdmap["hb"] = self.hb
       
        # guided模式的接口
        self.cmdmap["takeoff"] = self.takeoff
        self.cmdmap["p"] = self.pos
        self.cmdmap["v"] = self.velo
        self.cmdmap["yaw"] = self.yaw


        self.cmdmap["BCMD"] = self.BS_CMD
        self.cmdmap["data"] = self.data

    def decode(self, cmd, args):
        self.cmdmap[cmd](args)

    def add(self, args):
        if (len(args) != 2):
            print("add need 2 parameter")
            return

        if (len(args) == 2):
            a = float(args[0])
            b = float(args[1])
            print("test function add")
            print("add (system %u component %u)"
                  % (self.the_connection.target_system,
                     self.the_connection.target_component))
            print(a+b)

    def moded(self, args):
        if (len(args) != 1):
            print(dict_custom_mode)
            return
        if args[0].isdigit():
            custom_mode=int(args[0])
        else:
            custom_mode=dict_custom_mode[args[0]]
        self.cnct.mav.set_mode_send(self.cnct.target_system, 1, custom_mode)

    def arm(self, args):
        self.cnct.mav.command_long_send(
            self.cnct.target_system, self.cnct.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)      


    def hb(self, args):
         self.cnct.mav.heartbeat_send(6, 8, 0, 0,0)  

    def disarm(self, args):
        self.cnct.mav.command_long_send(
            self.cnct.target_system, self.cnct.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)
    
    def fdisarm(self, args):
        self.cnct.mav.command_long_send(
            self.cnct.target_system, self.cnct.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196, 0, 0, 0, 0, 0)

    def takeoff(self, args):
        if (len(args) != 1):
            print("input altitude")
            return

        if (len(args) == 1):
            altitude = float(args[0])
            self.cnct.mav.command_long_send(
                self.cnct.target_system, self.cnct.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, altitude)     

    
    def land(self, args):
        self.cnct.mav.command_long_send(
            self.cnct.target_system, self.cnct.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0)

    
    def pos(self, args):
        if (len(args) != 3):
            print("input x,y,z")
            return
        else:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])

            # yaw ign   1<<10 1024
            # yaw rate ign  1<<11   2048
            self.cnct.mav.set_position_target_local_ned_send(
                0, self.cnct.target_system, self.cnct.target_component,
                # 相对于EKF原点和东北地坐标系,位置需要给负，速度相对于前右下
                # mavutil.mavlink.MAV_FRAME_BODY_NED, 
                # 相对于飞机当前位置和指向，飞机前右下，速度位置
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
                0b110111111000,
                x, y, z,
                0, 0, 0, 
                0, 0, 0, 
                0, 0)    

    # 输入角度，响应到弧度
    def yaw(self, args):
        if (len(args) != 1):
            print("input yaw in rad")
            return
        yaw = float(args[0])
        # yaw_rate = float(args[1])
        mask = 0b000111111111
        # mask = 0b111111111111 - (1<<10)-(1<<3)-(1<<4)
        self.cnct.mav.set_position_target_local_ned_send(
            0, self.cnct.target_system, self.cnct.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
            mask,
            0, 0, 0,
            0, 0, 0, 
            0, 0, 0, 
            yaw, 1)

    def velo(self, args):
        if (len(args) != 3):
            print("input vx,vy,vz")
            return
        else:
            vx = float(args[0])
            vy = float(args[1])
            vz = float(args[2])

            self.cnct.mav.set_position_target_local_ned_send(
                0, self.cnct.target_system, self.cnct.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED, 
                0b11011000111,
                0, 0, 0, 
                vx, vy, vz,
                0, 0, 0, 
                0, 0)     



    def ekfhm(self, args):
        self.cnct.mav.set_gps_global_origin_send(
            self.cnct.target_system, 10, 0, 0)


    def BS_CMD(self, args):
        if len(args) == 0:
            print("input cmd para1-7")
            return
        ID = int(args.pop(0))
        A = [0]*7
        for i in range(len(args)):
            A[i] =  float(args[i])

        print(A)
            
        self.cnct.mav.command_long_send(
            self.cnct.target_system, self.cnct.target_component,
            ID, 0,
            A[0],A[1],A[2],A[3],A[4],A[5],A[6])


    def data(self,args):
        if len(args) == 0:
            start = 1
        else:
            start = int(args[0])
            
        # def request_data_stream_send(self, target_system, target_component, req_stream_id, req_message_rate, start_stop, force_mavlink1=False):
        # REQUEST_DATA_STREAM {target_system : 0, target_component : 0, req_stream_id : 2, req_message_rate : 2, start_stop : 1}
        self.cnct.mav.request_data_stream_send(
                 self.cnct.target_system, self.cnct.target_component,
                 2,2,1)
              

if __name__ == '__main__':
    main()
