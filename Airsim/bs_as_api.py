#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2020-03-22 22:40:44
# @Author  : BrightSoul (653538096@qq.com)



import time

import pprint
import airsim
import numpy as np
import cv2
import threading



rad = np.pi / 180.0
deg = 180 / np.pi

# client = airsim.MultirotorClient()
# client = airsim.MultirotorClient("192.168.2.103")
# client = airsim.MultirotorClient("192.168.1.192")
client = airsim.MultirotorClient()
# client = airsim.MultirotorClient("172.17.0.1")
img_count = 0
time_txt = time.strftime("%Y%m%d_%H%M%S", time.localtime())

SERVO_START     =   0
SERVO_FIND      =   1
SERVO_APPROACH  =   2
SERVO_THROUGH   =   3

circle_count = 0

class BSQueue():
    def __init__(self,length):
        self.queue = []
        self.length = length

    def append(self,element):
        if len(self.queue) < self.length:
            self.queue.append(element)
        else:
            self.queue.pop(0)
            self.queue.append(element)

    def average(self):
        return np.average(self.queue)

    def clr(self):
        self.queue = []
    
    def __len__(self):
        return len(self.queue)
    
    def __str__(self):
        return str(self.queue)

class AsAssist:
    def print_time(self):
        print("[{:.2f}]".format(time.time()-self.start_time),end="")

    # FLU逆时针转yaw到NED
    # NED转到FLU也是一样的，NED的正方向决定了旋转矩阵，符号决定了一样的
    def FLU2NED(self, x,y,z):
        '''
        逆时针顺z轴因而取正
        | cos   sin |
        | -sin    cos |
        '''

   
        mav_state = client.simGetGroundTruthKinematics(self.vehicle_name)
        # yaw角在NED坐标系下
        pry = airsim.to_eularian_angles(mav_state.orientation)
        current_heading = pry[2]
        NED_x = x * np.cos(current_heading) + y * np.sin(current_heading)
        NED_y = -(-x * np.sin(current_heading) + y * np.cos(current_heading))
        NED_z = -z

        return np.array([NED_x, NED_y, NED_z])


    # return ypr in rad from quantion
    def q2ypr(self, q):
        if isinstance(q, Quaternion):
            ypr_rad = q.yaw_pitch_roll
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            ypr_rad = q_.yaw_pitch_roll

        return ypr_rad

    # constrain_deg -360~720 to 0~360
    def constrain_deg(self,deg):
        if deg >= 360:
            return (deg - 360)        
        elif deg <= 0:
            return (deg + 360)
        else:
            return deg

    def kine(self,args):
        if len(args) == 0:
            mav_name = self.vehicle_name
        else:
            mav_name = self.name_prefix.format(int(mav_id))
        mav_state = client.simGetGroundTruthKinematics(mav_name)
        pry = airsim.to_eularian_angles(mav_state.orientation)
        
        print(mav_name)
        pprint.pprint(mav_state)
        print("pry:",[np.rad2deg(i) for i in pry])
        # pprint.pprint(dir(client))
        # print(client)
        
        # pprint.pprint(mav_state.linear_velocity)
        # pprint.pprint(mav_state.position)
        # print(mav_state.linear_velocity)
        # print(vars(mav_state.linear_velocity))

    def getImg_test(self):
        global img_count
        # cam_list = {0:"front_center",1:"front_cc",2:"bottom_center"}
        cam_list = ["front_center","bottom_center"]
        responses = client.simGetImages([
            # tan(fov) = w/2f
            # f =w/(2* tan(fov))
            # arctan(tan(50)/n)
            # w=640,fov=50*2 -> f = 268.5
            # w=640,fov=30.7*2 <- f = 268.5*2
            # w=640,fov=13.4*2 <- f = 268.5*5
            # w=640,fov=6.7*2 <- f = 268.5*10
            airsim.ImageRequest(cam_list[cam_id], airsim.ImageType.Scene, False, False)
            for cam_id in range(len(cam_list))
        ])
        for cam_id,response in enumerate(responses):
            img = as2cv(response)
            # bmp>jpg>png
            cv2.imwrite(f"img/{time_txt}_{cam_list[cam_id]}_{img_count}.png",img)
        img_count += 1

            # cv2.imshow("front_center",img)
            # cv2.waitKey(1)
    def state(self,args):
        if len(args) == 0:
            mav_name = self.vehicle_name
        else:
            mav_name = self.name_prefix.format(int(mav_id))
        mav_state = client.getMultirotorState(mav_name)
        print(mav_state.kinematics_estimated.position)
        # print(mav_state.kinematics_estimated.position.to_numpy_array())
        # pprint.pprint(mav_state)

    def getCircle_Depth(self):
        responses = client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, pixels_as_float = True, compress = False),
            airsim.ImageRequest("0", airsim.ImageType.Scene, pixels_as_float = False, compress = False)
        ])    



        depthperspective = responses[0]
        depthperspective = airsim.get_pfm_array(depthperspective)
        depthperspective[depthperspective > 5] = 0
        depthperspective = depthperspective.astype(np.uint8)
        depthperspective = cv2.equalizeHist(depthperspective)


        rgb = responses[1]
        img1d = np.frombuffer(rgb.image_data_uint8, dtype=np.uint8) 
        img_rgb = img1d.reshape(rgb.height, rgb.width, 3)

        circles = findCircle(depthperspective)
        if circles is not None:

            circle = circles[0]
            x, y ,r = [int(i) for i in circle]

            img_rgb = cv2.circle(img_rgb, (x, y), r, (0, 0, 255), 3)
            img_rgb = cv2.circle(img_rgb, (x, y), 2, (255, 255, 0), -1) 
        
        cv2.imshow('new', img_rgb)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()
        

        # start_time = time.time()
        # image = client.simGetImage("front_center", airsim.ImageType.Scene)
        # img_bgr = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
        # t1 = time.time() - start_time

        # start_time = time.time()
        # responses = client.simGetImages([
        #             airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
        # img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8) #get numpy array
        # img_bgr = img1d.reshape(responses[0].height, responses[0].width, 3) #reshape array to 3 channel image array H X W X 3
        # t2 = time.time() - start_time

        # print(t1,t2)

        # image = client.simGetImage("front_center", airsim.ImageType.DepthPerspective)
        # # img_depth = np.frombuffer(image, np.float32)
        # # img_depth = cv2.imdecode(, cv2.IMREAD_GRAYSCALE)
        # # print(type(img_depth))
        # # print(img_depth.shape)
        # print(len(image))
     

class AsController(AsAssist):
    # global client
    
    def __init__(self,mav_id=1,home = [0,0,0]):
        self.v_max = 2

        self.mav_id = mav_id
        self.taskstate = None
        self.taskdone = False

        self.home = np.array(home)

        self.name_prefix = "drone_{:d}"
        # self.vehicle_name = "mav_{:02d}".format(int(mav_id))
        self.vehicle_name = self.name_prefix.format(int(mav_id))
        
        
        self.init_func()
        print("init of mav_{:02d}".format(mav_id))
        

    def start(self, start_time=time.time()):

        self.start_time = start_time


        # init_succ = False
        # while not init_succ:
        #     try:
        #         client.enableApiControl(True, vehicle_name=self.vehicle_name)
        #         self.takeoff()
        #         init_succ = True
        #     except Exception as e:
        #         continue
        
        client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.takeoff()
        
        # s = threading.Thread(target=self.airsim_save, args=())
        # s.start()


    def init_func(self):
        self.NED2FLU = self.FLU2NED

        self.cmdmap = {}
        
        # function for debug 
        self.cmdmap["state"] = self.state
        self.cmdmap["kine"] = self.kine
        
        self.cmdmap["imgt"] = self.getImg_test
        self.cmdmap["imgd"] = self.getCircle_Depth

        self.cmdmap["imu"] = self.imu
        self.cmdmap["vflu"] = self.get_vflu

        # function for zzfx
        # vp 15.80 -17.6 4.8 -90
        # self.cmdmap["vp"] = self.velocity_pos
        # self.cmdmap["vo"] = self.vision_servo
        # self.cmdmap["vl"] = self.vision_land
        


        # function for control
        self.cmdmap["arm"] = self.arm
        self.cmdmap["disarm"] = self.disarm

        self.cmdmap["takeoff"] = self.takeoff
        self.cmdmap["land"] = self.land
        self.cmdmap["rtl"] = self.rtl
        self.cmdmap["stop"] = self.stop

        self.cmdmap["v"] = self.velocity
        self.cmdmap["vr"] = self.velocity_yawrate
        self.cmdmap["move"] = self.move
        self.cmdmap["pose"] = self.pose

        self.cmdmap["yaw"] = self.yaw
        self.cmdmap["rate"] = self.yaw_rate

        self.cmdmap["hover"] = self.hover

    def decode(self, cmd, args):
        # print(cmd, args)
        # print(type(self.cmdmap[cmd]))
        try:
            self.cmdmap[cmd](args)
        except TypeError as e:
            self.cmdmap[cmd]()
        
    

    # 用于保存后期重建的数据
    # def airsim_save(self):
    #     f = open("data/{}_{}".format(time_txt,self.mav_id),"w")
    #     save_time = time.time()
    #     while True:
    #         if time.time() - save_time > 0.02:
    #             save_time = time.time()
    #             pos_vec,vel_vec,yaw_deg = self.get_pvy()
                

    #             data_txt = "[{:.2f}] {} {} {} {} {} {} {}".format(time.time()-self.start_time,*pos_vec,*vel_vec,yaw_deg)
    #             f.write(data_txt)
    #             f.write("\n")
    #             self.print_time()
    #             print("save done")

    def setHome(self,home=[0,0,0]):
        self.home = np.array(home)

    # 相对于起飞点加入了位置偏移
    # 数据类型为np.array
    # 获取pose,vel和yaw
    def get_pvy(self):
        run_succ = False
        mav_state = client.simGetGroundTruthKinematics(self.vehicle_name)
        # NED位置
        p = mav_state.position.to_numpy_array()
        # NED速度
        v = mav_state.linear_velocity.to_numpy_array()
        yaw_deg = np.rad2deg(airsim.to_eularian_angles(mav_state.orientation)[2])
        return self.home+p,v,yaw_deg

    def get_vflu(self):
        mav_state = client.simGetGroundTruthKinematics(self.vehicle_name)
        v = mav_state.linear_velocity.to_numpy_array()
        vflu = self.NED2FLU(*v)
        # print("NED:",v)
        # print("FLU:",vflu)
        return np.array(vflu)    
    def imu(self):
        mav_state = client.simGetGroundTruthKinematics(self.vehicle_name)
        v = mav_state.linear_velocity.to_numpy_array()
        vflu = self.NED2FLU(*v)
        # print("NED:",v)
        # print("FLU:",vflu)
        return np.array(vflu)

    def arm(self):
        client.armDisarm(True, vehicle_name=self.vehicle_name)
    
    def disarm(self):
        client.armDisarm(False, vehicle_name=self.vehicle_name)
        
    def takeoff(self):
        client.takeoffAsync(vehicle_name=self.vehicle_name)
    
    def land(self):
        client.landAsync(vehicle_name=self.vehicle_name)
    
    def rtl(self):
        # client.goHomeAsync(vehicle_name=self.vehicle_name)
        client.moveToPositionAsync(0,0,0,1.5,vehicle_name=self.vehicle_name)
    


    # 给的是NED的坐标系
    def velocity_NED(self, args):
        vx, vy, vz = [float(i) for i in args]
        client.moveByVelocityAsync(vx, vy, vz,duration=3,vehicle_name=self.vehicle_name)

    # API是FRD
    def velocity(self,args):
        F,R,D = 0,0,0
        if (len(args) == 0):
            pass   
        elif (len(args) == 1):
            F = float(args[0])           
        elif (len(args) == 2):
            R = float(args[0])
            D = float(args[1])
        elif (len(args) == 3):
            F = float(args[0])
            R = float(args[1])
            D = float(args[2])
        else:
            print("args error:",len(args))
            return
        client.moveByVelocityBodyFrameAsync(F,R,D,duration=30,vehicle_name=self.vehicle_name)
    



    # 给到NED
    # 传入NEU
    def pose(self,args):
        mav_state = client.simGetGroundTruthKinematics(self.vehicle_name)
        N,E,D =  mav_state.position.to_numpy_array().tolist()
        if (len(args) == 0):
            pass   
        elif (len(args) == 1):
            D = -float(args[0])           
        elif (len(args) == 2):
            N = float(args[0])
            E = float(args[1])
        elif (len(args) == 3):
            N = float(args[0])
            E = float(args[1])
            D = -float(args[2])
        else:
            print("args error:",len(args))
            return
        v_max = np.min([np.linalg.norm([N,E,D]),self.v_max])   
        client.moveToPositionAsync(N,E,D,v_max,vehicle_name=self.vehicle_name)

        # N, E, D = [float(i) for i in args]
        # client.moveToPositionAsync(N, E, D,self.v_max,vehicle_name=self.vehicle_name)

    # 习惯上只改变F和R
    # 但保持和MAVROS一致就FLU
    def move(self,args):
        F,L,U = 0,0,0
        if (len(args) == 0):
            pass   
        elif (len(args) == 1):
            U = float(args[0])           
        elif (len(args) == 2):
            F = float(args[0])
            L = float(args[1])
        elif (len(args) == 3):
            F = float(args[0])
            L = float(args[1])
            U = float(args[2])
        else:
            print("args error:",len(args))
            return
           
        mav_state = client.simGetGroundTruthKinematics(self.vehicle_name)
        current_p = mav_state.position.to_numpy_array()     
        delta_p = np.array(self.FLU2NED(F,L,U))
        p = (current_p+delta_p)
        if np.linalg.norm(delta_p)<10:
            v_max = np.min([np.linalg.norm(delta_p),self.v_max])
        else:
            v_max = 1
        # print(v_max)
        # print(p)
        client.moveToPositionAsync(*p,v_max,vehicle_name=self.vehicle_name)
        # if np.linalg.norm(delta_p)<10:
        #     client.moveToPositionAsync(*p,v_max,vehicle_name=self.vehicle_name)
        # else:
        #     client.moveToPositionAsync(*p,v_max,vehicle_name=self.vehicle_name).join()


    def stop(self):
        self.velocity([0,0,0]) 
    
    # yaw_rate in deg
    def velocity_yawrate(self, args):
        F,L,U,yaw_rate= [float(i) for i in args]
        vx, vy, vz = self.FLU2NED(F,L,U)
        client.moveByVelocityAsync(vx, vy, vz,duration=3,yaw_mode = airsim.YawMode(True, yaw_rate),vehicle_name=self.vehicle_name)
    
    def vr_rad(self, args):
        F,L,U,yaw_rate= [float(i) for i in args]
        vx, vy, vz = self.FLU2NED(F,L,U)
        client.moveByVelocityAsync(vx, vy, vz,duration=3,yaw_mode = airsim.YawMode(True, np.rad2deg(yaw_rate)),vehicle_name=self.vehicle_name)

    # ±180之间
    def yaw(self, args):
        yaw_deg = float(args[0])
        client.moveByVelocityAsync(0, 0, 0,duration=3,yaw_mode = airsim.YawMode(False, yaw_deg),vehicle_name=self.vehicle_name)
    
    # 传入deg/s,正的向右
    def yaw_rate(self, args):
        yaw_rate = float(args[0])
        client.moveByVelocityAsync(0, 0, 0,duration=1,yaw_mode = airsim.YawMode(True, yaw_rate),vehicle_name=self.vehicle_name)

    # yaw_rate in rad
    def hover(self,args):
        f_time = time.time()
        F,yaw_rate = 1,1
        while time.time()-f_time < 3:
            # vx, vy, vz = self.FLU2NED(F,0,0)
            print(vx, vy, vz)
            client.moveByVelocityAsync(vx, vy, 0,duration=3,yaw_mode = airsim.YawMode(True, np.rad2deg(yaw_rate)),vehicle_name=self.vehicle_name)
        self.stop()
