from MavrosAssist import MavrosAssist
from MavrosCallback import MavrosCallback

import numpy as np

rad = np.pi / 180.0
deg = 180 / np.pi

class MavrosCommander(MavrosCallback,MavrosAssist):
    def init_func(self):
        self.cmdmap = {}
        
        self.cmdmap["offboard"] = self.offboard
        
        self.cmdmap["arm"] = self.arm
        self.cmdmap["disarm"] = self.disarm
        
        self.cmdmap["land"] = self.land
        self.cmdmap["rtl"] = self.land
        

        self.cmdmap["yaw"] = self.yaw
        self.cmdmap["turn"] = self.turn


        self.cmdmap["move"] = self.move
        self.cmdmap["p"] = self.pose


        self.cmdmap["v"] = self.velocity
        self.cmdmap["stop"] = self.stop

        self.cmdmap["venu"] = self.velocity_ENU

        self.cmdmap["hc"] = self.half_circle

        # ----------------下面的指令没有严格测试过--------------------
        self.cmdmap["vd"] = self.vel_with_dis
        self.cmdmap["vr"] = self.velocity_yawrate


        self.cmdmap["yaw_rate"] = self.yaw_rate

        self.cmdmap["accel"] = self.accel


        # ----------------用于debug的指令--------------------
        self.cmdmap["lsmav"] = self.print_mav
        self.cmdmap["test"] = self.test

        
        
    def decode(self, cmd, args):
        try:
            self.cmdmap[cmd](args)
        except TypeError as e:
            self.cmdmap[cmd]()


    def test(self, args):
        x,y,z = self.local_pose.pose.position.x,self.local_pose.pose.position.y,self.current_height
        yaw = self.current_heading+self.yaw_revise
        if (len(args) == 1):
            yaw = float(args[0])*rad
        elif (len(args) == 3):
            x = float(args[0])
            y = float(args[1])
            z = float(args[2]) 
        self.cur_target_pose = self.construct_postarget(x,y,z,yaw) 
    
    # 传入NED下角度制的yaw，0~360
    # 转换到-180~180的角度
    # 0对正东，±90为北和南
    # 响应到ENU下的角yaw_rad
    # 但是有20的偏差，就得修正一下
    def yaw(self, args):
        yaw_deg = -float(args[0])

        # NED转ENU
        yaw_rad = (self.constrain_deg(yaw_deg)+90)*rad

        self.cur_target_pose = self.construct_postarget(self.local_pose.pose.position.x,
                                                        self.local_pose.pose.position.y,
                                                        self.local_pose.pose.position.z,
                                                        yaw_rad+self.yaw_revise)
    # 传入角度制的yaw正向左，负向右
    # 响应到ENU下的角yaw_rad
    def turn(self, args):
        yaw_deg = float(args[0])
        current_yaw_deg = self.current_heading*deg

        yaw_rad = self.constrain_deg(current_yaw_deg+yaw_deg)*rad
        self.cur_target_pose = self.construct_postarget(self.local_pose.pose.position.x,
                                                        self.local_pose.pose.position.y,
                                                        self.local_pose.pose.position.z,
                                                        yaw_rad+self.yaw_revise)   

    # 传入FLU响应到ENU
    def move(self, args):
        x,y,z = 0,0,0
        if (len(args) == 0):
            pass
        elif (len(args) == 1):
            z = float(args[0])        
        elif (len(args) == 2):
            x = float(args[0])
            y = float(args[1])
        elif (len(args) == 3):
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
        else:
            print("args error :",len(args))
            return

        # BODY_FLU
        # For Body frame, we will use FLU (Forward, Left and Up)
        #           +Z     +X
        #            ^    ^
        #            |  /
        #            |/
        #  +Y <------body
        ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(x,y,z)

        ENU_X = ENU_X + self.local_pose.pose.position.x
        ENU_Y = ENU_Y + self.local_pose.pose.position.y
        ENU_Z = ENU_Z + self.local_pose.pose.position.z

        # self.cur_target_pose = self.construct_postarget(ENU_X, ENU_Y, ENU_Z, self.current_heading)
        self.cur_target_pose = self.construct_postarget(ENU_X, ENU_Y, ENU_Z, self.current_heading+self.yaw_revise)

    # 绝对位置ENU
    def pose(self, args):
        x,y,z = self.local_pose.pose.position.x,self.local_pose.pose.position.y,self.local_pose.pose.position.z
        if (len(args) == 0):
            pass
        elif (len(args) == 1):
            z = float(args[0])        
        elif (len(args) == 2):
            x = float(args[0])
            y = float(args[1])
        elif (len(args) == 3):
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
        else:
            print("args error:",len(args))
            return

        # ENU
        self.cur_target_pose = self.construct_postarget(x, y, z, self.current_heading+self.yaw_revise)
   


    # 输入FLU的速度
    def velocity(self, args):
        x,y,z = 0,0,0
        if (len(args) == 0):
            pass   
        elif (len(args) == 1):
            x = float(args[0])           
        elif (len(args) == 2):
            y = float(args[0])
            z = float(args[1])
        elif (len(args) == 3):
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
        else:
            print("args error:",len(args))
            return
        self.cur_target_pose = self.construct_veltarget(x,y,z)

    def circle(self,args):
        r,v = 0.5,0.5
        if (len(args) == 1):
            r = float(args[0])
        if (len(args) == 2):
            r,v = float(args[0]),float(args[1])
        yaw_rate = v/r

        # self.current_heading是ENU下
        self.cur_target_pose = self.construct_veltarget(v,0,0,yaw_rate)
   
    
    def half_circle(self,args):
        r,v = 0.5,0.5
        if (len(args) == 1):
            r = float(args[0])
        if (len(args) == 2):
            r,v = float(args[0]),float(args[1])
        yaw_rate = v/r

        # self.current_heading是ENU下
        tgt_heading_rad = self.constrain_deg180(self.current_heading*deg+180)*rad
        self.cur_target_pose = self.construct_veltarget(v,0,0,yaw_rate)
        print("current heading {:.2f} to {:.2f}".format(self.current_heading*deg,tgt_heading_rad*deg))

        while np.abs(self.current_heading - tgt_heading_rad)*deg > 3:
            pass
        print("done")
        self.stop()


    def stop(self):
        self.velocity([0,0,0])


    # 给一个ENU的速度
    def velocity_ENU(self, args):
        E,N,U = 0,0,0
        if (len(args) == 3):
            E = float(args[0])
            N = float(args[1])
            U = float(args[2])
        else:
            print("args error:",len(args))
            return
        self.cur_target_pose = self.construct_veltarget_ENU(E,N,U)
    


    # rate in deg
    def yaw_rate(self,args):
        yaw_rate = float(args[0])*rad
        # self.cur_target_pose = self.construct_yawratetarget(yaw_rate)
        self.cur_target_pose = self.construct_veltarget(0,0,0,yaw_rate)
    

    # RFU的坐标系和yaw_rate
    def velocity_yawrate(self,args):
        F,L,U,yaw_rate= [float(i) for i in args]
        # vx, vy, vz = self.FLU2NED(F,L,U)
        # self.cur_target_pose = self.construct_veltarget(R,F,U)
        self.cur_target_pose = self.construct_veltarget(-L,F,U,yaw_rate)



    def accel(self, args):
        ax, ay, az = 0,0,0
        if (len(args) == 0):
            pass   
        elif (len(args) == 1):
            ax = float(args[0])           
        elif (len(args) == 2):
            ax = float(args[0])
            ay = float(args[1])
        elif (len(args) == 3):
            ax = float(args[0])
            ay = float(args[1])
            az = float(args[2])
        else:
            print("args error:",len(args))
            return
        self.cur_target_pose = self.construct_acceltarget(ax, ay, az)

    # 传入FLU
    def vel_with_dis(self, args):
        R,F,U = 0,0,0
        if (len(args) != 4):
            print("args error:",len(args))
            return False
        else:
            F = float(args[0])
            R = -float(args[1])
            U = float(args[2])
            dis = abs(float(args[3]))

        self.cur_target_pose = self.construct_veltarget(R,F,U)
        # print("mav_{:02d} start set vel".format(self.mav_id))
        pos = self.local_pose.pose.position
        pos_vec = np.array([pos.x,pos.y,pos.z],dtype=float)
        prcent = 0.1
        while True:
            pos = self.local_pose.pose.position
            dis_vec = np.array([pos.x,pos.y,pos.z],dtype=float)
            # print(np.linalg.norm(dis_vec - pos_vec))
            if np.linalg.norm(dis_vec - pos_vec) > dis*prcent:
                # print("mav_{:02d} move {:.2f}".format(self.mav_id,dis*prcent))
                prcent+=0.1

            if np.linalg.norm(dis_vec - pos_vec) > dis:
                # print("mav_{:02d} vel move done".format(self.mav_id))
                self.stop()
                break
        return True
                


