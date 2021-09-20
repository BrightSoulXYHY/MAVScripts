# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import threading
import numpy as np



rad = np.pi / 180.0
deg = 180 / np.pi

'''
从模式转换，解锁，起飞进行控制
'''

class MavrosDebug:
    def print_mav(self):
        pos = self.local_pose.pose.position
        vel = self.local_vel

        print("local_pose: e={:2f} n={:2f} u={:2f}".format(pos.x,pos.y,pos.z))        
        # print("vel: x={:4f} y={:4f} z={:4f}".format(vel.x,vel.y,vel.z))

        # print("ypr_enu(deg):",["{:4f}".format(i*deg) for i in ypr])
        
        # yaw = self.constrain_deg(-ypr[0]*deg+90)
        # print("yaw={:4f}".format(yaw))

        # self.current_heading = self.q2ypr(msg.orientation)[0]
        # self.current_heading = self.q2yaw(msg.orientation)
        msg = self.local_pose.pose
        # print("q2ypr={:4f}".format(self.q2ypr(msg.orientation)[0]*deg))
        # print("q2yaw={:4f}".format(self.q2yaw(msg.orientation)*deg))
        print("ENU current_heading={:4f}".format(self.q2yaw(msg.orientation)*deg))


        print("current_heading_tmp={:4f}".format(self.q2yaw(msg.orientation)*deg-20))



from MavrosAssist import MavrosAssist
from MavrosCallback import MavrosCallback
from MavrosCommander import MavrosCommander


class Px4Controller(MavrosCommander,MavrosCallback,MavrosAssist,MavrosDebug):

    def __init__(self,home=[0,0,0],start_time=time.time()):

        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.current_height = None
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None

        self.received_new_task = False
        self.arm_state = False
        self.mode_str = None

        self.mavros_state = None

        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.state = None

        self.vel_ctl = False

        self.mav_id = 0
        self.home = np.array(home)
        self.start_time = start_time


        '''
        ros subscribers
        '''
        '''
        state
            armed: False
            guided: True
            manual_input: False
            mode: "AUTO.LAND"
            system_status: 3
        '''

        # 对于单个mav而言,和串口连接的是真实的
        # 从mav_str来的是dds发过来的
        # ENU系的位置
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        
        # body系下的速度
        self.body_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.body_vel_callback)
        
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        # GPS位置
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)



        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        
        self.init_func()
        
   
    def start(self):
        rospy.init_node("offboard_node",anonymous=True,disable_signals=True)
        while True:
            pass
            if  ((self.current_heading is not None) and 
                    (self.local_pose is not None) and 
                    (self.mavros_state is not None)):
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        print("Px4Controller init!")
        print("current_heading={}".format(self.current_heading*deg))

        self.move([0,0,2.5])
        print("MAV arm is",self.mavros_state.armed)
        if self.mavros_state.armed:
            print("Px4Controller init in sky!")
            self.move([0,0,0])
            
        for i in range(10):
            self.local_target_pub.publish(self.cur_target_pose)
            self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)
        print("start publish")
        t = threading.Thread(target=self.mavros_pub, args=())
        t.start()        
        s = threading.Thread(target=self.mavros_save, args=())
        s.start()

    def mavros_pub(self):
        pub_time = time.time()

        while (rospy.is_shutdown() is False):
            # # 20Hz
            # if time.time() - pub_time > 0.05:
            #     pub_time = time.time()
                if self.arm_state and self.offboard_state:
                    self.local_target_pub.publish(self.cur_target_pose)
       
    def mavros_save(self):
        file_name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        f = open("{}.txt".format(file_name),"w")
        f.close()

        save_time = time.time()
        start_time = self.start_time
        data_txt = "{:.3f},"+("{},"*7)[:-1]
        while True:
            if time.time()-save_time > 0.02:
                save_time = time.time()
                pos_vec = self.local_pose_vec
                vel_vec = self.local_vel_vec
                
                f = open("{}.txt".format(file_name),"a")
                f.write(data_txt.format(time.time(),*pos_vec,*vel_vec,self.current_heading*deg))
                f.write("\n")
                f.close()


    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False


    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False    

    def land(self):
        if self.flightModeService(custom_mode='AUTO.LAND'):
            return True
        else:
            print("Vechile AUTO.LAND failed")
            return False
    
    def rtl(self):
        if self.flightModeService(custom_mode='AUTO.RTL'):
            return True
        else:
            print("Vechile AUTO.LAND failed")
            return False

    
    '''
     Receive A Custom Activity
     '''

    def custom_activity_callback(self, msg):

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_postarget(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         0.1,
                                                         self.current_heading)

        if msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


















