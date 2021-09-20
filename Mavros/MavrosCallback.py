import numpy as np



rad = np.pi / 180.0
deg = 180 / np.pi

class MavrosCallback:
    yaw_revise = -20*rad

    # 直接获得np数组
    def local_pose_callback(self, msg):
        pos = msg.pose.position
        pose_vec = np.array([pos.x,pos.y,pos.z],dtype=float)+self.home
        self.local_pose_vec = pose_vec
        
        self.local_pose = msg
        self.local_enu_position = msg
        self.current_height = pos.z
        # 在仿真中有误差，rad
        # self.current_heading = -self.q2ypr(msg.pose.orientation)[0]+90*rad

    # ENU的速度
    def local_vel_callback(self, msg):
        vel = msg.twist.linear
        vel_vec = np.array([vel.x,vel.y,vel.z],dtype=float)
        self.local_vel = msg.twist.linear
        self.local_angular = msg.twist.angular

        self.local_vel_vec = vel_vec

    # FLU的速度
    def body_vel_callback(self, msg):
        self.body_vel = msg.twist.linear

    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        self.arm_state = msg.armed
        self.mode_str = msg.mode


    def imu_callback(self, msg):
        # global global_imu, current_heading, current_height
        self.imu = msg
        self.received_imu = True
        # self.current_heading = self.q2ypr(msg.orientation)[0]
        
        # ENU下面的朝向
        self.current_heading = self.q2yaw(msg.orientation)

        # 有20度偏差，不知道为啥
        # self.yaw_revise=-20*rad
        # 0 = 20+(yaw_revise)
        self.current_heading_tmp = self.current_heading+self.yaw_revise


    def gps_callback(self, msg):
        self.gps = msg

