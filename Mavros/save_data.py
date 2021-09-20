import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import time
import threading
import numpy as np

class MavrosSaver:
    def __init__(self,start_time=time.time()):


        self.local_pose_vec = None
        self.local_vel_vec = None

        self.start_time = start_time
        # ENU系的位置
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        rospy.init_node("save_node",anonymous=True,disable_signals=True)


    def start(self):
        s = threading.Thread(target=self.mavros_save, args=())
        s.start()
    
    
    def mavros_save(self):
        file_name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        f = open("{}.txt".format(file_name),"w")
        f.close()


        save_time = time.time()
        data_txt = "{:.3f},"+("{},"*6)[:-1]
        while True:
            if time.time()-save_time > 0.02 and (self.local_pose_vec is not None) and (self.local_vel_vec is not None):
                save_time = time.time()
                pos_vec = self.local_pose_vec
                vel_vec = self.local_vel_vec
                f = open("{}.txt".format(file_name),"a")
                f.write(data_txt.format(time.time(),*pos_vec,*vel_vec))
                f.write("\n")
                f.close()
                print("save done")
    
    
    def local_pose_callback(self, msg):
        pos = msg.pose.position
        pose_vec = np.array([pos.x,pos.y,pos.z],dtype=float)
        self.local_pose_vec = pose_vec


    # ENU的速度
    def local_vel_callback(self, msg):
        vel = msg.twist.linear
        vel_vec = np.array([vel.x,vel.y,vel.z],dtype=float)
        self.local_vel_vec = vel_vec




if __name__ == '__main__':
    saver = MavrosSaver()
    saver.start()

