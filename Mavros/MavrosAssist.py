import rospy
from mavros_msgs.msg import PositionTarget
import numpy as np



rad = np.pi / 180.0
deg = 180 / np.pi


class MavrosAssist:
    # xyz in meter yaw in rad
    # 构造位置目标ENU系
    def construct_postarget(self, x, y, z, yaw, yaw_rate = 1):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()
        
        '''
        uint8 FRAME_LOCAL_NED = 1
        uint8 FRAME_LOCAL_OFFSET_NED = 7
        uint8 FRAME_BODY_NED = 8
        uint8 FRAME_BODY_OFFSET_NED = 9

        MAV_FRAME_BODY_FRD
        '''

        # 全部都给NED就ok,读取回来的也是NED坐标系下的
        target_raw_pose.coordinate_frame = 1

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose  
  
    # 构造FLU的速度目标
    def construct_veltarget(self, vx, vy, vz, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()
        
        '''
        uint8 FRAME_LOCAL_NED = 1
        uint8 FRAME_LOCAL_OFFSET_NED = 7
        uint8 FRAME_BODY_NED = 8
        uint8 FRAME_BODY_OFFSET_NED = 9
        '''
        target_raw_pose.coordinate_frame = 8

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz

        target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE + PositionTarget.IGNORE_YAW 

        target_raw_pose.yaw_rate = yaw_rate
        return target_raw_pose
    
    def construct_veltarget_ENU(self, vx, vy, vz, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()
        
        '''
        uint8 FRAME_LOCAL_NED = 1
        uint8 FRAME_LOCAL_OFFSET_NED = 7
        uint8 FRAME_BODY_NED = 8
        uint8 FRAME_BODY_OFFSET_NED = 9
        '''
        # 直接就ENU了
        target_raw_pose.coordinate_frame = 1

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz

        target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE + PositionTarget.IGNORE_YAW 

        target_raw_pose.yaw_rate = yaw_rate
        return target_raw_pose
    

    def construct_yawratetarget(self, yaw_rate):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()
        
        '''
        uint8 FRAME_LOCAL_NED = 1
        uint8 FRAME_LOCAL_OFFSET_NED = 7
        uint8 FRAME_BODY_NED = 8
        uint8 FRAME_BODY_OFFSET_NED = 9
        '''
        target_raw_pose.coordinate_frame = 9
        target_raw_pose.position = self.local_pose.pose.position

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.IGNORE_YAW 

        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose  
    
    def construct_acceltarget(self, ax, ay, az):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()
        
        '''
        uint8 FRAME_LOCAL_NED = 1
        uint8 FRAME_LOCAL_OFFSET_NED = 7
        uint8 FRAME_BODY_NED = 8
        uint8 FRAME_BODY_OFFSET_NED = 9
        '''
        target_raw_pose.coordinate_frame = 9
        target_raw_pose.position = self.local_pose.pose.position

        target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                    + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW 

        target_raw_pose.acceleration_or_force.x = ax
        target_raw_pose.acceleration_or_force.y = ay
        target_raw_pose.acceleration_or_force.z = az


        return target_raw_pose  

    # FLU顺时针转yaw到ENU,
    # yaw为0的时候朝E
    def FLU2ENU(self, x,y,z):
        '''顺时针因而取反
        | cos   -sin |
        | sin    cos |
        '''
        ENU_x = x * np.cos(self.current_heading) - y * np.sin(self.current_heading)
        ENU_y = x * np.sin(self.current_heading) + y * np.cos(self.current_heading)
        ENU_z = z

        return ENU_x, ENU_y, ENU_z

    def q2yaw(self, q):
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        # return math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        return np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

    # constrain_deg -360~720 to 0~360
    def constrain_deg(self,deg):
        if deg >= 360:
            return (deg - 360)        
        elif deg <= 0:
            return (deg + 360)
        else:
            return deg
    # constrain_deg  to -180~10
    def constrain_deg180(self,deg):
        if deg >= 180:
            return (deg - 360)        
        elif deg <= -180:
            return (deg + 360)
        else:
            return deg
