import airsim
import numpy as np

client = airsim.MultirotorClient()

# easy
# 1 RandomCirccle_BP2_5
# 2 RandomCirccle_BP3_8
# 3 RandomCirccle_BP5_12
# 4 RandomCirccle_BP6_15
# 5 RandomCirccle_BP7_18
# 6 RandomCirccle_BP8_21
# 7 RandomCirccle_BP9

# hard
# 1 Forest_RandomCircle_BP_5
# 2 Forest_RandomCircle_BP2_8
# 3 Forest_RandomCircle_BP3_11
# 4 MovLN_BP2_2
# 5 MovCircle_BP_3
# 6 MovBump_BP_2
# ROTATECOL_BP2_2
# 
easyCircleNameL = [
    "RandomCirccle_BP2_5",
    "RandomCirccle_BP3_8",
    "RandomCirccle_BP5_12",
    "RandomCirccle_BP6_15",
    "RandomCirccle_BP7_18",
    "RandomCirccle_BP8_21",
    "RandomCirccle_BP9",
]



pose = client.simGetObjectPose("RandomCirccle_BP5_12")
# pose = client.simGetObjectPose("MovLN_BP2_2")
print(pose)
# mav_state = client.getMultirotorState()
# cnt_pos = mav_state.kinematics_estimated.position.to_numpy_array()


# while True:
#     coll_info = client.simGetCollisionInfo()
#     if coll_info.has_collided:
#         mission_text = ""
        
#         mission_text += coll_info.object_name+","
#         mission_text += ("{:.2f},"*3).format(*coll_info.impact_point.to_numpy_array())

#         print(mission_text)
        # print(coll_info.object_id,coll_info.object_name)
        # print(coll_info.object_name) 
        # print(coll_info.impact_point.to_numpy_array()) 
        # cnt_pos = mav_state.kinematics_estimated.position.to_numpy_array()
        # print(cnt_pos)
        

# print(coll_info)
# end_pos = np.array([-62.9,-102,-3.0])

# print(np.linalg.norm(cnt_pos-end_pos))
# print(coll_info.has_collided)

# mav_state = client.getMultirotorState(self.vehicle_name)
# cnt_vel = mav_state.kinematics_estimated.linear_velocity.to_numpy_array()
# print(coll_info.object_id)
# print(coll_info.object_name)
# print(coll_info.impact_point.x_val)
# print(coll_info.time_stamp)
# print(coll_info.position.x_val)