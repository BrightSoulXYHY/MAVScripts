#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Date    : 2020-03-22 22:40:44
# @Author  : BrightSoul (653538096@qq.com)



import time
import readline
start_time = time.time()



# import json

# settings_file = open("C:/Users/BrightSoul/Documents/AirSim/settings.json")
# settings = json.load(settings_file)
# nums = len(settings["Vehicles"])
# homes = []
# for i in range(nums):
#     # vehicle_name = "Drone{}".format(i)
#     vehicle_name = "mav_{:02d}".format(i)
#     homes.append([settings["Vehicles"][vehicle_name]["X"],
#                   settings["Vehicles"][vehicle_name]["Y"],
#                   settings["Vehicles"][vehicle_name]["Z"]])



import airsim
from bs_as_api import AsController,client

client.confirmConnection()
def test_as():
    
    con = AsController()
    # con = AsController(1)
    
    con.start(start_time = start_time)
    while True: 
        str_cdmd = input(">")
        L = str_cdmd.split(" ")
        if(L[0] == ""):
            continue
        else:
            cmd = L.pop(0)
            try:
                # print(cmd, L)
                con.decode(cmd, L)
            except KeyError as e:
                print("no command:",e)
                print("support cmd:")
                for key, value in con.cmdmap.items() :
                    print(key,end=' ')
                print()




if __name__ == '__main__':
    test_as()