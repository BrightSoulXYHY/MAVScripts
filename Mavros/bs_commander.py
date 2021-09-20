#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Date    : 2020-03-22 22:40:44
# @Author  : BrightSoul (653538096@qq.com)



import time
import readline
start_time = time.time()




# from bs_px4_mavros import Px4Controller
from bs_px4_mavrosv2 import Px4Controller

def test_bs_mavros():
    # mav_id = 0
    # con = Px4Controller(mav_id,homes[mav_id])
    con = Px4Controller()
    con.start()
    while True:
        str_cdmd = input(">")
        L = str_cdmd.split(" ")
        if(L[0] == ""):
            continue
        else:
            cmd = L.pop(0)
            try:
                con.decode(cmd, L)
            except KeyError as e:
                print("no command:",e)
                print("support cmd:")
                for key, value in con.cmdmap.items() :
                    print(key,end=' ')
                print()


if __name__ == '__main__':
    test_bs_mavros()
