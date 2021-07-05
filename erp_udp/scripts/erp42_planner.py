#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import numpy as np
from lib.morai_udp_parser import udp_parser,udp_sender
from lib.utils import pathReader,findLocalPath,purePursuit,Point
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json


path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]

user_ip = params["user_ip"]
host_ip = params["host_ip"]



class planner :

    def __init__(self):
        self.status=udp_parser(user_ip, params["vehicle_status_dst_port"],'erp_status')
        self.obj=udp_parser(user_ip, params["object_info_dst_port"],'erp_obj')
        self.traffic=udp_parser(user_ip, params["get_traffic_dst_port"],'get_traffic')


        

        self.ctrl_cmd=udp_sender(host_ip,params["ctrl_cmd_host_port"],'erp_ctrl_cmd')
        self.set_traffic=udp_sender(host_ip,params["set_traffic_host_port"],'set_traffic')
  

        self.txt_reader=pathReader()
        self.global_path=self.txt_reader.read('kcity.txt')

        self.pure_pursuit=purePursuit()
  

        self._is_status=False
        while not self._is_status :
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self._is_status=True


        self.main_loop()


    
    def main_loop(self):
        self.timer=threading.Timer(0.01,self.main_loop)
        self.timer.start()
        
        status_data=self.status.get_data()
        # obj_data=self.obj.get_data()
        traffic_data=self.traffic.get_data()

        position_x=status_data[0]
        position_y=status_data[1]
        position_z=status_data[2]
        heading=status_data[5]  # degree
        velocity=status_data[6]




        local_path,current_point =findLocalPath(self.global_path,position_x,position_y)

        self.pure_pursuit.getPath(local_path)
        self.pure_pursuit.getEgoStatus(position_x,position_y,position_z,velocity,heading)


        steering_angle=self.pure_pursuit.steering_angle() # deg
        accel=1
        brake=0

        self.ctrl_cmd.send_data([accel,brake,steering_angle])
        if not len(traffic_data) == 0 :
            self.set_traffic.send_data([False,traffic_data[1],16])

      

if __name__ == "__main__":


    kicty=planner()
    while True :
        pass
 





