#!/usr/bin/env python
# -*- coding: utf-8 -*-
from lib.morai_udp_parser import udp_parser,udp_sender
from lib.utils import pathReader,findLocalPath,purePursuit,Point,cruiseControl,vaildObject,velocityPlanning,pidController
import time
import threading
from math import cos,sin,sqrt,pow,atan2,pi
import os,json


path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]
user_ip = params["user_ip"]
host_ip = params["host_ip"]


status_port =params["vehicle_status_dst_port"]
object_port =params["object_info_dst_port"]
get_traffic_port=params["get_traffic_dst_port"]

set_traffic_port=params["set_traffic_host_port"]
ctrl_cmd_port = params["ctrl_cmd_host_port"]

planner_path_file_name = params["planner_path_file_name"]

traffic_greenlight_setting= params["traffic_greenlight_setting"]


class planner :

    def __init__(self):
        self.status=udp_parser(host_ip, status_port,'status')
        self.obj=udp_parser(host_ip, object_port,'obj')
        self.traffic=udp_parser(host_ip, get_traffic_port,'get_traffic')


        self.ctrl_cmd=udp_sender(user_ip,ctrl_cmd_port,'ctrl_cmd')
        self.set_traffic=udp_sender(user_ip,set_traffic_port,'set_traffic')
        

        self.txt_reader=pathReader()
        self.global_path=self.txt_reader.read(planner_path_file_name)

        self.traffic_info = [[58.50, 1180.41 ,'C119BS010001'],
                             [85.61, 1227.88 ,'C119BS010021'],
                             [136.58,1351.98 ,'C119BS010025'],
                             [141.02,1458.27 ,'C119BS010028'],
                             [139.39,1596.44 ,'C119BS010033'],
                             [48.71, 1208.02 ,'C119BS010005'],
                             [95.58, 1181.56 ,'C119BS010047'],
                             [104.46,1161.46 ,'C119BS010046'],
                             [85.29, 1191.77 ,'C119BS010007'],
                             [106.32,1237.04 ,'C119BS010022'],
                             [75.34, 1250.43 ,'C119BS010024'],
                             [73.62, 1218.01 ,'C119BS010012'],
                             [116.37,1190.65 ,'C119BS010040'],
                             [153.98,1371.48 ,'C119BS010073'],
                             [129.84,1385.08 ,'C119BS010039'],
                             [116.28,1367.77 ,'C119BS010074'],
                             [75.08, 1473.34 ,'C119BS010075'],
                             [67.10, 1506.66 ,'C119BS010076'],
                             [114.81,1485.81 ,'C119BS010079'],
                             [159.11,1496.63 ,'C119BS010060'],
                             [122.24,1608.26 ,'C119BS010072'],
                             [132.70,1624.78 ,'C119BS010034']]
        

        vel_planner=velocityPlanning(60,2) ## 속도 계획
        self.vel_profile=vel_planner.curveBasedVelocity(self.global_path,100)        

        self.pure_pursuit=purePursuit()
        self.cc=cruiseControl(1.0,0.5) ## cruiseControl import (object_vel_gain, object_dis_gain)
        self.vo=vaildObject(self.traffic_info) ## 장애물 유무 확인 (TrafficLight)
        self.pid=pidController() ## pidController import         

        self._is_status=False
        while not self._is_status :
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self._is_status=True
                
        self.main_loop()                
        # while True:
        #     self.main_loop()
        #     time.sleep(0.1)
        
    
    def main_loop(self):
        
        
        self.timer=threading.Timer(0.1,self.main_loop)
        self.timer.start()
        
        status_data=self.status.get_data()
        obj_data=self.obj.get_data()
        traffic_data = self.traffic.get_data()
        position_x=status_data[12]
        position_y=status_data[13]
        position_z=status_data[14]
        heading=status_data[17]# degree
        velocity=status_data[18]

        #set trafficlight (green)
        if not len(traffic_data) == 0 and traffic_greenlight_setting == "True": #set trafficlight (green)                        
            self.set_traffic.send_data([False,traffic_data[0],16])
            traffic_data[2]=16

        #fine_local_path, waypoint
        local_path,current_waypoint =findLocalPath(self.global_path,position_x,position_y)
        
        ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
        self.vo.get_object(obj_data)
        global_obj,local_obj=self.vo.calc_vaild_obj([position_x,position_y,(heading)/180*pi])

        if not len(traffic_data) == 0:
            self.cc.checkObject(local_path,global_obj,local_obj,traffic_data[0],traffic_data[2])
        else:
            self.cc.checkObject(local_path,global_obj,local_obj,[],[])         

        #pure_pursuit. get_steering_angle_value                
        self.pure_pursuit.getPath(local_path) 
        self.pure_pursuit.getEgoStatus(position_x,position_y,position_z,velocity,heading)
            
        steering_angle=self.pure_pursuit.steering_angle()

        #ACC                 
        cc_vel = self.cc.acc(local_obj,velocity,self.vel_profile[current_waypoint]) ## advanced cruise control 적용한 속도 계획        
        target_velocity = cc_vel
              
        control_input=self.pid.pid(target_velocity,velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
        if control_input > 0 :
            accel= control_input
            brake= 0
        else :
            accel= 0
            brake= -control_input

        ctrl_mode = 2 # 2 = AutoMode / 1 = KeyBoard
        Gear = 4 # 4 1 : (P / parking ) 2 (R / reverse) 3 (N / Neutral)  4 : (D / Drive) 5 : (L)
        cmd_type = 1 # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration
        
        send_velocity = 0 #cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = 0 #cmd_type이 3일때 원하는 가속도를 넣어준다.

        
        self.ctrl_cmd.send_data([ctrl_mode,Gear,cmd_type,send_velocity,acceleration,accel,brake,steering_angle])

        ######PRINT_INFO#######
        self.print_info(status_data,obj_data,traffic_data,position_x,position_y,position_z,heading,velocity,steering_angle,current_waypoint,target_velocity,accel,brake)


    def print_info(self,status_data,obj_data,traffic_data,position_x,position_y,position_z,heading,velocity,steering_angle,current_waypoint,target_velocity,accel,brake):
        os.system('cls')
        print('--------------------status-------------------------')
        print('ctrl Mode : {}'.format(status_data[0]))
        print('Gear : {}'.format(status_data[1]))
        print('signed velocity : {}'.format(status_data[2]))
        print('Map data id : {}'.format(status_data[3]))
        print('position : {0} ,{1}, {2}'.format(position_x,position_y,position_z))
        print('velocity : {} km/h'.format(velocity,heading))
        print('heading : {} deg'.format(heading-90))

        print('--------------------object-------------------------')
        print('object num : {}'.format(len(obj_data)))
        for i,obj_info in enumerate(obj_data) :
            print('id[{0}] : type = {1}, x = {2}, y = {3}, z = {4} '.format(obj_info[0],obj_info[1],obj_info[1],obj_info[2],obj_info[3]))

        print('--------------------controller-------------------------')
        print('target steering_angle : {} deg'.format(steering_angle))
        print('target target_velocity : {} km/h'.format(target_velocity))
        print('target accel : {} '.format(accel))
        print('target brake : {} '.format(brake))

        print('--------------------localization-------------------------')
        print('all waypoint size : {} '.format(len(self.global_path)))
        print('current waypoint : {} '.format(current_waypoint))

        print('--------------------trafficLight-------------------------')
        if len(traffic_data) ==3:
            print('traffic index : {}'.format(traffic_data[0]))
            print('traffic type : {}'.format(traffic_data[1]))
            print('traffic status : {}'.format(traffic_data[2]))

if __name__ == "__main__":
    kicty=planner()
 








