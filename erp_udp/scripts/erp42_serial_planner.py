# -*- coding: utf-8 -*-
from lib.morai_udp_parser import erp_udp_parser,erp_udp_sender
from lib.serial_node import erpSerial
from lib.utils import pathReader,findLocalPath,purePursuit,Point,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
import time
import threading
from math import cos,sin,sqrt,pow,atan2,pi
import os,json

path = os.path.dirname(os.path.abspath( __file__ ))

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]

user_ip = params["user_ip"]
host_ip = params["host_ip"]
user_serial_port=params["user_serial_port"]

status_port =params["vehicle_status_dst_port"]
object_port =params["object_info_dst_port"]
get_traffic_port=params["get_traffic_dst_port"]

set_traffic_port=params["set_traffic_host_port"]
ctrl_cmd_port = params["ctrl_cmd_host_port"]

traffic_greenlight_setting= params["traffic_greenlight_setting"]

planner_path_file_name = params["planner_path_file_name"]

class erp_planner():
    def __init__(self):

        #subscriber
        self.status=erp_udp_parser(user_ip, status_port,'status')
        self.obj=erp_udp_parser(user_ip, object_port,'obj')
        self.traffic=erp_udp_parser(user_ip, get_traffic_port,'get_traffic')

        self.erp_serial=erpSerial(user_serial_port)
        self.set_traffic=erp_udp_sender(host_ip,set_traffic_port,'set_traffic')
        
        #read path
        self.txt_reader=pathReader()
        self.global_path=self.txt_reader.read(planner_path_file_name)

        #def
        self.is_status=False
        self.is_obj=False
        self.is_traffic=False
        self.traffic_info = [[58.50, 1180.41 ,'C119BS010001'],
                             [85.61, 1227.88 ,'C119BS010021'],
                             [136.58,1351.98 ,'C119BS010026'],
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

        #class
        self.pure_pursuit=purePursuit()
        self.cc=cruiseControl(0.5,1)
        self.vo=vaildObject(self.traffic_info)
        self.pid=pidController() ## pidController import

        vel_planner=velocityPlanning(200/3.6,1.5)
        self.vel_profile=vel_planner.curveBasedVelocity(self.global_path,100)


        while not self.is_status :
            if not self.status.get_data() :
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else :
                self.is_status=True

        self.main_loop()

    def main_loop(self):
        lattice_current_lane=3
        self.timer=threading.Timer(0.33,self.main_loop)
        self.timer.start()
        self.pure_pursuit=purePursuit()
        status_data=self.status.get_data()
        
        obj_data=self.obj.get_data()
        
        traffic_data = self.traffic.get_data()
        position_x=status_data[0]
        position_y=status_data[1]
        position_z=status_data[2]
        heading=status_data[5]+90   # degree
        velocity=status_data[6]

        #set trafficlight (green)
        if not len(traffic_data) == 0 and traffic_greenlight_setting == "True":
            self.set_traffic.send_data([False,traffic_data[1],16])
            traffic_data[3]=16            

        #fine_local_path, waypoint
        local_path,current_waypoint=findLocalPath(self.global_path,position_x,position_y) ##
        
        ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
        self.vo.get_object(obj_data)
        global_obj,local_obj=self.vo.calc_vaild_obj([position_x,position_y,heading])

        ########################  lattice  ########################
        vehicle_status=[position_x,position_y,heading,velocity]
        lattice_path,selected_lane=latticePlanner(local_path,global_obj,vehicle_status,lattice_current_lane)
        lattice_current_lane=selected_lane
        
        if selected_lane != -1: #and selected_lane != 5  :
            local_path=lattice_path[selected_lane]
        ########################  lattice  ########################
        
        if not len(traffic_data) == 0:            
            self.cc.checkObject(local_path,global_obj,local_obj,[traffic_data[1],traffic_data[3]]) 
        else:
            self.cc.checkObject(local_path,global_obj,local_obj) 
        
                
        #pure_pursuit. get_steering_angle_value
        self.pure_pursuit.getPath(local_path)
        self.pure_pursuit.getEgoStatus(position_x,position_y,position_z,velocity,heading)

        steering_angle=self.pure_pursuit.steering_angle()

        #ACC 
        cc_vel = self.cc.acc(local_obj,velocity,self.vel_profile[current_waypoint])
        target_velocity = cc_vel

        control_input=self.pid.pid(target_velocity, velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
        if control_input > 0 :
            accel= control_input
            brake= 0
        else :
            accel= 0            
            brake= -control_input

   

        self.erp_serial.send_ctrl_cmd(target_velocity,steering_angle)        

        self.print_info(status_data,obj_data,traffic_data,position_x,position_y,position_z,heading,velocity,steering_angle,current_waypoint)
        


    def print_info(self,status_data,obj_data,traffic_data,position_x,position_y,position_z,heading,velocity,steering_angle,current_waypoint):

        os.system('clear')
        print('--------------------status-------------------------')
        print('position :{0} ,{1}, {2}'.format(position_x,position_y,position_z))
        print('velocity :{} km/h'.format(velocity,heading))
        print('heading :{} deg'.format(heading-90))

        print('--------------------object-------------------------')
        print('object num :{}'.format(len(obj_data)))
        for i,obj_info in enumerate(obj_data) :
            print('{0} : type = {1}, x = {2}, y = {3}, z = {4} '.format(i,obj_info[0],obj_info[1],obj_info[2],obj_info[3]))

        print('--------------------controller-------------------------')
        print('target steering_angle :{} deg'.format(steering_angle))

        print('--------------------localization-------------------------')
        print('all waypoint size: {} '.format(len(self.global_path.poses)))
        print('current waypoint : {} '.format(current_waypoint))

        print('--------------------trafficLight-------------------------')
        if len(traffic_data) ==4:
            print('traffic mode : {}'.format(traffic_data[0]))
            print('traffic index : {}'.format(traffic_data[1]))
            print('traffic type : {}'.format(traffic_data[2]))
            print('traffic status : {}'.format(traffic_data[3]))
    
if __name__ == '__main__':
    kcity_pathtracking=erp_planner()
    while True:
        pass








