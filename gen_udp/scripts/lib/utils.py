#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from math import cos,sin,sqrt,pow,atan2,pi
import numpy as np

class Point() :
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class pathReader :
    def __init__(self):
       self.file_path=os.path.dirname( os.path.abspath( __file__ ) )
       self.file_path = os.path.normpath(os.path.join(self.file_path, '..'))     
 
 
    def read(self,file_name):
        full_file_name=self.file_path+"/../path/"+file_name
        openFile = open(full_file_name, 'r')

        out_path=[]

        line=openFile.readlines()

        for i in line :
            tmp=i.split()
            read_pose = []
            read_pose.append(float(tmp[0]))
            read_pose.append(float(tmp[1]))
            read_pose.append(float(tmp[2]))        
            out_path.append(read_pose)

        openFile.close()
        return out_path
        
## global_path와 차량의 x, y를 이용해 local_path를 생성 ##
def findLocalPath(ref_path,position_x,position_y):
    out_path=[]
    current_x=position_x
    current_y=position_y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path)) :
        dx=current_x - ref_path[i][0]
        dy=current_y - ref_path[i][1]
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i


    if current_waypoint+50 > len(ref_path) :
        last_local_waypoint= len(ref_path)
    else :
        last_local_waypoint=current_waypoint+50

    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=[]
        tmp_pose.append(ref_path[i][0])
        tmp_pose.append(ref_path[i][1])
        out_path.append(tmp_pose)

    return out_path,current_waypoint ## local_path와 waypoint를 반환 ##


class purePursuit :
    def __init__(self):
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=4
        self.lfd=5
        self.min_lfd=5
        self.max_lfd=30
        self.steering=0
        
    def getPath(self,path):
        self.path=path 
 
    
    
    def getEgoStatus(self,position_x,position_y,position_z,velocity,heading):

        self.current_vel=velocity  #kph
        self.vehicle_yaw=heading/180*pi   # rad
        self.current_postion.x=position_x
        self.current_postion.y=position_y
        self.current_postion.z=position_z




    def steering_angle(self):
        vehicle_position=self.current_postion
        rotated_point=Point()
        self.is_look_forward_point= False


        for i in self.path :
            dx= i[0] - vehicle_position.x
            dy= i[1] - vehicle_position.y
            rotated_point.x=cos(self.vehicle_yaw)*dx +sin(self.vehicle_yaw)*dy
            rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
 
            
            if rotated_point.x>0 :
                dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                
                if dis>= self.lfd :
                    
                    self.lfd=self.current_vel*0.3 # or 0.4
                    if self.lfd < self.min_lfd : 
                        self.lfd=self.min_lfd
                    elif self.lfd > self.max_lfd :
                        self.lfd=self.max_lfd
                    # self.forward_point=i
                    self.is_look_forward_point=True
                    
                    break
        
        theta=atan2(rotated_point.y,rotated_point.x)

        if self.is_look_forward_point :
            # self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi
            self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)
            return self.steering #deg
        else : 
            print("no found forward point")
            return 0
        


class velocityPlanning : #경로기반속도계획
    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed=car_max_speed
        self.road_friction=road_friction
 
    def curveBasedVelocity(self,global_path,point_num):
        out_vel_plan=[]
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num,len(global_path)-point_num):
            x_list=[]
            y_list=[]

            for box in  range(-point_num,point_num):
                x=global_path[i+box][0]
                y=global_path[i+box][1]
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))

            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T
            

            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)  #0.7
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path)-point_num,len(global_path)):
            out_vel_plan.append(self.car_max_speed)
        
        return out_vel_plan



class cruiseControl: ## ACC(advanced cruise control) 적용 ##
    def __init__(self,object_vel_gain,object_dis_gain):
        self.object=[False,0]
        self.traffic=[False,0]
        self.Person=[False,0]
        self.object_vel_gain=object_vel_gain
        self.object_dis_gain=object_dis_gain


    def checkObject(self,ref_path,global_vaild_object,local_vaild_object,traffic_data_1,traffic_data_3): ## 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호) ##
        self.object=[False,0]
        self.traffic=[False,0]
        self.Person=[False,0]
        if len(global_vaild_object) >0  :
            min_rel_distance=float('inf')
            for i in range(len(global_vaild_object)):
                for j in range(len(ref_path)) :
                    if global_vaild_object[i][0]==1 or global_vaild_object[i][0]==2:

                        dis=sqrt(pow(ref_path[j][0]-global_vaild_object[i][1],2)+pow(ref_path[j][1]-global_vaild_object[i][2],2))

                        if dis<2.5:
                            rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                            
                            if rel_distance < 30:
                                if rel_distance < min_rel_distance:
                                    min_rel_distance=rel_distance
                                    self.object=[True,i]
                            

                    if global_vaild_object[i][0]==0 :
                    
                        dis=sqrt(pow(ref_path[j][0]-global_vaild_object[i][1],2)+pow(ref_path[j][1]-global_vaild_object[i][2],2))

                        if dis<4.7: # 4.35
                            
                            rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                            
                            if rel_distance < 35:
                                if rel_distance < min_rel_distance:
                                    min_rel_distance=rel_distance
                                    self.Person=[True,i]


                    if global_vaild_object[i][0]==3 :
                        traffic_sign='STOP'

                        if len(traffic_data_1)!=0  and  global_vaild_object[i][3] == traffic_data_1 :
                            # if traffic_data_3 == 48 or traffic_data_3==16   :   #
                            if not traffic_data_3 ==  1 :   #
                                traffic_sign ='GO'
                        if traffic_sign =='STOP':
                            dis=sqrt(pow(ref_path[j][0]-global_vaild_object[i][1],2)+pow(ref_path[j][1]-global_vaild_object[i][2],2))
                            
                            if dis<2.5 :
                                rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                                if rel_distance < min_rel_distance and rel_distance < 23:
                                    min_rel_distance=rel_distance
                                    self.traffic=[True,i]                     
                    

    def acc(self,local_vaild_object,ego_vel,target_vel): ## advanced cruise control 를 이용한 속도 계획 ##
        out_vel=target_vel
        pre_out_vel = out_vel
        if self.object[0] == True :
            print("ACC ON_vehicle")   
            front_vehicle=[local_vaild_object[self.object[1]][1],local_vaild_object[self.object[1]][2],local_vaild_object[self.object[1]][3]]
            time_gap=0.8
            default_space=5
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-3
            
            vel_rel=(front_vehicle[2]-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel

            dx = front_vehicle[0]
            dy = front_vehicle[1]

            t_dis = sqrt(pow(dx,2)+pow(dy,2))

        if self.Person[0]==True:
            print("ACC ON_person")
            Pedestrian=[local_vaild_object[self.Person[1]][1],local_vaild_object[self.Person[1]][2],local_vaild_object[self.Person[1]][3]]
            time_gap=0.8
            default_space=9
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(Pedestrian[0],2)+pow(Pedestrian[1],2))-3
            
            vel_rel=(Pedestrian[2]-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel - 7
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel
            dx =  Pedestrian[0]
            dy =  Pedestrian[1]

            t_dis = sqrt(pow(dx,2)+pow(dy,2))


        if self.traffic[0] == True :
            print("Traffic_ON")   
            front_vehicle=[local_vaild_object[self.traffic[1]][1],local_vaild_object[self.traffic[1]][2],local_vaild_object[self.traffic[1]][3]]
            time_gap=0.8
            default_space=5
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-3
            
            vel_rel=(0-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel

            if dis_rel < 5 :
                out_vel = 0
        return out_vel     


class object_info :
    def __init__(self):
        self.num_of_objects=0
        self.pose_x=[]
        self.pose_y=[]
        self.velocity=[]
        self.object_type=[]
        
        


class vaildObject : ## 장애물 유무 확인 (차량, 사람, 정지선 신호) ##

    def __init__(self,stop_line=[]):
        self.stop_line=stop_line
    def get_object(self,obj_data):
        # print(obj_data)
        self.all_object=object_info()
        self.all_object.num_of_objects=len(obj_data)
        for i,obj_info in enumerate(obj_data) :
            self.all_object.object_type.append(obj_data[i][1])
            self.all_object.pose_x.append(obj_data[i][2])
            self.all_object.pose_y.append(obj_data[i][3])
            self.all_object.velocity.append(obj_data[i][12])
            

    def calc_vaild_obj(self,ego_pose):  # x, y, heading
        global_object_info=[]
        loal_object_info=[]
        tmp_theta=ego_pose[2]
        tmp_translation=[ego_pose[0],ego_pose[1]]
        tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                        [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                        [0,0,1]])
        tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],
                            [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],
                            [0,0,1]])

        for num in range(self.all_object.num_of_objects):
            # print(self.all_object.pose_x[num], self.all_object.pose_y[num])
            global_result=np.array([[self.all_object.pose_x[num]],[self.all_object.pose_y[num]],[1]])
            local_result=tmp_det_t.dot(global_result)
            if local_result[0][0]> 0 :
                global_object_info.append([self.all_object.object_type[num],self.all_object.pose_x[num],self.all_object.pose_y[num],self.all_object.velocity[num]])
                loal_object_info.append([self.all_object.object_type[num],local_result[0][0],local_result[1][0],self.all_object.velocity[num]])
        
        
        for line in self.stop_line:
            global_result=np.array([[line[0]],[line[1]],[1]])
            local_result=tmp_det_t.dot(global_result)
            if local_result[0][0]> 0 :
                global_object_info.append([3,line[0],line[1],line[2]])
                loal_object_info.append([3,local_result[0][0],local_result[1][0],line[2]])
        

        return global_object_info,loal_object_info

class pidController : ## 속도 제어를 위한 PID 적용 ##
    def __init__(self):
        self.p_gain=0.018
        self.i_gain=0.0
        self.d_gain=0.005
        self.controlTime=0.033
        self.prev_error=0
        self.i_control=0


    def pid(self,target_vel,current_vel):
        error= target_vel-current_vel
        
        p_control=self.p_gain*error
        self.i_control+=self.i_gain*error*self.controlTime
        d_control=self.d_gain*(error-self.prev_error)/self.controlTime

        output=p_control+self.i_control+d_control
        if target_vel == 0 :
            output = -1
        self.prev_error=error
        return output