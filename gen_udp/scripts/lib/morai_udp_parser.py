import socket
import threading
import time
import struct
class udp_parser :
    def __init__(self,ip,port,data_type):
        self.data_type=data_type
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (ip,port)
        self.sock.bind(recv_address)
        self.data_size=65535 
        self.parsed_data=[]
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

    

    def recv_udp_data(self):
        while True :
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)




    def data_parsing(self,raw_data) :
        if self.data_type == 'status' :
            header=raw_data[0:11].decode()
            data_length=struct.unpack('i',raw_data[11:15])

                            
            if header == '#MoraiInfo$' and data_length[0] ==32:
                vgen_ctrl_cmd = struct.unpack('b',raw_data[15:16])
                vgen_gear = struct.unpack('b',raw_data[16:17])
                unpacked_data_1 = struct.unpack('fi',raw_data[17:25])
                unpacked_data_2 = struct.unpack('ffffffff',raw_data[27:59])
                unpacked_data = vgen_ctrl_cmd + vgen_gear + unpacked_data_1 + unpacked_data_2
                # unpacked_data=struct.unpack('ffffffff',raw_data[27:59])
                self.parsed_data=list(unpacked_data)  
           
            
        elif self.data_type == 'obj' :
            
            header=raw_data[0:14].decode()            
            if header == '#MoraiObjInfo$' :
                unpacked_data=[]
                offset_byte=30
                
                for i in range(20) :
                    start_byte=i*34
                    obj_type=struct.unpack('h',raw_data[start_byte+offset_byte:start_byte+offset_byte+2])
                    obj_info=struct.unpack('8f',raw_data[start_byte+offset_byte+2:start_byte+offset_byte+34])
                    obj_info_list=list(obj_info)
                    obj_info_list.insert(0,obj_type[0])
                    if not(obj_info_list[0] == 0 and obj_info_list[1] == 0 and obj_info_list[2] == 0) :
                        unpacked_data.append(obj_info_list)
                    
             
                if len(obj_info_list) !=0 :
                    self.parsed_data=unpacked_data
                else :
                    self.parsed_data=[]      

        elif self.data_type == 'get_traffic' :
            
            header=raw_data[0:14].decode()
            data_length=struct.unpack('i',raw_data[14:18])

            # print("raw_data = ", raw_data)
            
            if header == '#TrafficLight$' and data_length[0]==17 :
                # auto_mode=struct.unpack('?',raw_data[30])
                auto_mode=raw_data[30]
                traffic_index=raw_data[31:43].decode()
                traffic_type,traffic_status=struct.unpack('2h',raw_data[43:47])

                self.parsed_data=[auto_mode,traffic_index,traffic_type,traffic_status]
                
       
                

    def get_data(self) :
        return self.parsed_data

    def __del__(self):
        self.sock.close()        
        print('del')


class udp_sender :
    def __init__(self,ip,port,data_type):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip=ip
        self.port=port
        self.data_type=data_type

        if self.data_type=='ctrl_cmd':  
            header='#MoraiCtrlCmd$'.encode()
            data_length=struct.pack('i',12)
            # aux_data=struct.pack('iii',0,0,0)
            self.upper=header+data_length # +aux_data
            self.tail='\r\n'.encode()  
        
        elif self.data_type == 'set_traffic':
            header='#TrafficLight$'.encode()
            data_length=struct.pack('i',15)
            aux_data=struct.pack('iii',0,0,0)
            self.upper=header+data_length+aux_data
            self.tail='\r\n'.encode()     

        elif self.data_type == 'multi_ego':
            header='#MultiEgoSetting$'.encode()
            data_length=struct.pack('i',920)
            aux_data=struct.pack('iii',0,0,0)
            self.upper=header+data_length+aux_data
            self.tail='\r\n'.encode()        

        elif self.data_type == 'scenario':
            header=struct.pack('III',77,79,82)
            
            self.upper=header
            self.tail=struct.pack('II',65,73)

    def send_data(self,data):
        
        if self.data_type=='ctrl_cmd':  
            packed_mode=struct.pack('b',data[0])
            packed_gear=struct.pack('b',data[1])
            aux_data1=struct.pack('h',0)
            aux_data2=struct.pack('ii',0,0)
            packed_accel=struct.pack('f',data[2])
            packed_brake=struct.pack('f',data[3])
            packed_steering_angle=struct.pack('f',data[4])
            lower=packed_mode+packed_gear+aux_data1+aux_data2+packed_accel+packed_brake+packed_steering_angle
            send_data=self.upper+lower+self.tail
            # print(len(send_data),send_data)

        elif self.data_type == 'set_traffic':            
            packed_automode=struct.pack('?',data[0])
            packed_traffic_index=data[1].encode()
            packed_traffic_status=struct.pack('h',data[2])
            lower=packed_automode+packed_traffic_index+packed_traffic_status
            send_data=self.upper+lower+self.tail

        elif self.data_type == 'multi_ego':
            
            num_of_ego=len(data)
            camera_index=0
            packed_num_of_ego=struct.pack('i',num_of_ego)
            packed_camera_index=struct.pack('i',camera_index)
            lower=None
            for ego in range(20) :
                if ego <len(data):
                    print(ego,data[ego][0],data[ego][1],data[ego][2])                    
                    ego_index=struct.pack('i',data[ego][0])
                    status_data=struct.pack('3dffffBB',data[ego][1],data[ego][2],data[ego][3],data[ego][4],data[ego][5],data[ego][6],data[ego][7],data[ego][8],data[ego][9])
                    pack_data=ego_index+status_data       
                    
                else:
                    ego_index=struct.pack('i',0)
                    status_data=struct.pack('3dffffBB',0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0)
                    pack_data=ego_index+status_data            

                if lower==None :
                    lower=pack_data
                else :
                    lower+=pack_data

            send_data=self.upper+packed_num_of_ego+packed_camera_index+lower              
        
        elif self.data_type=='scenario':
            packed_data=struct.pack('IIIII',data[0],data[1],data[2],data[3],data[4])
            lower=packed_data
            send_data=self.upper+lower+self.tail
            print(len(send_data),send_data)

        self.sock.sendto(send_data,(self.ip,self.port))


        
        

      

