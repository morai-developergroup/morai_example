import cv2
import numpy as np
import os
import socket
import struct
import threading
class UDP_CAM_Parser:

    def __init__(self, ip, port, params_cam=None):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (ip,port)
        self.sock.bind(recv_address)

        print("connected")

        self.data_size=int(65000)
        
        # the steps while checking how many blocks need to complete jpg
        self.ready_step = int(10)
        self.check_max_len()
        self.raw_img=None
        self.is_img=False
        thread = threading.Thread(target=self.loop)
        thread.daemon = True 
        thread.start() 

    def loop(self):
        while True:
            self.raw_img=self.recv_udp_data()
            self.is_img=True

    def check_max_len(self):

        idx_list = b''

        r_step = 0

        while r_step<self.ready_step:
            
            UnitBlock, sender = self.sock.recvfrom(self.data_size)

            idx_list+=UnitBlock[3:7]

            r_step+=1



    def recv_udp_data(self):

        TotalBuffer = b''
        num_block = 0

        while True:

            UnitBlock, sender = self.sock.recvfrom(self.data_size)
        
            UnitIdx = struct.unpack('i',UnitBlock[3:7])[0]
            UnitSize = struct.unpack('i',UnitBlock[7:11])[0]
            UnitTail = UnitBlock[-2:]
                
            if num_block==UnitIdx:
                TotalBuffer+=UnitBlock[11:(11 + UnitSize)]
                num_block+=1   
            if UnitTail==b'EI' and num_block==UnitIdx+1:

                TotalIMG = cv2.imdecode(np.fromstring(TotalBuffer, np.uint8), 1)
                self.img_byte = np.array(cv2.imencode('.jpg', TotalIMG)[1]).tostring()
             


                TotalBuffer = b''

                break

        return TotalIMG
        
    def __del__(self):
        self.sock.close()
        print('del')

