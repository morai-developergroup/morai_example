#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from morai_msgs.msg  import GPSMessage
from math import pi,cos,sin,pi,sqrt,pow
from nav_msgs.msg import Path
import tf
from geometry_msgs.msg import PoseStamped
import pyproj

class test :

    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_folder_name=arg[1]
        self.make_path_name=arg[2]
        self.x_offset=float(arg[3])
        self.y_offset=float(arg[4])

        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)

        self.is_gps = False
        self.prev_x = 0
        self.prev_y = 0

        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('scout_ros')
        full_path=pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f=open(full_path, 'w')

        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_gps == True :
                self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
        x=self.xy_zone[0]- self.x_offset
        y=self.xy_zone[1]- self.y_offset
        z=0
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        
        if distance > 0.5:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            print(x,y)


    def gpsCB(self, data):
        self.xy_zone = self.proj_UTM(data.longitude, data.latitude)
        self.is_gps = True
        

if __name__ == '__main__':
    try:
        test_track=test()
    except rospy.ROSInterruptException:
        pass
