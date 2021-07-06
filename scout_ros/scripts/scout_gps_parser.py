#!/usr/bin/env python
 
import rospy
import numpy as np
import tf
import math
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from scout_msgs.msg import ScoutStatus
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from math import sqrt

def proj_coef_0(e):
    c0_transverse_mercator = np.array([
        [ -175 / 16384.0, 0.0,  -5 / 2560.0, 0.0, -3 / 64.0 , 0.0, -1 / 4.0, 0.0, 1.0],
        [ -105 / 40960.0, 0.0, -45 / 1024.0, 0.0, -3 / 32.0 , 0.0, -3 / 8.0, 0.0, 0.0],
        [  525 / 16384.0, 0.0,  45 / 1024.0, 0.0, 15 / 256.0, 0.0,      0.0, 0.0, 0.0],
        [ -175 / 12288.0, 0.0, -35 / 3072.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0],
        [ 315 / 131072.0, 0.0,          0.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0]
    ])

    c_out = np.zeros(5)

    for i in range(0,5):
        c_out[i] = np.poly1d(c0_transverse_mercator[i,:])(e)

    return c_out

def proj_coef_1(e):
    c0_transverse_mercator_reverse_coefficients = np.array([
        [    -175 / 16384.0, 0.0,   -5 / 256.0, 0.0,  -3 / 64.0, 0.0, -1 / 4.0, 0.0, 1.0 ],
        [       1 / 61440.0, 0.0,   7 / 2048.0, 0.0,   1 / 48.0, 0.0,  1 / 8.0, 0.0, 0.0 ],
        [    559 / 368640.0, 0.0,   3 / 1280.0, 0.0,  1 / 768.0, 0.0,      0.0, 0.0, 0.0 ],
        [    283 / 430080.0, 0.0, 17 / 30720.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0 ],
        [ 4397 / 41287680.0, 0.0,          0.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0 ]
    ])

    c_out = np.zeros(5)

    for i in range(0,5):
        c_out[i] = np.poly1d(c0_transverse_mercator_reverse_coefficients[i,:])(e)

    return c_out

    
def proj_coef_2(e):
    c0_merdian_arc = np.array([
        [ -175 / 16384.0    , 0.0, -5 / 256.0  , 0.0,  -3 / 64.0, 0.0, -1 / 4.0, 0.0, 1.0 ],
        [ -901 / 184320.0   , 0.0, -9 / 1024.0 , 0.0,  -1 / 96.0, 0.0,  1 / 8.0, 0.0, 0.0 ],
        [ -311 / 737280.0   , 0.0, 17 / 5120.0 , 0.0, 13 / 768.0, 0.0,      0.0, 0.0, 0.0 ],
        [ 899 / 430080.0    , 0.0, 61 / 15360.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0 ],
        [ 49561 / 41287680.0, 0.0,          0.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0 ]
    ])

    c_out = np.zeros(5)

    for i in range(0,5):
        c_out[i] = np.poly1d(c0_merdian_arc[i,:])(e)

    return c_out

class LocationSensor:
    def __init__(self, zone=52):
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.ego_sub = rospy.Subscriber("/scout_status",ScoutStatus, self.status_callback)
        
        self.vp_pub = rospy.Publisher("/Ego_GPS", EgoVehicleStatus, queue_size=8)
        
        self.zone = zone

        self.vehicle_p = EgoVehicleStatus()

        # rad to deg 
        self.D0 = 180 / np.pi

        # WGS84
        self.A1 = 6378137.0
        self.F1 = 298.257223563

        # Scale Factor
        self.K0 = 0.9996

        # False East & North 
        self.X0 = 500000
        if (self.zone > 0):
            self.Y0 = 0.0
        else:
            self.Y0 = 1e7

        # UTM origin latitude & longitude
        self.P0 = 0 / self.D0
        self.L0 = (6 * abs(self.zone) - 183) / self.D0
        
        # ellipsoid eccentricity
        self.B1 = self.A1 * (1 - 1 / self.F1)
        self.E1 = np.sqrt((self.A1**2 - self.B1**2) / (self.A1**2))
        self.N = self.K0 * self.A1

        # mercator transverse proj params
        self.C = np.zeros(5)
        self.C = proj_coef_0(self.E1)

        self.YS = self.Y0 - self.N * (
            self.C[0] * self.P0
            + self.C[1] * np.sin(2 * self.P0)
            + self.C[2] * np.sin(4 * self.P0)
            + self.C[3] * np.sin(6 * self.P0)
            + self.C[4] * np.sin(8 * self.P0))

        self.C2 = proj_coef_2(self.E1)

        self.rate = rospy.Rate(30)

        self.x, self.y, self.heading, self.velocity, self.gps_status = None, None, None, None, None

        self.x_old, self.y_old = 0, 0
        self.lat_old, self.lon_old = 0, 0
        self.heading_old = 0

        
    def convertLL2UTM(self, lat, lon):
        
        p1 = lat / self.D0  # Phi = Latitude(rad)
        l1 = lon / self.D0  # Lambda = Longitude(rad)

        es = self.E1 * np.sin(p1)
        L = np.log( np.tan(np.pi/4.0 + p1/2.0) * 
                    np.power( ((1 - es) / (1 + es)), (self.E1 / 2)))

        z = np.complex(
            np.arctan(np.sinh(L) / np.cos(l1 - self.L0)),
            np.log(np.tan(np.pi / 4.0 + np.arcsin(np.sin(l1 - self.L0) / np.cosh(L)) / 2.0))
        )        

        Z = self.N * self.C2[0] * z \
            + self.N * (self.C2[1] * np.sin(2.0 * z)
            + self.C2[2] * np.sin(4.0 * z)
            + self.C2[3] * np.sin(6.0 * z)
            + self.C2[4] * np.sin(8.0 * z))

        east = Z.imag + self.X0
        north = Z.real + self.YS

        return east, north

    def navsat_callback(self, gps_msg):
        
        lat = gps_msg.latitude
        lon = gps_msg.longitude

        e_o = gps_msg.eastOffset
        n_o = gps_msg.northOffset
        
        e_global, n_global = self.convertLL2UTM(lat, lon)
        
        self.x, self.y = e_global - e_o, n_global - n_o
        pi = lat * np.pi / 180
        pi2 = self.lat_old * np.pi / 180

        delta_pi = (self.lat_old - lat) * np.pi / 180
        delta_lmb = (self.lon_old - lon) * np.pi / 180


        dx = self.x - self.x_old
        dy = self.y - self.y_old
        dis=sqrt(dx*dx + dy*dy)
        if(dis < 0.1):
            self.heading = self.heading_old
        else:
            self.heading = math.atan2(self.y - self.y_old, self.x - self.x_old)
        self.gps_status = gps_msg.status
        self.lat_old , self.lon_old = lat , lon

    def status_callback(self, data):
        self.velocity = data.linear_velocity 

if __name__ == '__main__':
     
    rospy.init_node('gps_parser', anonymous=True)

    loc_sensor = LocationSensor()

    rate = rospy.Rate(30)
 
    while not rospy.is_shutdown():
    
        if loc_sensor.x is not None and loc_sensor.gps_status == 1 and loc_sensor.velocity is not None:
     
            loc_sensor.vehicle_p.position.x = loc_sensor.x
            loc_sensor.vehicle_p.position.y = loc_sensor.y
            loc_sensor.vehicle_p.position.z = 0
            loc_sensor.vehicle_p.heading = loc_sensor.heading * 180 / np.pi
            loc_sensor.vehicle_p.velocity.x = loc_sensor.velocity

            loc_sensor.x_old, loc_sensor.y_old = loc_sensor.x, loc_sensor.y
            loc_sensor.heading_old = loc_sensor.heading

            loc_sensor.vp_pub.publish(loc_sensor.vehicle_p)


        elif loc_sensor.x is not None and loc_sensor.gps_status != 1:
     
            loc_sensor.vehicle_p.position.x = 0
            loc_sensor.vehicle_p.position.y = 0
            loc_sensor.vehicle_p.position.z = 0
            loc_sensor.vehicle_p.heading = 0

            loc_sensor.vp_pub.publish(loc_sensor.vehicle_p)

        else:
            pass
        
        rate.sleep()
