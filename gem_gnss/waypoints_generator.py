from __future__ import print_function

# Python Headers
import os
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
import time

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

def wps_to_local_xy(lon_wp, lat_wp):
    # convert GNSS waypoints into local fixed frame reprented in x and y
    lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, 40.0928563, -88.2359994)
    return lon_wp_x, lat_wp_y

def heading_to_yaw(heading_curr):
    # 0   <= heading < 90  --- 90 to 0     (pi/2 to 0)
    # 90  <= heading < 180 --- 0 to -90    (0 to -pi/2)
    # 180 <= heading < 270 --- -90 to -180 (-pi/2 to -pi)
    # 270 <= heading < 360 --- 180 to 90   (pi to pi/2)
    if (heading_curr >= 0 and heading_curr < 90):
        yaw_curr = np.radians(90 - heading_curr)
    elif(heading_curr >= 90 and heading_curr < 180):
        yaw_curr = np.radians(90 - heading_curr)
    elif(heading_curr >= 180 and heading_curr < 270):
        yaw_curr = np.radians(90 - heading_curr)
    else:
        yaw_curr = np.radians(450 - heading_curr)
    return yaw_curr

def inspva_callback(inspva_msg):
    global waypoints
    lat2     = inspva_msg.latitude  # latitude
    lon2     = inspva_msg.longitude # longitude
    heading2 = inspva_msg.azimuth   # heading in degrees
    
    lon2, lat2 = wps_to_local_xy(lon2, lat2)
    # heading2 = heading_to_yaw(heading2)
    print(lon2, lat2, heading2)
    waypoints.append([lon2, lat2 , heading2])
    # rospy.sleep0.1)


if __name__ == '__main__':
    rospy.init_node('gnss_pp_node', anonymous=True)
    global waypoints
    waypoints = []

    gnss_sub = rospy.Subscriber("/novatel/inspva", Inspva, inspva_callback)
    while not rospy.is_shutdown():
        #print(waypoints)
        time.sleep(1.5)        
        pass
    np.savetxt("right_bottom.csv",
           waypoints,
           delimiter =", ",
           fmt ='% s')
