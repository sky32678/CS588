
from __future__ import print_function

# Python Headers
import os
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

def inspva_callback(self, inspva_msg):
    global waypoints
    lat2     = inspva_msg.latitude  # latitude
    lon2     = inspva_msg.longitude # longitude
    heading2 = inspva_msg.azimuth   # heading in degrees
    print(lat2, lon2, heading2)
    waypoints.append([lat2, lon2, heading2])


if __name__ == '__main__':
    rospy.init_node('gnss_pp_node', anonymous=True)
    global waypoints
    waypoints = []

    gnss_sub = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)
    while not rospy.is_shutdown():
        print("COLLECTING DATASET")
    np.savetxt("GFG.csv",
           waypoints,
           delimiter =", ",
           fmt ='% s')
