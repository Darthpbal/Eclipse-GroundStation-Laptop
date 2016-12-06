# -*- coding: utf-8 -*-
"""
Created on Fri Aug 19 12:09:40 2016

@author: Ground Station
"""

import math
import numpy as np
import matplotlib.pyplot as plt

## function     :   bearing
## date         :   8.19.2016
## great circle bearing, see: http://www.movable-type.co.uk/scripts/latlong.html
## 
def bearing(trackerLat, trackerLon, remoteLat, remoteLon):
        dLat = math.radians(remoteLat-trackerLat)       # delta latitude in radians
        dLon = math.radians(remoteLon-trackerLon)       # delta longitude in radians
        
        y = math.sin(dLon)*math.cos(math.radians(remoteLat))
        x = math.cos(math.radians(trackerLat))*math.sin(math.radians(remoteLat))-math.sin(math.radians(trackerLat))*math.cos(math.radians(remoteLat))*math.cos(dLat)
        tempBearing = math.degrees(math.atan2(y,x))     # returns the bearing from true north

        if (tempBearing < 0):
                tempBearing = tempBearing + 360

        return tempBearing
    
# haversine formula, see: http://www.movable-type.co.uk/scripts/latlong.html    
def haversine(trackerLat, trackerLon, remoteLat, remoteLon):
        R = 6371        # radius of earth in Km

        dLat = math.radians(remoteLat-trackerLat)       # delta latitude in radians
        dLon = math.radians(remoteLon-trackerLon)       # delta longitude in radians
        ####################################
        a = math.sin(dLat/2)*math.sin(dLat/2)+math.cos(math.radians(trackerLat))*math.cos(math.radians(remoteLat))*math.sin(dLon/2)*math.sin(dLon/2)
        #############################
        c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a))
        
        d = R*c
        
#        return d*3280.839895 # multiply distance in Km by 3280 for feet
        return d
        
        
def elevation(targetAlt, stationAlt, distanceToTarget):
    deltaAlt = targetAlt - stationAlt
    elevationAngle = math.degrees(math.atan2(deltaAlt,distanceToTarget))
    
    return elevationAngle
    
    
def fitCalibration(calData):
    
    x = np.where(calData != -1)
    y = calData[x]
    
    m,c = np.linalg.lstsq(x,y)[0]
    
    print (m, c)

    