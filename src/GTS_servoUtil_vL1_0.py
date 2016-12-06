# -*- coding: utf-8 -*-
"""
Created on Fri Aug 19 21:45:37 2016

@author: Ground Station
"""

import math

# Pololu servo controller commands using Mini SSC Protocol, 
#  see: http://www.pololu.com/docs/0J40/5.c  
# Shouldn't need to change these usually

moveCommand = 0xFF
accelCommand = 0x89             
speedCommand = 0x87

# Shouldn't need to change these unless you change to some exotic servos

servo_min = 0
servo_max = 254

# change the movement speed etc of ubiquity tilt servo
tiltChannel = 0
tiltRange = 360
tiltAccel = 2
tiltSpeed = 3
tilt_angle_min = -180        #-90
tilt_angle_max = 180         #90

# change the movement speed etc of ubiquity pan servo
panChannel = 1
panRange = 360
panAccel = 2
panSpeed = 3

# change the movement speed etc of RFD900 tilt servo
rfd_tiltChannel = 2
rfd_tiltRange = 360
rfd_tiltAccel = 3
rfd_tiltSpeed = 7
rfd_tilt_angle_min = -180
rfd_tilt_angle_max = 180

# change the movement speed etc of RFD900 pan servo
rfd_panChannel = 3
rfd_panRange = 360
rfd_panAccel = 5
rfd_panSpeed = 7

previousPan = 127       #Memory for last position (To account for backlash)

currAltPosition = 255
currAziPosition = 255

altMinPosition = 50
altMaxPosition = 115

def setServoAccel(s):
        #Ubiquity Setup

    if s.isOpen:
        setAccel = [accelCommand,tiltChannel,tiltAccel,0]
        s.write(setAccel)
        setAccel = [accelCommand,panChannel,panAccel,0]
        s.write(setAccel)
        #RFD setup
        setAccel = [accelCommand,rfd_tiltChannel,rfd_tiltAccel,0]
        s.write(setAccel)
        setAccel = [accelCommand,rfd_panChannel,rfd_panAccel,0]
        s.write(setAccel)
    else:
        return []
        
def setServoSpeed(s):
        #Ubiquity Setup

    if s.isOpen:
        setSpeed = [speedCommand,tiltChannel,tiltSpeed,0]
        s.write(setSpeed)
        setSpeed = [speedCommand,panChannel,panSpeed,0]
        s.write(setSpeed)
        #RFD setup
        setSpeed = [speedCommand,rfd_tiltChannel,rfd_tiltSpeed,0]
        s.write(setSpeed)
        setSpeed = [speedCommand,rfd_panChannel,rfd_panSpeed,0]
        s.write(setSpeed)
    else:
        return []

## for altitude, the lower the number the higher the altitude
## 86 is the minimum servo setting == the higest altitude
## A servo setting of 123 is almost level with the horizon.        
def moveServosToPosition(s, aziPosition, altPosition):
    global currAziPosition, currAltPosition, altMinPosition, altMaxPosition

    if s.isOpen:
        if altPosition < altMinPosition: altPosition = altMinPosition
        if altPosition > altMaxPosition: altPosition = altMaxPosition
        if aziPosition > 254: aziPosition = 254
        if aziPosition < 0: aziPosition = 0
        
        moveAlt = [moveCommand, tiltChannel, chr(altPosition)]
        moveAzi = [moveCommand, panChannel, chr(254-aziPosition)]

        print 'move servos to: %.2f, %.2f' % (aziPosition, altPosition)

        try:        
            s.write(moveAlt)
            s.write(moveAzi)
        
            currAziPosition = aziPosition
            currAltPosition = altPosition
        except:
            print 'moveServosToPosition failed'
            return []

    else:
        return []
            
    return [aziPosition, altPosition]
    
def nudgeServos(s, aziNudge, altNudge):
    global currAziPosition, currAltPosition, altMinPosition, altMaxPosition
    
    if currAziPosition == 255 or currAltPosition == 255:
        moveServosToPosition(s, 127, 127)
        print 'centering servos, they have not been initialized'
        
    altPosition = currAltPosition + altNudge
    aziPosition = currAziPosition + aziNudge
    
    if s.isOpen:       
        if altPosition < altMinPosition: altPosition = altMinPosition
        if altPosition > altMaxPosition: altPosition = altMaxPosition
        if aziPosition > 254: aziPosition = 0
        if aziPosition < 0: aziPosition = 253
        
        moveAlt = [moveCommand, tiltChannel, chr(altPosition)]
        moveAzi = [moveCommand, panChannel, chr(254-aziPosition)]

        print 'move servos to: %.2f, %.2f' % (aziPosition, altPosition)

        try:        
            s.write(moveAlt)
            s.write(moveAzi)
        
            currAziPosition = aziPosition
            currAltPosition = altPosition
        except:
            print 'moveServosToPosition failed'
            return []

    else:
        return []
            
    return [aziPosition, altPosition]
    
## function     :   getServoMaxAltitude
## author       :   Michael Stewart
## date         :   8.27.16
    
def getServoMaxAltitude():
    global altMinPosition
    
    return altMinPosition
    
## function     :   getServoMinAltitude
## author       :   Michael Stewart
## date         :   8.27.16
    
def getServoMinAltitude():
    global altMaxPosition
    
    return altMaxPosition
    