###############################################################################
#       Antenna Tracker Controller for Trident Antenna Array                                                                    #
#                                                                                                                               #
#       Author: Michael F. Stewart, MS
#       Email:  stewart@phunds.phys.lsu.edu
#				mfstewart@outlook.com
#       Date:   August 2016
#
#       Very first implementation created by Dylan Trafford, EE & CpE                                                                                        #
#       and based on work from Scott Miller, CpE                                                                                    #
#       
#       Software created for use by Louisiana State University, Louisiana
#       Space Consoritium (LaSPACE) specifically for use by the Eclipse and
#       COTEL projects
#
#       Version Log
#       vL1_0 - Started Aug 16, 2016
#       vL2_0 - Adding comments and header descriptions
#                       
###############################################################################
from GTS_mainwindow_vL1_0 import Ui_MainWindow, QtGui, QtCore

import sys
import serial
import time
import numpy as np

from GTS_mathUtil_vL1_0 import haversine, bearing, elevation
from GTS_servoUtil_vL1_0 import setServoAccel, setServoSpeed, moveServosToPosition, nudgeServos, \
                                getServoMinAltitude, getServoMaxAltitude
from GTS_fileUtil_vL1_0 import readLine

configFile = "GTS_configuration.ini"

comList = ["COM1","COM2","COM3","COM4","COM5","COM6","COM7","COM8","COM9","COM10","COM11","COM12","COM13","COM14","COM15"]
baudList = ["1200","2400","4800","9600","19200","38400","57600","115200"]
dataSourceList = ["APRS", "Iridium", "RFD"]
updateMethodList = ["Single", "Periodic"]
#callsignList = ['KF5FPX-11', 'KG5NBC-11']
callsignList = ['KF5FPX-11']
settingList = ['$GSIMU', '$GSAPR', '$GSSRV']

timerCount = 0
trackingPeriod = 10
trackingON = False

imuConnected = False
aprsConnected = False
srvoConnected = False
calibrateGroundStation = False
calibrateAzi = False
calibrateElev = False
isCalibrated = False

calibrateIndex = 0
skipCount = 0
startCount = 0
calibrateSteps = 0

imuPort = serial.Serial()
aprsPort = serial.Serial()
srvoPort = serial.Serial()

imuList = []
aprsList = []
trackList = []

aziDegreeLookup = np.zeros(361)
elevDegreeLookup = np.zeros(361)
aziDegreeLookup.fill(-1)
elevDegreeLookup.fill(-1)

prevAziDiff = 0.0
prevElevDiff = 0.0
trueAziDiff = 0.0
trueElevDiff = 0.0
moveAziDirection = 1.0
moveElevDirection = 1.0

centerBearing = 0.0
panOffset = 0.0
tiltOffset = 0.0

feetPerKm = 3280.84

class MainWindow(QtGui.QMainWindow, Ui_MainWindow):
        
    def __init__(self, parent=None):        
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        
        self.cbo_imuComPort.addItems(comList)
        self.cbo_imuBaudRate.addItems(baudList)
        self.btn_imuConnect.clicked.connect(self.imuConnect)
        
        self.cbo_aprsComPort.addItems(comList)
        self.cbo_aprsBaudRate.addItems(baudList)
        self.btn_aprsConnect.clicked.connect(self.aprsConnect)
        
        self.cbo_srvoComPort.addItems(comList)
        self.cbo_srvoBaudRate.addItems(baudList)
        self.btn_srvoConnect.clicked.connect(self.servoConnect)
        
        self.cbo_trgtDataSource.addItems(dataSourceList)
        self.cbo_sttnUpdateMethod.addItems(updateMethodList)
        
        self.btn_trgtClearSources.clicked.connect(self.clearSources)
        self.btn_sttnUpdatePointing.clicked.connect(self.updatePointing)
        self.btn_sttnUpdatePosition.clicked.connect(self.updatePosition)
        
        self.btn_moveCenter.clicked.connect(lambda: self.moveAntenna("Center", False))
        self.btn_moveLeft.clicked.connect(lambda: self.moveAntenna("Left", False))
        self.btn_moveLeftFine.clicked.connect(lambda: self.moveAntenna("Left", True))
        self.btn_moveRight.clicked.connect(lambda: self.moveAntenna("Right", False))
        self.btn_moveRightFine.clicked.connect(lambda: self.moveAntenna("Right", True))
        self.btn_moveUp.clicked.connect(lambda: self.moveAntenna("Up", False))
        self.btn_moveUpFine.clicked.connect(lambda: self.moveAntenna("Up", True))
        self.btn_moveDown.clicked.connect(lambda: self.moveAntenna("Down", False))
        self.btn_moveDownFine.clicked.connect(lambda: self.moveAntenna("Down", True))
        self.btn_sttnCalibrate.clicked.connect(self.calibrateStation)
#        self.btn_sttnCalibrateIMU.clicked.connect(self.calibrateIMU)
        self.btn_sttnTrackOn.clicked.connect(self.toggleTracking)
        self.btn_saveSettings.clicked.connect(self.saveSettings)
        
        self.refreshtimer = QtCore.QTimer()
        self.refreshtimer.timeout.connect(self.timerExpire)
        self.refreshtimer.start(50)
        
        self.initialize()
        
    ## Class    :   MainWindow
    ## Function :   initialize
    ## Date     :   8.24.16
        
    def initialize(self):
        global settingList, configFile
        
        print 'config file is %s' % configFile
        
        tstr = readLine(configFile, '$GSIMU')
        
        if tstr:
            splitStr = tstr.split(',')
            self.log('Read settings: ' + tstr, 'INF')
            index = self.cbo_imuComPort.findText(splitStr[1].strip(), QtCore.Qt.MatchFixedString)
            if index >= 0: self.cbo_imuComPort.setCurrentIndex(index)
            
            index = self.cbo_imuBaudRate.findText(splitStr[2].strip(), QtCore.Qt.MatchFixedString)
            if index >= 0: self.cbo_imuBaudRate.setCurrentIndex(index)
                
            self.spn_imuPollPeriod.value = int(splitStr[3])
                
        tstr = readLine(configFile, '$GSAPR')
        
        if tstr:
            splitStr = tstr.split(',')
            self.log('Read settings: ' + tstr, 'INF')
            index = self.cbo_aprsComPort.findText(splitStr[1].strip(), QtCore.Qt.MatchFixedString)
            if index >= 0: self.cbo_aprsComPort.setCurrentIndex(index)
            
            index = self.cbo_aprsBaudRate.findText(splitStr[2].strip(), QtCore.Qt.MatchFixedString)
            if index >= 0: self.cbo_aprsBaudRate.setCurrentIndex(index)
                
        tstr = readLine(configFile, '$GSSRV')
        
        if tstr:
            splitStr = tstr.split(',')
            self.log('Read settings: ' + tstr, 'INF')
            index = self.cbo_srvoComPort.findText(splitStr[1].strip(), QtCore.Qt.MatchFixedString)
            if index >= 0: self.cbo_srvoComPort.setCurrentIndex(index)
            
            index = self.cbo_srvoBaudRate.findText(splitStr[2].strip(), QtCore.Qt.MatchFixedString)
            if index >= 0: self.cbo_srvoBaudRate.setCurrentIndex(index)
        
    ## Class    :   MainWindow
    ## Function :   imcConnect
    ## Date     :   8.17.16
    ## 
    def imuConnect(self):
        global imuConnected, imuPort
        
        if (not imuConnected):
            imuPort = serialConnect(self.cbo_imuComPort.currentText(), int(self.cbo_imuBaudRate.currentText()))
        
            if (imuPort != []):
                imuConnected = True
                self.btn_imuConnect.setText("Disconnect")
        else:
            imuPort.close()
            imuConnected = False
            self.btn_imuConnect.setText("Connect")
                        
    ## Class    :   MainWindow
    ## Function :   aprsConnect
    ## Date     :   8.17.16
    ##    
    def aprsConnect(self):
        global aprsConnected, aprsPort
        
        if (not aprsConnected):
            aprsPort = serialConnect(self.cbo_aprsComPort.currentText(), int(self.cbo_aprsBaudRate.currentText()))
        
            if (aprsPort != []):
                aprsConnected = True
                self.btn_aprsConnect.setText("Disconnect")
        else:
            aprsPort.close()
            aprsConnected = False
            self.btn_aprsConnect.setText("Connect")
            
    ## Class    :   MainWindow
    ## Function :   servoConnect
    ## Date     :   8.17.16
    ##    
    def servoConnect(self):
        global srvoConnected, srvoPort
        
        if (not srvoConnected):
            srvoPort = serialConnect(self.cbo_srvoComPort.currentText(), int(self.cbo_srvoBaudRate.currentText()))
        
            if (srvoPort != []):
                srvoConnected = True
                setServoAccel(srvoPort)
                setServoSpeed(srvoPort)
                moveServosToPosition(srvoPort, 127, 127)
                self.btn_srvoConnect.setText("Disconnect")
        else:
            srvoPort.close()
            srvoConnected = False
            self.btn_srvoConnect.setText("Connect")
            
    ## Class    :   MainWindow
    ## Function :   
    ## Date     :   8.19.16
    ##    
    def clearSources(self):
        global aprsConnected, aprsPort

    ## Class    :   MainWindow
    ## Function :   
    ## Date     :   8.19.16
    ##    
    def updatePosition(self):
        global aprsConnected, aprsPort

    ## Class    :   MainWindow
    ## Function :   
    ## Date     :   8.19.16
    ##    
    def updatePointing(self):
        global aprsConnected, aprsPort                   

    ## Class    :   MainWindow
    ## Function :   
    ## Date     :   8.19.16
    ##    
    def moveAntenna(self, dirStr, isFine):
        global srvoConnected, srvoPort, centerBearing, panOffset, tiltOffset
        
        moveSteps = int(self.spn_moveSensitivity.text())
        
        if not isFine:
            moveSteps *= 5
            
        bearing = self.lcd_sttnBearing.value()
        elevation = self.lcd_sttnElevation.value()
        
        self.log('Moving antenna from %.2f, %.2f by %.2f deg.' % (bearing, elevation, moveSteps), 'INF')
        
        if dirStr == 'Center':
            moveServosToPosition(srvoPort, 127, 127)
        elif dirStr == 'Up':
            nudgeServos(srvoPort, 0, 0-moveSteps)
        elif dirStr == 'Left':
            nudgeServos(srvoPort, 0-moveSteps, 0)
        elif dirStr == 'Right':
            nudgeServos(srvoPort, moveSteps, 0)
        elif dirStr == 'Down':
            nudgeServos(srvoPort, 0, moveSteps)
        else:
            print 'errorrrr'
            
        self.log('After: antenna positions: centerBear: %.2f, panOffset: %.2f, tiltOffset: %.2f' % (centerBearing, panOffset, tiltOffset), 'INF')
        
    ## Class    :   MainWindow
    ## Function :   This function is tied to the calibrateStation button, 
    ##              if the station is calibrating then stop, if it is not
    ##              then start the calibration
    ## Date     :   8.19.16    
    def calibrateStation(self):
        global  calibrateGroundStation, calibrateIndex, skipCount, startCount, \
                calibrateAzi, calibrateElev, calibrateSteps
        
        self.prg_sttnCalibrate.setValue(0)
        
        if calibrateGroundStation:
            # if currently calibrating the ground station, then stop calibration
            calibrateGroundStation = False
            calibrateAzi = False            # Azimuth and Elevation calibrated separately
            calibrateElev = False
            self.btn_sttnCalibrate.setText('Calibrate Station')
            moveServosToPosition(srvoPort, 127, 127)
        else:
            moveServosToPosition(srvoPort, 0, 127)
            calibrateIndex = 0
            skipCount = 0
            startCount = 0
            calibrateSteps = 0
            self.btn_sttnCalibrate.setText('Stop Calibration')
            calibrateGroundStation = True
            calibrateAzi = True
            self.txt_sttnElevCalibrationTable.clear()
            self.txt_sttnAziCalibrationTable.clear()
            
    ## Class    :   MainWindow
    ## Function :   
    ## Date     :   8.19.16
    ##    
    def toggleTracking(self):
        global trackingON
        
        self.prg_sttnCalibrate.setValue(0)
        
        if trackingON:
            trackingON = False
            self.btn_sttnTrackOn.setText('Tracking ON')
        else:
            trackingON = True
            self.btn_sttnTrackOn.setText('Tracking OFF')
        
    ## Class    :   MainWindow
    ## Function :   
    ## Date     :   8.19.16
    ##    
    def saveSettings(self):
#        global
        
        imuConfigList = ['$GSIMU', self.cbo_imuComPort.currentText(), self.cbo_imuBaudRate.currentText()]
        aprsConfigList = ['$GSAPR', self.cbo_aprsComPort.currentText(), self.cbo_aprsBaudRate.currentText()]
        servoConfigList = ['$GSSRV', self.cbo_srvoComPort.currentText(), self.cbo_srvoBaudRate.currentText()]
           
            
    ## Class    :   MainWindow
    ## Function :   aprsConnect
    ## Date     :   8.17.16
    ##        
    def timerExpire(self):
        global timerCount
        
        self.refreshtimer.stop()
        
        self.processIMU()
        self.processAPRS()
        self.processTracking()
        self.processCalibration()
                   
        timerCount += 1
        
        self.refreshtimer.start(50)
        
    ## Class    :   MainWindow
    ## Function :   
    ## Date     :   8.19.16
    ##    
    def processCalibration(self):
        global  srvoConnected, srvoPort, calibrateGroundStation, calibrateIndex, \
                skipCount, imuList, startCount, calibrateAzi, calibrateElev, \
                aziDegreeLookup, elevDegreeLookup, isCalibrated, calibrateSteps
        
        if startCount < 100:
            startCount += 1
            return
        
        if skipCount < 3:
            skipCount += 1
            return
            
        skipCount = 0
        
        if calibrateGroundStation:
            
            if not imuList:
                self.log('attempt to calibrate station when imuList is NULL', 'ERR')
                self.calibrateStation()
                return
            
            if calibrateAzi:
                idx = int(round(imuList[7]))
                
                if idx < 0 or idx > 360:
                    self.log('error azi calibrating, index %d out of bounds' % (idx), 'ERR')
                    return
                    
                aziDegreeLookup[idx] = calibrateIndex
                tStr = '%03d - %03d' % (idx, aziDegreeLookup[idx])
                calibrateIndex += 1
                calibrateSteps += 1
                self.prg_sttnCalibrate.setValue(calibrateSteps*100/330)
                
                if calibrateIndex >= 253:
                    calibrateAzi = False
                    calibrateElev = True
                    skipCount = 0
                    startCount = 0
                    calibrateIndex = getServoMinAltitude()
                    moveServosToPosition(srvoPort, 127, calibrateIndex)
                    return
                    
                moveServosToPosition(srvoPort, calibrateIndex, 127)           
                self.txt_sttnAziCalibrationTable.append(tStr)
            
            elif calibrateElev:
                print 'calibrate elevation %d' % (calibrateIndex)
                idx = int(round(imuList[9]))
                
                if idx < 0:
                    self.log('error elev calibrating, index %d out of bounds' % (idx), 'ERR')
                    calibrateIndex -= 1
                    moveServosToPosition(srvoPort, 127, calibrateIndex)
                    return
                
                if idx > 360:
                    self.log('error elev calibrating, index %d out of bounds' % (idx), 'ERR')
                    return
                    
                elevDegreeLookup[idx] = calibrateIndex
                tStr = '%03d - %03d' % (idx, elevDegreeLookup[idx])
                calibrateIndex -= 1
                calibrateSteps += 1
                self.prg_sttnCalibrate.setValue(calibrateSteps*100/330)
                
                if calibrateIndex <= getServoMaxAltitude():
                    calibrateElev = False
                    return
                    
                moveServosToPosition(srvoPort, 127, calibrateIndex)           
                self.txt_sttnElevCalibrationTable.append(tStr)
            else:
                isCalibrated = True
                printCalibration()
                self.calibrateStation()
                
    ## Class    :   MainWindow
    ## Function :   
    ## Date     :   8.19.16
    ##    
    def processCalibrationV2(self):
        global  srvoConnected, srvoPort, calibrateGroundStation, calibrateIndex, \
                skipCount, imuList, startCount, calibrateAzi, calibrateElev, \
                aziDegreeLookup, elevDegreeLookup, isCalibrated, calibrateSteps
        
        if startCount < 100:
            startCount += 1
            return
        
        if skipCount < 3:
            skipCount += 1
            return
            
        skipCount = 0
        
        if calibrateGroundStation:
            
            if not imuList:
                self.log('attempt to calibrate station when imuList is NULL', 'ERR')
                self.calibrateStation()
                return
            
            if calibrateAzi:
                deg = int(round(imuList[7]))
                
                if deg < 0 or deg > 360:
                    self.log('error azi calibrating, index %d out of bounds' % (deg), 'ERR')
                    return
                    
#                aziPositionLookup[calibrateIndex] = deg
                tStr = '%03d - %03d' % (calibrateIndex,aziPositionLookup[calibrateIndex])
                calibrateIndex += 1
                calibrateSteps += 1
                self.prg_sttnCalibrate.setValue(calibrateSteps*100/330)
                
                if calibrateIndex >= 253:
                    calibrateAzi = False
                    calibrateElev = True
                    skipCount = 0
                    startCount = 0
                    calibrateIndex = getServoMinAltitude()
                    moveServosToPosition(srvoPort, 127, calibrateIndex)
                    return
                    
                moveServosToPosition(srvoPort, calibrateIndex, 127)           
                self.txt_sttnAziCalibrationTable.append(tStr)
            
            elif calibrateElev:
                print 'calibrate elevation %d' % (calibrateIndex)
                idx = int(round(imuList[9]))
                
                if idx < 0:
                    self.log('error elev calibrating, index %d out of bounds' % (idx), 'ERR')
                    calibrateIndex -= 1
                    moveServosToPosition(srvoPort, 127, calibrateIndex)
                    return
                
                if idx > 360:
                    self.log('error elev calibrating, index %d out of bounds' % (idx), 'ERR')
                    return
                    
                elevDegreeLookup[idx] = calibrateIndex
                tStr = '%03d - %03d' % (idx, elevDegreeLookup[idx])
                calibrateIndex -= 1
                calibrateSteps += 1
                self.prg_sttnCalibrate.setValue(calibrateSteps*100/330)
                
                if calibrateIndex <= getServoMaxAltitude():
                    calibrateElev = False
                    return
                    
                moveServosToPosition(srvoPort, 127, calibrateIndex)           
                self.txt_sttnElevCalibrationTable.append(tStr)
            else:
                isCalibrated = True
                printCalibration()
                self.calibrateStation()
                
            
        
    ## function     :   processIMC
    ## date         :   8.18.16
    ##                
    def processIMU(self):
        global imuConnected, imuPort, timerCount, imuList
        
        if (imuConnected) and (timerCount % int(self.spn_imuPollPeriod.text()) == 0):
            serialStr = serialRead(imuPort, 200)
            
            if (serialStr != []):
                self.log(serialStr, 'IMU')
                tList = parseIMU(serialStr)
                
                if tList:
                    imuList = tList
                    self.displayIMU(imuList)
                
    ## function     :   loadImuList()
    ## date         :   8.19.2016
    ##               
    def displayIMU(self, L):
        
        if not L:
            self.log('IMU list is empty', 'WRN')
            return
            
        self.lcd_sttnTimestamp.display(L[1])
        self.lcd_sttnPointingTimestamp.display(L[1])
        self.lcd_sttnLatitude.display(L[2])
        self.lcd_sttnLongitude.display(L[3])
        self.lcd_sttnAltitude.display(L[4])
        self.lcd_sttnLock.display(L[5])
        self.lcd_sttnSatelliteCount.display(L[6])
        self.lcd_sttnBearing.display(L[7])
        self.lcd_sttnElevation.display(L[9])
        self.lcd_sttnLevel.display(L[8])
        self.lcd_sttnSysCalibration.display(L[10])
        self.lcd_sttnGyroCalibration.display(L[11])
        self.lcd_sttnAcclCalibration.display(L[12])
        self.lcd_sttnMagCalibration.display(L[13])
       
    ## function     :   processAPRS
    ## date         :   8.18.18
    ##
    def processAPRS(self):
        global aprsConnected, aprsPort, timerCount, aprsList
        
        if (aprsConnected) and (timerCount % int(self.spn_aprsPollPeriod.text()) == 0):
            serialStr = serialRead(aprsPort, 68)
            
            if (serialStr != []):
                self.log(serialStr, 'APR')
                tList = parseAPRS(serialStr)
                
                if tList:
                    aprsList = tList
                    self.displayAPRS(aprsList)
                
    ## function     :   loadImuList()
    ## date         :   8.19.2016
    ##               
    def displayAPRS(self, L):
        
        if not L:
            self.log('APRS list is empty', 'WRN')
            return
            
        self.lcd_trgtTimestamp.display(L[2])
        self.lcd_trgtLatitude.display(L[3])
        self.lcd_trgtLongitude.display(L[4])
        self.lcd_trgtAltitude.display(L[5])
        
    ## function     :   processTracking
    ## date         :   8.19.2016
    ##
    def processTracking(self):
        global imuList, aprsList, trackList, trackingON, timerCount, trackingPeriod
        
#        if timerCount % trackingPeriod == 0:
        if (imuConnected) and (timerCount % int(self.spn_imuPollPeriod.text()) == 0):
        
            if not imuList:
                self.log('IMU list is empty', 'WRN')
                return
                
            if not aprsList:
                self.log('APRS list is empty', 'WRN')
                return
            
            tList = self.calculateTrack(imuList, aprsList)
            
            if tList:
                trackList = tList
                self.trueAziDiff = trackList[1] - imuList[7]
                self.trueElevDiff = trackList[2] - imuList[9]
                
                self.lcd_trgtBearing.display(trackList[1])
                self.lcd_trgtElevation.display(trackList[2])
                self.lcd_trgtDistance.display(trackList[0])
                
            if trackingON:
                print "tracking is ON"
                
#                if isCalibrated:
#                    trackTarget(trackList)
                fineTracking()
        
    ## function     :   trackTarget
    ## date         :   8.19.2016
    ##
    def calculateTrack(self, sL, tL):
        global feetPerKm
        
        distanceToTarget = haversine(sL[2], sL[3], tL[3], tL[4])
        bearingToTarget = bearing(sL[2], sL[3], tL[3], tL[4])
        elevationAngle = elevation(tL[5], sL[4], distanceToTarget * feetPerKm)
        
        return [distanceToTarget, bearingToTarget, elevationAngle]
        
    ## function     :   log
    ## date         :   8.17.16
    ##               
    def log(self, logStr, severity):
        doLog = False
        
        if severity == 'IMU' and self.chkBox_logIMU.isChecked():
            doLog = True            
        if severity == 'APR' and self.chkBox_logAPRS.isChecked():
            doLog = True
        if severity == 'INF' and self.chkBox_logInfo.isChecked():
            doLog = True
        if severity == 'WRN' and self.chkBox_logWarning.isChecked():
            doLog = True
        if severity == 'ERR' and self.chkBox_logError.isChecked():
            doLog = True
        if severity == 'SEV' and self.chkBox_logSevere.isChecked():
            doLog = True
        if severity == 'FTL' and self.chkBox_logFatal.isChecked():
            doLog = True
        
        logStr = getTimestamp() + ' :: ' + severity + ' :: ' + logStr
        
        if doLog:
            self.txt_logData.append(logStr)
            
#        print logStr
                
###############################################################################
## End of MainWindow Class
###############################################################################
                
                
## function     :   parseIMU
## date         :   8.18.16
##
            
def parseIMU(s):
    
    imuList = s.split(',');
    
    if imuList[0] != '$GSIMU':
        mGui.log('error with IMU string; $GSIMU not first string: ' + s, 'WRN')
        return []
        
    if len(imuList) < 15:
        mGui.log('error with IMU string; list only has %d elements' % (len(imuList)), 'WRN')
        return []
        
#    if imuList[27] != '*':
#        mGui.log('error with IMU string; not enough parameters: ' + s, 'WRN')
#        return []
                
    for i in range(2,14):
        
        try:
            imuList[i] = float(imuList[i])
        except:
            # may have to remove imuList or check to make sure that it isn't null
            mGui.log('error inserting into imuList', 'ERR')
            return []
            
    imuList[9] = -1.0 * imuList[9]
            
    return imuList
    
    
## function     :   parseAPRS
## date         :   5.1.16
## author       :   Connor Mayeaux
## description  :   Parses the verified APRS string into an array with the format
##                  [callsign, UTC time, latitude, longitude, altitude]. UTC time
##                  is further parsed into an array [hours, minutes, seconds].
##    
## @param       :   radio_string   the incoming data from the HAM radio
## @return      :   an array with the above format or nothing
##                  if results does not have 5 elements
##
    
def parseAPRS(s):
    global mGui
    
    if not check_callsign(s):
        mGui.log('error with APRS string; bad callsign: ' + s, 'WRN')
        return []

    if not isAPRS(s): 
        mGui.log('error, not a valid APRS string: ' + s, 'WRN')
        return []
    
    results = s.split('/')

    if(len(results) != 5):
        mGui.log('error, incorrect number of APRS substrings' + s, 'WRN')
        return []
        
    callsign = results[0][:9]
    timeStr = results[1][:6]
    latStr = results[1][-8:-1]
    latDir = results[1][-1]
    lonStr = results[2][:8]
    lonDir = results[2][8]
    altStr = results[-1][2:8]
    timeStr = [timeStr[:2], timeStr[2:4], timeStr[4:]]
    
    try:
        hour = int(timeStr[0])
        minute = int(timeStr[1])
        second = int(timeStr[2])
    except:
        mGui.log('error, time string not valid', 'WRN')
        return []
        
    try:
        latitude = float(latStr[:2]) + (float(latStr[2:]) / 60.0) #converts from degrees&decimal minutes(DDMM.MM) to decimal degrees(DDD.DD)        
        if (latDir=='S'):  #checks if lat is negative
            latitude = latitude * -1.0
            
        longitude = float(lonStr[:3]) + (float(lonStr[3:]) / 60.0)
        if(lonDir=='W'):  #checks if long is negative
            longitude = longitude * -1.0
            
        altitude = int(altStr)
    except:
        mGui.log('error, latitude, longitude or altitude not valid', 'WRN')
        return []
        
    tStr = getDatestamp() + ' ' + '%02d:%02d:%02d' % (hour, minute, second)      
    aprsList = ['$GDAPR', callsign, tStr, latitude, longitude, altitude, '*']
    
    return aprsList
    
    
# Checks whether the APRS string begins with a valid callsign
# by confirming the SSID and (optionally) confirming the 2x3
# format of the callsign.
# @param callsigns      array of callsigns already encountered
# @param radio_string   the incoming data from the HAM radio
# @return (boolean)     whether the callsign exists and is valid

def check_callsign(radio_string):
    global callsignList
    j=0

    for i in range(len(radio_string)):
        if radio_string[i] == '>':
            j=i
            break

    for i in range(len(callsignList)):
        if(radio_string[:j]==callsignList[i]):
            return True

    return False


# Checks whether the APRS string is in the format which is meant
# to contain relevant data.
# @param radio_string   the incoming data from the HAM radio
# @return               whether the APRS string contains data

def isAPRS(radio_string):
    bracket_pos,colon_pos = 0,0
    
    for i in range(len(radio_string)):
        if radio_string[i] == '>':
            bracket_pos = i
        elif radio_string[i] == ':':
            colon_pos = i

    if radio_string[bracket_pos:bracket_pos + 3] == '>AP' and\
       radio_string[colon_pos + 1] == '/':
        return True

    return False
                

## function     :   serialConnect
## date         :   8.18.16
##
            
def serialConnect(comPort, baudRate):
    port = []
   
    try:
        port = serial.Serial(comPort, baudrate=baudRate, timeout=0.1)
        mGui.log('Opened ' + comPort + ' at baudrate ' + str(baudRate), 'INF')
    except:
        mGui.log('error connecting to ' + comPort, 'ERR')
        return port
    
    return port
    
## function     :   readSerial
## date         :   8.17.16
##

def serialRead(port, readLength):

    try:    
        serStr = port.read(readLength) #length of APRS packet is 68 char
        port.flushInput()

        if( len(serStr) <= 1):
            serStr = []

    except:
        mGui.log('error reading serial port %s' % (port), 'ERR')
        return []
    
    return serStr
    
## function     :   getTimestamp
## date         :   8.18.16
##
 
def trackTarget(trackList, imuL):
    global aziDegreeLookup, elevDegreeLookup
    
    aziPosition = -1
    elevPosition = -1
    reachedMax = False
    reachedMin = False
    moveDown = False
    moveDeg = 0
    direction = 1
    
    aziDegree = int(round(trackList[1]))
    elevDegree = int(round(trackList[2]))
        
    if aziDegree < 0 or aziDegree > 360:
        print 'Error pointing to Azi degree %d out of range' % (aziDegree)
        return
        
    if elevDegree < 0:
        elevDegree = 0
        
    if elevDegree > 90:
        elevDegree = 90
    
    print 'Pointing to Azi degree %d and Elev degree %d' % (aziDegree, elevDegree)
            
    while aziPosition == -1:
        aziPosition = aziDegreeLookup[aziDegree+moveDeg*direction]
        moveDeg += 1
        direction = 1
        
        if moveDown:
            direction = -1
            moveDown = False
        else:
            moveDown = True
            
        if aziDegree+moveDeg*direction > 360:
            direction = -1
            reachedMax = True
            
        if aziDegree+moveDeg*direction < 0:
            direction = 1
            reachedMin = True
            
        if reachedMax and reachedMin:
            print 'Have check all values, everything is -1'
            return
        
        print 'Point to Azi position %d - degree %d' % (aziPosition, aziDegree+moveDeg*direction)
            
    reachedMax = False
    reachedMin = False
    moveDown = False
    moveDeg = 0
        
    while elevPosition == -1:
        elevPosition = elevDegreeLookup[elevDegree+moveDeg*direction]
        moveDeg += 1
        direction = 1
        
        if moveDown:
            direction = -1
            moveDown = False
        else:
            moveDown = True
            
        if elevDegree+moveDeg*direction > 90:
            direction = -1
            reachedMax = True
            
        if elevDegree+moveDeg*direction < 0:
            direction = 1
            reachedMin = True
            
        if reachedMax and reachedMin:
            print 'Have check all values, everything is -1'
            return
        
        print 'Point to Elev position %d - degree %d' % (elevPosition, elevDegree+moveDeg*direction)
        
    moveServosToPosition(srvoPort, int(aziPosition), int(elevPosition))
    
        
## function     :   getTimestamp
## date         :   8.18.16
##
 
def fineTracking():
    global  srvoPort, trackList, imuList, prevAziDiff, prevElevDiff, \
            moveAziDirection, moveElevDirection, trueAziDiff, trueElevDiff
    
    if not checkAzimuth(trackList[1]) or not checkAzimuth(imuList[7]):
        print 'tracking azimuth %d or station azi %d is bad' % (trackList[1], imuList[7])
        return
        
    if not checkElevation(trackList[2]) or not checkElevation(imuList[9]):
        print 'tracking elevation %d or station elevation %d is bad' % (trackList[2], imuList[9])
        return
        
    aziDiff = int(round(trackList[1])) - int(round(imuList[7]))
    elevDiff = int(round(trackList[2])) - int(round(imuList[9]))
    
    if abs(aziDiff) <= 2 and abs(elevDiff) <= 2: 
         print 'Azi Diff (%d) and Elev Diff (%d) less than 2' % (abs(aziDiff), abs(elevDiff))
         return
         
    print 'Azi diff %d, %d, %d Elev Diff %d, %d, %d' % (aziDiff, prevAziDiff, moveAziDirection, elevDiff, prevElevDiff, moveElevDirection)  
   
    if abs(aziDiff) > 2:
        
        if abs(aziDiff) > abs(prevAziDiff)+1:
            moveAziDirection *= -1
        
        nudgeServos(srvoPort, int(moveAziDirection * 1), 0)
        
    if abs(elevDiff) > 2:
        
        if abs(elevDiff) > abs(prevElevDiff)+1:
            moveElevDirection *= -1
            
        nudgeServos(srvoPort, 0, int(moveElevDirection * 1))
            
    prevAziDiff = aziDiff
    prevElevDiff = elevDiff

    
def checkAzimuth(aziDegree):
    
    if aziDegree < 0 or aziDegree > 360:
        print 'Error pointing to Azi degree %d out of range' % (aziDegree)
        return False
        
    return True
        
def checkElevation(elevDegree):
        
    if elevDegree < 0:
        elevDegree = 0
        
    if elevDegree > 90:
        elevDegree = 90
        
    return True

    
def printCalibration():
    global aziDegreeLookup, elevDegreeLookup
    
    for i in range(len(aziDegreeLookup)):
        print 'At degree %d -- Azi Position: %d,  Elev Position: %d' % (i, aziDegreeLookup[i], elevDegreeLookup[i])
        
     
## function     :   getTimestamp
## date         :   8.18.16
##
       
def getTimestamp():
    t = time.gmtime()
    return '%02d:%02d:%02d %02d:%02d:%02d' % (t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec)
    
    
## function     :   getDatestamp
## date         :   8.18.16
##
       
def getDatestamp():
    t = time.gmtime()
    return '%02d:%02d:%02d' % (t.tm_year, t.tm_mon, t.tm_mday)

        
## Function     :   main
## Date         :   
##        
def main():
    global mGui
    
    app = QtGui.QApplication(sys.argv)
    mGui = MainWindow()
    mGui.show()   
    app.exec_()


if __name__ == '__main__':  # if we're running file directly and not importing it
    main()                  # run the main function


