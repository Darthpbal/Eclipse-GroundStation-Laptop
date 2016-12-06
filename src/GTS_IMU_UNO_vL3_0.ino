#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/sleep.h>
#include <EEPROM.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
   */

//Instance Initializations
Adafruit_BNO055   bno = Adafruit_BNO055(55);    //Initializes an instance of the BNO055 called bno with an I2C address of 55
SoftwareSerial    mySerial(8, 7);               //Initializes an instance of SoftwareSerial called mySerial with RX and TX on pins 8 and 7
Adafruit_GPS      GPS(&mySerial);               //Initializes an instance of Adafruit_GPS called GPS using the mySerial instance

//Global Intializations
boolean           usingInterrupt = true;        //Use an interrupt to parce GPS data (preferred to be true)
int               saveCalTimestamp = 0;

#define IMU_CAL_SAVE_PERIOD (1000 * 60 * 10)    // 10 minutes

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (200)
#define GPSECHO  false                          //Display GPS data as read from the GPS in the Serial Monitor (dont do for actual use but good for debugging)


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void) {
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void) {
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display GPS data
    */
/**************************************************************************/
void displayGPS(void) {    
    Serial.print(GPS.hour);                 Serial.print(":");
    Serial.print(GPS.minute);               Serial.print(":");
    Serial.print(GPS.seconds);              Serial.print(",");  
    Serial.print(GPS.latitudeDegrees, 7);   Serial.print(",");
    Serial.print(GPS.longitudeDegrees, 7);  Serial.print(",");
    Serial.print(GPS.altitude * 3.28084);   Serial.print(",");
    Serial.print(GPS.fix);                  Serial.print(",");
    Serial.print(GPS.satellites);           Serial.print(",");
}

/**************************************************************************/
/*
    Display sensor orientation
    */
/**************************************************************************/
void displayOrientation(void) {
    sensors_event_t event;
    
    bno.getEvent(&event);
    
    Serial.print(event.orientation.x, 2);   Serial.print(",");
    Serial.print(event.orientation.y, 2);   Serial.print(",");
    Serial.print(event.orientation.z, 2);   Serial.print(",");
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void) {

    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */

    uint8_t sys, gyro, accel, mag;
    sys = gyro = accel = mag = 0;
    
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print(sys);    Serial.print(",");
    Serial.print(gyro);   Serial.print(",");
    Serial.print(accel);  Serial.print(",");
    Serial.print(mag);    Serial.print(",");
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(void) {
    adafruit_bno055_offsets_t calData;
    
    bno.getSensorOffsets(calData);

    Serial.print(calData.accel_offset_x); Serial.print(",");
    Serial.print(calData.accel_offset_y); Serial.print(",");
    Serial.print(calData.accel_offset_y); Serial.print(",");

    Serial.print(calData.gyro_offset_x);  Serial.print(",");
    Serial.print(calData.gyro_offset_y);  Serial.print(",");
    Serial.print(calData.gyro_offset_z);  Serial.print(",");

    Serial.print(calData.mag_offset_x);   Serial.print(",");
    Serial.print(calData.mag_offset_y);   Serial.print(",");
    Serial.print(calData.mag_offset_z);   Serial.print(",");

    Serial.print(calData.accel_radius);   Serial.print(",");
    Serial.print(calData.mag_radius);     Serial.print(",");
}

/**************************************************************************/
/*
    Check the calibration from the eeprom
    */
/**************************************************************************/
bool checkCalibration(void) {
    long      bnoID;
    sensor_t  sensor;
    int       eeAddress = 0;

    EEPROM.get(eeAddress, bnoID);
    bno.getSensor(&sensor);

    if (bnoID == sensor.sensor_id) {
        Serial.println("Found Calibration for this sensor in EEPROM.");
        return(true);
    }

    return(false);
}

/**************************************************************************/
/*
    Display the check the calibration from the eeprom 1=true, 0=false
    */
/**************************************************************************/
void displayCheckCalibration(void) {
    long      bnoID;
    sensor_t  sensor;
    int       eeAddress = 0;

    EEPROM.get(eeAddress, bnoID);
    bno.getSensor(&sensor);

    if (bnoID == sensor.sensor_id) {
        Serial.print("1");
    }

    else {
        Serial.print("0");
    }

    Serial.print(",");
}

/**************************************************************************/
/*
    Load the calibration from the eeprom
    */
/**************************************************************************/
bool loadCalibration(void) {

    int       eeAddress = 0;
    bool      foundCalib = false;
    adafruit_bno055_offsets_t calData;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */

    if (checkCalibration()) {
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calData);

        Serial.println("Restoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calData);

        Serial.println("Calibration data loaded into BNO055");
        foundCalib = true;
    }

    else {
        Serial.println("Calibration data NOT loaded into BNO055");
    }

    return(foundCalib);
}


/**************************************************************************/
/*
    Save the calibration to the eeprom
    */
/**************************************************************************/
bool saveCalibration(void) {
    int                         eeAddress = 0;
    long                        bnoID;
    bool                        foundCalib = false;
    sensor_t                    sensor;
    adafruit_bno055_offsets_t   newCalib;
    
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;
    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    bno.getSensorOffsets(newCalib);
    EEPROM.put(eeAddress, newCalib);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
void setup(void) {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Entering Setup...");

    /* Initialise the sensor */
    if (!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    bool foundCalib = loadCalibration();
    delay(1000);
 
    bno.setExtCrystalUse(true);                     //Use the external clock in the IMU (true for better accuracy)
//    bno.setMode(bno.OPERATION_MODE_NDOF);

    GPS.begin(9600);                                //Launches a software serial connection to the GPS at a baud rate of 9600
    delay(1500);                                    //Wait for 1.5s
    
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   //String formatting on the GPS
//    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);      //GPS packet dump rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    GPS.sendCommand(PGCMD_ANTENNA);

    useInterrupt(usingInterrupt);                   //Set to use or not use the interrupt for GPS parcing
}

SIGNAL(TIMER0_COMPA_vect) {                         // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
    char c = GPS.read();

    #ifdef UDR0
      if (GPSECHO)
          if (c) UDR0 = c;                          //UDRO is the register for the hardware serial module
    #endif
}

//void(* resetFunc) (void) = 0;                     //declare reset function at address 0. This is for a software reset (unused in current version)

void useInterrupt(boolean v) {                      //turns the interrupt(TIMER0_COMPA_vect) on or off based on the boolean passed to it
    
    if (v) {
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
    } 
    
    else {
        TIMSK0 &= ~_BV(OCIE0A);
        usingInterrupt = false;
    }
}

void loop() {

    Serial.print("$GSIMU,");   
    displayGPS();
    displayOrientation();
    displayCalStatus();
    displayCheckCalibration();   
    displaySensorOffsets(); 

    if (bno.isFullyCalibrated()) {
        Serial.print("1");

        if (millis() - saveCalTimestamp >= IMU_CAL_SAVE_PERIOD) {
            saveCalibration();
            saveCalTimestamp = millis(); 
        }
    }

    else {
      Serial.print("0");
    }

    Serial.println(",*,");

    delay(BNO055_SAMPLERATE_DELAY_MS);

    if (!usingInterrupt) {                                    //If interrupt is not being used, the GPS data needs to be checked for parse here.
        Serial.println("not using interrupt");
        char c = GPS.read();
        
        if (GPSECHO)
            if (c) Serial.print(c);
    }

    if (GPS.newNMEAreceived()) {                              //If data is new, parse it!

        if (!GPS.parse(GPS.lastNMEA()))
          return;
    }
}
