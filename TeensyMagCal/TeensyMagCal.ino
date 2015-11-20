////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib2-Teensy
//
//  Copyright (c) 2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "RTIMULib.h"
#include "RTIMUMagCal.h"

RTIMU *imu;                                           // the IMU object
RTIMUSettings *settings;                              // the settings object
RTIMUMagCal *magCal;                                  // the mag calibration object
unsigned long lastDisplay;

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  200                         // interval between min/max displays

void setup()
{
    int errcode;

    Serial.begin(SERIAL_PORT_SPEED);
    while (!Serial) {
        ; // wait for serial port to connect. 
    }
    Wire.begin();
   
    settings = new RTIMUSettings();
    imu = RTIMU::createIMU(settings);                        // create the imu object
  
    if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
    
    Serial.print("TeensyMagCal starting using device "); Serial.println(imu->IMUName());
    Serial.println("Enter s to save current data to SD card");

    imu->setCompassCalibrationMode(true);
    magCal = new RTIMUMagCal(settings);
    magCal->magCalInit();
    lastDisplay = millis();
}

void loop()
{  
    unsigned long now = millis();
  
    if (imu->IMURead()) {                                 // get the latest data
        magCal->newMinMaxData(imu->getIMUData().compass);
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
            Serial.println("-------");
            Serial.print("minX: "); Serial.print(magCal->m_magMin.data(0));
            Serial.print(" maxX: "); Serial.print(magCal->m_magMax.data(0)); Serial.println();
            Serial.print("minY: "); Serial.print(magCal->m_magMin.data(1));
            Serial.print(" maxY: "); Serial.print(magCal->m_magMax.data(1)); Serial.println();
            Serial.print("minZ: "); Serial.print(magCal->m_magMin.data(2));
            Serial.print(" maxZ: "); Serial.print(magCal->m_magMax.data(2)); Serial.println();
        }
    }
  
    if (Serial.available()) {
        if (Serial.read() == 's') {                  // save the data
            magCal->magCalSaveMinMax();
            Serial.print("Mag cal data saved for device "); Serial.println(imu->IMUName());
            while(1) ;
        }
    }
}
