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

//#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "RTIMULib.h"
#include "RTIMUMagCal.h"

RTIMU *imu;                                           // the IMU object
RTIMUSettings *settings;                              // the settings object
RTIMUMagCal *magCal;                                  // the mag calibration object
RTIMU_DATA imuData;                                   // IMU Data Structure
unsigned long lastDisplay;
float compassCalOffset[3];
float compassCalScale[3];
int inByte;

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  50                         // interval between min/max displays

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
        imuData = imu->getIMUData();
        magCal->newMinMaxData(imuData.compass);
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
            Serial.println("-------");
            Serial.printf("%s", RTMath::displayRadians("Compass:", imuData.compass));
            Serial.printf("%s", imuData.motion ? "IMU is moving\n" : "IMU is still \n");  
            Serial.println("-------");
            Serial.print("minX: "); Serial.print(magCal->m_magMin.data(0));
            Serial.print(" maxX: "); Serial.print(magCal->m_magMax.data(0)); Serial.println();
            Serial.print("minY: "); Serial.print(magCal->m_magMin.data(1));
            Serial.print(" maxY: "); Serial.print(magCal->m_magMax.data(1)); Serial.println();
            Serial.print("minZ: "); Serial.print(magCal->m_magMin.data(2));
            Serial.print(" maxZ: "); Serial.print(magCal->m_magMax.data(2)); Serial.println();

            float maxDelta = -1;
            float delta;
            for (int i = 0; i < 3; i++) {
              if ((magCal->m_magMax.data(i) - magCal->m_magMin.data(i)) > maxDelta)
                maxDelta = magCal->m_magMax.data(i) - magCal->m_magMin.data(i);
            }
            maxDelta /= 2.0f;
            for (int i = 0; i < 3; i++) {
              delta = (magCal->m_magMax.data(i) - magCal->m_magMin.data(i)) / 2.0f;
              compassCalScale[i] = maxDelta / delta;            // makes everything the same range
              compassCalOffset[i] = (magCal->m_magMax.data(i) + magCal->m_magMin.data(i)) / 2.0f;
            }
            imuData.compass.setX((imuData.compass.x() - compassCalOffset[0]) * compassCalScale[0]);
            imuData.compass.setY((imuData.compass.y() - compassCalOffset[1]) * compassCalScale[1]);
            imuData.compass.setZ((imuData.compass.z() - compassCalOffset[2]) * compassCalScale[2]);
            Serial.println("-------");
            Serial.printf("%s", RTMath::displayRadians("Compass Calibrated:", imuData.compass));  
        }
    }
  
    if (Serial.available()) {
        inByte = Serial.read(); 
        if ( inByte == 's') {                  // save the data
            magCal->magCalSaveMinMax();
            Serial.print("Mag cal data saved for device "); Serial.println(imu->IMUName());
        }
        if ( inByte == 'r') {                  // reset
            magCal->magCalReset();
        }

    }
}
