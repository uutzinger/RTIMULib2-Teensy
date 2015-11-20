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
#include "utility/RTPressure.h"



RTIMU *imu;                                           // the IMU object
RTPressure *pressure;                                 // the pressure object
RTIMUSettings *settings;                              // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  300                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

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
    
    pressure = RTPressure::createPressure(settings);        // create the pressure sensor
    
    if (pressure == 0) {
        Serial.println("No pressure sensor has been configured - terminating"); 
        while (1) ;
    }
  
    Serial.print("TeensyIMU10 starting using device "); Serial.print(imu->IMUName());
    Serial.print(", pressure sensor "); Serial.println(pressure->pressureName());
    if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if ((errcode = pressure->pressureInit()) < 0) {
        Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);
    }

    if (imu->getCompassCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");

    // set up any fusion parameters here
    
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    lastDisplay = lastRate = millis();
    sampleCount = 0;
}

void loop()
{  
    unsigned long now = millis();
    unsigned long delta;
    RTIMU_DATA imuData;
  
    if (imu->IMURead()) {                                // get the latest data if ready yet
        imuData = imu->getIMUData();
        sampleCount++;
        if ((delta = now - lastRate) >= 1000) {
            Serial.print("Sample rate: "); Serial.print(sampleCount);
            if (imu->IMUGyroBiasValid())
                Serial.print(", gyro bias valid");
            else
                Serial.print(", calculating gyro bias");
       
            if (!imu->getCompassCalibrationValid()) {
                if (imu->getRuntimeCompassCalibrationValid())
                    Serial.print(", runtime mag cal valid");
                else     
                    Serial.print(", runtime mag cal not valid");
            }
            Serial.println();
        
            sampleCount = 0;
            lastRate = now;
        }
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
//            Serial.print(RTMath::displayRadians("Gyro:", imuData.gyro));       // gyro data
//            Serial.print(RTMath::displayRadians("Accel:", imuData.accel));    // accel data
//            Serial.print(RTMath::displayRadians("Mag:", imuData.compass));     // compass data
            Serial.print(RTMath::displayDegrees("Pose:", imuData.fusionPose)); // fused output
            if (pressure->pressureRead(imuData)) {
                if (imuData.pressureValid)
                    Serial.print(", pressure: "); Serial.print(imuData.pressure);
                if (imuData.temperatureValid)
                    Serial.print(", temperature: "); Serial.print(imuData.temperature);
            }
            Serial.println();
        }
    }
}

