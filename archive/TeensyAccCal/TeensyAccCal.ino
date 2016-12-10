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
#include "RTIMUAccCal.h"

RTIMU *imu;                                           // the IMU object
RTIMUSettings *settings;                              // the settings object
RTIMUAccCal *accCal;                                  // the mag calibration object
unsigned long lastDisplay;
bool CALIBRATE = false;
RTIMU_DATA imuData;                                   // IMU Data Structure

int inByte = 0;

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
    
    Serial.print("TeensyAccCal starting using device "); Serial.println(imu->IMUName());
    Serial.println("Enter s to save current data.");
    Serial.println("Enter A to activate acceleration calibration.");
    Serial.println("Enter a to stop acceleration calibration.");

    imu->setAccelCalibrationMode(true); 
    accCal = new RTIMUAccCal(settings);
    accCal->accCalInit();
}

void loop()
{  
  
    if (imu->IMURead()) {                                 // get the latest data
        imuData = imu->getIMUData();
		    Serial.println("Enter s to save current data.");
		    Serial.println("Enter A to activate acceleration calibration.");
		    Serial.println("Enter a to stop acceleration calibration.");
        Serial.println("Enter X to clear X acceleration calibration.");
        Serial.println("Enter Y to clear Y acceleration calibration.");
        Serial.println("Enter Z to clear Z acceleration calibration.");
        Serial.println("-Data----");
        Serial.print(RTMath::displayRadians("Gyro [r/s]", imuData.gyro));      // gyro data
        Serial.print(RTMath::displayRadians("Accel  [g]", imuData.accel));     // accel data
        Serial.print(RTMath::displayRadians("Mag   [uT]", imuData.compass));   // compass data
        Serial.println("--Calib--");
        if (settings->m_accelCalValid)
            Serial.println("Accel cal valid");
        else
            Serial.println("No accel cal");
        if (imuData.motion) { Serial.println("Sensor is moving."); } else { Serial.println("Sensor is still."); } // motion
        Serial.print(RTMath::displayRadians("Acc Max", settings->m_accelCalMax ));       // 
        Serial.print(RTMath::displayRadians("Acc Min", settings->m_accelCalMin ));       // 
        Serial.print(RTMath::displayRadians("Mag Max", settings->m_compassCalMax ));     // 
        Serial.print(RTMath::displayRadians("Mag Min", settings->m_compassCalMin ));     // 
        Serial.print(RTMath::displayRadians("Gyro Bias", settings->m_gyroBias ));      // 
        Serial.printf("Declination: %+4.3f \n", (settings->m_compassAdjDeclination/3.141*180.0));
        if (imu->IMUGyroBiasValid())
            Serial.print("Gyro bias valid");
        else
            Serial.print("Calculating gyro bias");
        if (!imu->getCompassCalibrationValid()) {
            if (imu->getRuntimeCompassCalibrationValid())
                Serial.print(", runtime mag cal valid");
            else     
                Serial.print(", runtime mag cal not valid");
        } else {
                Serial.print(", EEPROM mag cal valid");
        }
        if (CALIBRATE)  imu->runtimeAdjustAccelCal(); 
  
      	Serial.printf("%s", CALIBRATE ? ", Calibrating\n" : ", Not Calibrating\n");  
	}
  
    if (Serial.available()) {
        inByte=Serial.read();
        if (inByte == 'a') {                  // runtime calibration off
            CALIBRATE=false;
        }
        if (inByte == 'A') {                  // runtime calibration on
            CALIBRATE=true;
        }
        if (inByte == 'X') {                  // X calibration reset
            accCal->accCalXReset();
        }
        if (inByte == 'Y') {                  // Y calibration rest
            accCal->accCalYReset();
        }
        if (inByte == 'Z') {                  // Z calibration rest
            accCal->accCalZReset();
        }
        if (inByte == 's') {                  // save the data
            accCal->accCalSaveMinMax();
            Serial.print("Acc cal data saved for device "); Serial.println(imu->IMUName());
        }
    }
}
