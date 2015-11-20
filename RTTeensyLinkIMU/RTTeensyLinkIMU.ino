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

#include "RTTeensyLinkDefs.h"
#include "RTTeensyLinkEEPROM.h"
#include "RTTeensyLink.h"
#include "RTTeensyLinkUtils.h"
#include "RTTeensyLinkIMUDefs.h"

#include "RTTeensyLinkIMU.h"

RTIMU *imu;                                           // the IMU object
RTIMUSettings *settings;                              // the settings object
RTTeensyLinkIMU linkIMU;                              // the link object
RTTEENSYLINKIMU_MESSAGE linkMessage;                  // the message that is sent to the host

//  SERIAL_PORT_SPEED defines the speed to use for the serial port

#define  SERIAL_PORT_SPEED  115200

void setup()
{
    int errcode;
  
    Serial.begin(SERIAL_PORT_SPEED);
//    while (!Serial) {
//        ; // wait for serial port to connect. 
//    }
    linkIMU.begin(":RTTeensyLinkIMU");
   
    Wire.begin();
    settings = new RTIMUSettings();
    imu = RTIMU::createIMU(settings);                        // create the imu object
   
    Serial.print("TeensyIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
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

    RTTeensyLinkEEPROMDisplay();

    linkIMU.sendDebugMessage("RTTeensyLinkIMU starting");
}

void loop()
{ 
    unsigned char state;
    linkIMU.background();
    if (imu->IMURead()) {                                // get the latest data if ready yet
        // build message
        RTTeensyLinkConvertLongToUC4(millis(), linkMessage.timestamp);
        linkMessage.gyro[0] = imu->getGyro().x();
        linkMessage.gyro[1] = imu->getGyro().y();
        linkMessage.gyro[2] = imu->getGyro().z();
        linkMessage.accel[0] = imu->getAccel().x();
        linkMessage.accel[1] = imu->getAccel().y();
        linkMessage.accel[2] = imu->getAccel().z();
        linkMessage.mag[0] = imu->getCompass().x();
        linkMessage.mag[1] = imu->getCompass().y();
        linkMessage.mag[2] = imu->getCompass().z();
        
        state = 0;
        if (imu->IMUGyroBiasValid())
            state |= RTTEENSYLINKIMU_STATE_GYRO_BIAS_VALID;
        if (imu->getCompassCalibrationValid())
            state |= RTTEENSYLINKIMU_STATE_MAG_CAL_VALID;
            
        // send the message
        linkIMU.sendMessage(RTTEENSYLINK_MESSAGE_IMU, state,
                (unsigned char *)(&linkMessage), sizeof(RTTEENSYLINKIMU_MESSAGE));
   }
}

void RTTeensyLinkIMU::processCustomMessage(unsigned char messageType, unsigned char messageParam,
                unsigned char *data, int length)
{
}

