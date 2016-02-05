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
RTVector3 compass;									  // Compass

unsigned long lastDisplay;
float compassCalOffset[3];
float compassCalScale[3];
float compassCalOffsetAutoTune[3];
float compassCalScaleAutoTune[3];
bool autoTune = false;
bool updateMaxMin = false;

int inByte;

// Field Strength in uT at your location
// http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
#define FIELDSTRENGTH 47.1185
// autoTune learning filter
#define compass_ALPHA 0.01f

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
    float maxDelta = -1;
    float delta;
  
    if (imu->IMURead()) {                                 // get the latest data
        imuData = imu->getIMUData();
		if (updateMaxMin)
			magCal->newMinMaxData(imuData.compass);
			
      if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
        lastDisplay = now;
        Serial.println("-------");
        Serial.println("-u/U disable/enable new Max/Min update");
        Serial.println("-a/A disable/enable autotune");
        Serial.println("-s save regular Max/Min values");
        Serial.println("-S save auto tuned Max/Min values");
        Serial.println("-t transfer regular Max/Min to Autotune");
        Serial.println("-r reset");
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
  
  			maxDelta = -1;
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
  			compass.setX((imuData.compass.x() - compassCalOffset[0]) * compassCalScale[0]);
  			compass.setY((imuData.compass.y() - compassCalOffset[1]) * compassCalScale[1]);
  			compass.setZ((imuData.compass.z() - compassCalOffset[2]) * compassCalScale[2]);
  		
  			Serial.println("-------");
  			Serial.printf("%s", RTMath::displayRadians("Compass Calibrated:", compass));  
  		
  			// attempt Max Min adjustments so that field strength matches local magnetic field length
  			// this adjusts x,y,z components with weight based on the strength of the field in those components
  			// this needs to be run several times until it iterates to wards solution
  			// BETA DOES NOT WORK YET AS IT DOES NOT CONVERGE
  			if (autoTune) {
  				if (updateMaxMin) {
  					Serial.println("You need to turn off Update Max/Min for auto tuning");
  				} else {

            compass.setX((imuData.compass.x() - compassCalOffsetAutoTune[0]) * compassCalScaleAutoTune[0]);
            compass.setY((imuData.compass.y() - compassCalOffsetAutoTune[1]) * compassCalScaleAutoTune[1]);
            compass.setZ((imuData.compass.z() - compassCalOffsetAutoTune[2]) * compassCalScaleAutoTune[2]);

            RTFLOAT l = compass.length();  // This should be same as FIELDSTRENGTH
            RTFLOAT c = (FIELDSTRENGTH / l) - (FIELDSTRENGTH / l / l); // adjust calibration values (empirically)

            // compassCalOffset[0]-compassCalScale[0]

            compassCalScaleAutoTune[0]=(1.0 - compass_ALPHA)*compassCalScaleAutoTune[0] + compass_ALPHA * (compassCalScaleAutoTune[0] *(1.0 + compass.x()/l * c));
            compassCalScaleAutoTune[1]=(1.0 - compass_ALPHA)*compassCalScaleAutoTune[1] + compass_ALPHA * (compassCalScaleAutoTune[1] *(1.0 + compass.y()/l * c));
            compassCalScaleAutoTune[2]=(1.0 - compass_ALPHA)*compassCalScaleAutoTune[2] + compass_ALPHA * (compassCalScaleAutoTune[2] *(1.0 + compass.z()/l * c));
  
  					compass.setX((imuData.compass.x() - compassCalOffsetAutoTune[0]) * compassCalScaleAutoTune[0]);
  					compass.setY((imuData.compass.y() - compassCalOffsetAutoTune[1]) * compassCalScaleAutoTune[1]);
  					compass.setZ((imuData.compass.z() - compassCalOffsetAutoTune[2]) * compassCalScaleAutoTune[2]);
  					Serial.printf("%s", RTMath::displayRadians("Compass Auto Calibrated:", compass));  

  				} // need to have Update Off for Autotune
  			} // autotune
      } // display
    } // imu read
  
    if (Serial.available()) {
        inByte = Serial.read(); 
        if ( inByte == 'u') {                  // update max/min data
            updateMaxMin=false;
        }
        if ( inByte == 'U') {                  // update max/min data
            updateMaxMin=true;
        }
        if ( inByte == 's') {                  // save regular max/min data
            magCal->magCalSaveMinMax();
            Serial.print("Mag cal data saved for device "); Serial.println(imu->IMUName());
        }
        if ( inByte == 'S') {                  // save the autoTune max/min data
			    magCal->m_magMax.setX( magCal->m_magMaxAutoTune.x() );
			    magCal->m_magMax.setY( magCal->m_magMaxAutoTune.y() );
			    magCal->m_magMax.setZ( magCal->m_magMaxAutoTune.z() );
			    magCal->m_magMin.setX( magCal->m_magMinAutoTune.x() );
			    magCal->m_magMin.setY( magCal->m_magMinAutoTune.y() );
			    magCal->m_magMin.setZ( magCal->m_magMinAutoTune.z() );
          magCal->magCalSaveMinMax();
          Serial.print("Mag cal data saved for device "); Serial.println(imu->IMUName());
        }
        if ( inByte == 'r') {                  // reset
            magCal->magCalReset();
        }
        if ( inByte == 'A') {                  // auto tune
            autoTune = true;
        }
        if ( inByte == 'a') {                  // auto tune
            autoTune = false;
        }
        if ( inByte == 't') {                  // auto tune
          magCal->m_magMaxAutoTune.setX( magCal->m_magMax.x());
          magCal->m_magMaxAutoTune.setY( magCal->m_magMax.y());
          magCal->m_magMaxAutoTune.setZ( magCal->m_magMax.z());
          magCal->m_magMinAutoTune.setX( magCal->m_magMin.x());
          magCal->m_magMinAutoTune.setY( magCal->m_magMin.y());
          magCal->m_magMinAutoTune.setZ( magCal->m_magMin.z());
          maxDelta = -1;
          for (int i = 0; i < 3; i++) {
            if ((magCal->m_magMaxAutoTune.data(i) - magCal->m_magMinAutoTune.data(i)) > maxDelta)
            maxDelta = magCal->m_magMaxAutoTune.data(i) - magCal->m_magMinAutoTune.data(i);
          }
          maxDelta /= 2.0f;
          for (int i = 0; i < 3; i++) {
            delta = (magCal->m_magMaxAutoTune.data(i) - magCal->m_magMinAutoTune.data(i)) / 2.0f;
            compassCalScaleAutoTune[i] = maxDelta / delta * (FIELDSTRENGTH/maxDelta);            // makes everything the same range
            compassCalOffsetAutoTune[i] = (magCal->m_magMaxAutoTune.data(i) + magCal->m_magMinAutoTune.data(i)) / 2.0f;
          }
          Serial.println("Copied.");
        }
	} // serial in avail
} // main
