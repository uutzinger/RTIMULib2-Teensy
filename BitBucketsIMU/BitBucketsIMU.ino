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
#include "utility/RTPressure.h"
#include "utility/RTHumidity.h"

RTIMU *imu;                                           // the IMU object
RTPressure *pressure;                                 // the pressure object
RTHumidity *humidity;                                 // the humidity object
RTIMUSettings *settings;                              // the settings object
RTIMU_DATA imuData;                                   // IMU Data Structure

//  DISPLAY_INTERVAL sets the rate at which results are transferred to host
#define DISPLAY_INTERVAL      35000                   // interval in microseconds between data updates over serial port: 100000=10Hz, Fusion update is 50Hz (20000), lowpass is 40Hz
#define CHECKINPUT_INTERVAL   35000                   // 35000 will result in 40ms update time

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  250000

unsigned long lastDisplay;
unsigned long lastRate;
unsigned long lastInAvail;
unsigned long lastTimestamp;
unsigned long motionstart;
int dt;       // time in between samples
int dtmotion; // time during which motion occured
int sampleCount;
int sampleRate;

// Controlling Reporting and Input Output
bool REPORT=false;                                    // do we need to produce output data?
bool STREAM=false;                                    // are we continously streaming output?
bool INAVAIL=false;                                   // do we need to check for input ?
int  inByte = 0;                                      // serial input buffer, one byte

// Fusion Status
byte fusionStatus = B00000000;				               // Represents the enabled components of the fusion algorithm
float slerpPower = 0.02;                             // Fusion convergence
// devices used for fusion 
#define STATE_FUSION_GYROENABLE    0                  // Use Gyroscope for Fusion
#define STATE_FUSION_COMPASSENABLE 1                  // Use Compass for Fusion
#define STATE_FUSION_ACCELENABLE   2                  // Use Accelerator for Fusion

// IMU Status
byte systemStatusIMU = B00000000;
#define STATE_FUSIONPOSEVALID    0
#define STATE_FUSIONQPOSEVALID   1
#define STATE_GYROVALID          2
#define STATE_ACCELVALID         3
#define STATE_COMPASSVALID       4
#define STATE_IMUTEMPVALID       5

// AUX Status
byte systemStatusAUX = B00000000;
#define STATE_PRESSUREVALID      0
#define STATE_PRESSURETEMPVALID  1
#define STATE_HUMIDITYVALID      2
#define STATE_HUMIDITYTEMPVALID  3

// Position Sensing
float accScale = 9.81;                                       // conversion to 9.81 m/s/s
RTVector3 residuals;                                  // gravity subtracted acceleration
RTVector3 residuals_bias;                             // average acceleration when device is stationary
RTVector3 residuals_sum;                              // for computing average acceleration
RTVector3 worldResiduals;                             // residuals in the world coordinate system
RTVector3 worldResiduals_previous;                    // for trapezoidal integration of acceleration
RTVector3 worldVelocity;                              // integrated acceleration
RTVector3 worldVelocity_bias;                         // velocity bias computed at end of motion
RTVector3 worldVelocity_previous;                     // for trapezoidal integration of velocity
RTVector3 worldLocation;                              // integrated velocity
RTQuaternion gravity;
unsigned long residualsCount = 0;                     // for bias accumulation
bool lastmotion=false, motionstarted=false, motionended=false;

// Heading
float heading;                                        // tilt compensated heading
RunningAverage heading_avg(5);	                      // Running average for heading (noise reduction)

void setup()
{
    int errcode;
  
    Serial.begin(SERIAL_PORT_SPEED);
    while (!Serial) {
        ; // wait for serial port to connect. 
    }
    Wire.begin();
    settings = new RTIMUSettings();
    // change configurations here if you need to as there is no SD card and therefore no ini file.
    
    imu = RTIMU::createIMU(settings);                        // create the imu object
    Serial.print("TeensyIMU starting using device "); Serial.println(imu->IMUName());

    pressure = RTPressure::createPressure(settings);        // create the pressure sensor
    if (pressure == 0) {
        Serial.println("No pressure sensor has been configured."); 
    } else {
        Serial.println(", pressure sensor "); Serial.print(pressure->pressureName());
        if ((errcode = pressure->pressureInit()) < 0) {
            Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);
        }
    }

    humidity = RTHumidity::createHumidity(settings);        // create the pressure sensor
    if (humidity == 0) {
        Serial.println("No humidity sensor has been configured."); 
    } else {
      Serial.println(", humidity sensor "); Serial.println(humidity->humidityName());
      if ((errcode = humidity->humidityInit()) < 0) {
          Serial.print("Failed to init humidity sensor: "); Serial.println(errcode);
      }
    }

    if ((errcode = imu->IMUInit()) < 0) { Serial.print("Failed to init IMU: "); Serial.println(errcode); }

    if (imu->getCompassCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");
		
    ////////////////////////////////////////////////////////////////////////////////////////
    //// set up any fusion parameters here
  	////////////////////////////////////////////////////////////////////////////////////////
    imu->setSlerpPower(slerpPower);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);
	
	  setBit(fusionStatus, (byte)STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
	  setBit(fusionStatus, (byte)STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
	  setBit(fusionStatus, (byte)STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );

	  // House keeping & Initializing
    lastDisplay = lastRate = lastTimestamp = lastInAvail = micros();
    sampleCount = 0;
	  residuals.zero();
	  worldResiduals.zero();
	  worldResiduals_previous.zero();
	  worldVelocity.zero();
  	worldVelocity_previous.zero();
	  worldVelocity_bias.zero();
	  heading_avg.clear();
		accScale = 9.81f; 

    gravity.setScalar(0);
    gravity.setX(0);
    gravity.setY(0);
    gravity.setZ(1);
}

void loop()
{  
    unsigned long now = micros();
    
    RTQuaternion rotatedGravity;
    RTQuaternion fusedConjugate;
    RTQuaternion qTemp;
    
    if (imu->IMURead()) {                       // get the latest data if any avail
        imuData = imu->getIMUData();
        sampleCount++;
        dt=imuData.timestamp-lastTimestamp;
        lastTimestamp = imuData.timestamp;

        ////////////////////////////////////////////////////////////////////////////////
        // Motion Computations
        ////////////////////////////////////////////////////////////////////////////////

        //  do gravity rotation and subtraction
        fusedConjugate = imuData.fusionQPose.conjugate(); // create the conjugate of the pose
        qTemp = gravity * imuData.fusionQPose; // now do the rotation - takes two steps with qTemp as the intermediate variable
        rotatedGravity = fusedConjugate * qTemp;
        // now adjust the measured accel and change the signs to make sense
        residuals.setX(-(imuData.accel.x() - rotatedGravity.x()));
        residuals.setY(-(imuData.accel.y() - rotatedGravity.y()));
        residuals.setZ(-(imuData.accel.z() - rotatedGravity.z()));
        residuals = residuals * accScale; // create physical values

        // At end of a motion period, acceleration needs to be zero
        // During no motion, acceleration should be zero
        motionstarted = (imuData.motion && !lastmotion); // is 1 if motion started
        motionended   = (!imuData.motion && lastmotion); // is 1 if motion ended
        lastmotion = imuData.motion;
        if (motionstarted) {motionstart = imuData.timestamp;}
        // Accumulate residual acceleration during no motion to compute an acceleration bias
        if (imuData.motion == false) {                  // if no motion, accumulate residual motion for background acceleration computation
          residuals_sum  = residuals_sum + residuals;
          residualsCount++; 
        } // end motion
        if ((motionstarted) || (residualsCount >=50))  { // freeze background acceleration accumulation when motion starts
          if (residualsCount > 10) {                     // We want at least 10 useful background readings
            residuals_bias = residuals_sum / ((float)residualsCount); // update average residual acceleration
            // lowpass filter the bias
          }
          residualsCount=0;                             // Reset background accumulation
          residuals_sum.zero();
        }
        // subtract acceleration bias from residuals 
        residuals = (residuals - residuals_bias); // subtract acceleration bias

        // Convert data into World Coordinate System
        // Compute Velocity
        worldResiduals = RTMath::toWorld(residuals, imuData.fusionQPose); // rotate residuals to world coordinate system
        // integrate acceleration and add to velocity (uses trapezoidal intergration technique
        worldVelocity = worldVelocity_previous + ((worldResiduals + worldResiduals_previous)*0.5f*(float(dt)  * 0.000001f)); // dt is in microseconds 
        // Update Velocity Bias
        // When motion ends, velocity should be zero
        if (motionended) {
          dtmotion = (float(imuData.timestamp-motionstart))/1.0E6; // in seconds
          if (dtmotion > 0.5f) { // update velocity bias if we had at least half of second motion
            worldVelocity_bias=worldVelocity/dtmotion;
            // could use some learning averaging here with previous values and weigthing long motions more than short ones
          }
        }
        // Update Velocity
        worldVelocity = worldVelocity - ( worldVelocity_bias * (float(dt) * 0.000001f) );
        // Avoid Velcity Bias
        if (imuData.motion == false) {     // its not moving
          worldVelocity.zero();    // minimize error propagation
        }

        // Compute Position
        worldLocation = worldLocation + ((worldVelocity + worldVelocity_previous)*0.5f)*((float)dt)*0.000001f;

        // Reset previous values
        worldResiduals_previous = worldResiduals;
        worldVelocity_previous  = worldVelocity;

    		// Do we want to update sampling rate?
        if ((now - lastRate) >= 1000000) { // 1 second
            sampleRate=sampleCount;
            sampleCount = 0;
            lastRate = now;
        }
		
		    // Do we want to display data?
        if ((now-lastDisplay) >= DISPLAY_INTERVAL) {
			    lastDisplay = now;
			    if (STREAM) { REPORT = true; } else { REPORT = false; }
        }
		
        // Do we want to check input ?
        if ((now-lastInAvail) >= CHECKINPUT_INTERVAL) {
          lastInAvail = now;
          INAVAIL = true;
        } else {
          INAVAIL = false;
        }

    		///////////////////////////////////////////////////////////////
		    // Data Reporting
		    ///////////////////////////////////////////////////////////////
        if (REPORT) {
            Serial.print("Sample rate: "); Serial.print(sampleRate);
            if (imu->IMUGyroBiasValid())
                Serial.println(", gyro bias valid");
            else
                Serial.println(", calculating gyro bias");
        
            if (!imu->getCompassCalibrationValid()) {
                if (imu->getRuntimeCompassCalibrationValid())
                    Serial.println(", runtime mag cal valid");
                else     
                    Serial.println(", runtime mag cal not valid");
            }
            Serial.print(RTMath::displayRadians("Gyro:", imuData.gyro));       // gyro data
            Serial.print(RTMath::displayRadians("Accel:", imuData.accel));     // accel data
            Serial.print(RTMath::displayRadians("Mag:", imuData.compass));     // compass data
            Serial.print(RTMath::displayDegrees("Pose:", imuData.fusionPose)); // fused output
            
            Serial.print(RTMath::displayRadians("Residuals:", residuals));                    // Residuals in device coordiante system
            Serial.print(RTMath::displayRadians("Residuals Bias:", residuals_bias));          // Residuals bias in device coordiante system
            Serial.print(RTMath::displayRadians("World Residuals:", worldResiduals));         // Residuals in device world coordiante system
            Serial.print(RTMath::displayRadians("World Velocity:", worldVelocity));           // Velocity in world coordiante system
            Serial.print(RTMath::displayRadians("World Velocity Bias:", worldVelocity_bias)); // Velocity bias in world coordiante system
            Serial.print(RTMath::displayRadians("World Position:", worldLocation));           // Location in world coordiante system
            
            if (imuData.motion) { Serial.println("Sensor is moving."); } else { Serial.println("Sensor is still."); } // motion

            Serial.printf("IMU_T %x, ", imuData.IMUtemperature);
            Serial.printf("P %x, ",     imuData.pressure);
            Serial.printf("P_T%x, ",    imuData.pressureTemperature);
            Serial.printf("H %x, ",     imuData.humidity);
            Serial.printf("H_T %x ",    imuData.humidityTemperature);
            Serial.println();
			
      			Serial.printf("IMUP:  %b ", getBit(systemStatusIMU, STATE_FUSIONPOSEVALID));
      			Serial.printf("IMUQP: %b ", getBit(systemStatusIMU, STATE_FUSIONQPOSEVALID));
      			Serial.printf("IMUG:  %b ", getBit(systemStatusIMU, STATE_GYROVALID));
      			Serial.printf("IMUA:  %b ", getBit(systemStatusIMU, STATE_ACCELVALID));
      			Serial.printf("IMUC:  %b ", getBit(systemStatusIMU, STATE_COMPASSVALID));
      			Serial.printf("IMUT:  %b ", getBit(systemStatusIMU, STATE_IMUTEMPVALID));
      			Serial.println();
      			Serial.printf("AUXP:  %b ", getBit(systemStatusAUX, STATE_PRESSUREVALID));
      			Serial.printf("AUXPT: %b ", getBit(systemStatusAUX, STATE_PRESSURETEMPVALID));
      			Serial.printf("AUXH:  %b ", getBit(systemStatusAUX, STATE_HUMIDITYVALID));
      			Serial.printf("AUXHT: %b ", getBit(systemStatusAUX, STATE_HUMIDITYTEMPVALID));
      			Serial.println();
      			Serial.printf("G:     %b ", getBit(fusionStatus, STATE_FUSION_GYROENABLE));
      			Serial.printf("A:     %b ", getBit(fusionStatus, STATE_FUSION_ACCELENABLE));
      			Serial.printf("C:     %b ", getBit(fusionStatus, STATE_FUSION_COMPASSENABLE));
      			Serial.println();

        } // report
    } // imuread
	
    ///////////////////////////////////////////////////////////////
    // Input Commands
    ///////////////////////////////////////////////////////////////
    if (INAVAIL) {
  		if (Serial.available()) {
  			inByte=Serial.read();
  			// ENABLE/DISABLE FUSION ALGORITHM INPUTS
  			if (inByte == 'm') {      // turn off Compass Fusion
  				imu->setCompassEnable(false);;
  				setBit(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
  			} else if (inByte == 'M') { // turn on  Compass Fusion
  				imu->setCompassEnable(true);
  				setBit(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
  			} else if (inByte == 'a') {      // turn off Accelerometer Fusion
  				imu->setAccelEnable(false);
  				setBit(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
  			} else if (inByte == 'A') { // turn on Accelerometer Fusion
  				imu->setAccelEnable(true);
  				setBit(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
  			} else if (inByte == 'g') {      // turn off Gyroscope Fusion
  				imu->setGyroEnable(false);
  				setBit(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
  			} else if (inByte == 'G') { // turn on Gyroscope Fusion
  				imu->setGyroEnable(true);
  				setBit(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
  			} else if (inByte == 's') { // turn off streaming
  				STREAM=false;
  			} else if (inByte == 'S') { // turn on  streaming
  				STREAM=true;
  			} else if (inByte == 'K') {
  				//calibrateCompass(); 
  				// Needs to be written to enable auto calibration and override EEPROM settings
  			} else if (inByte == 'k') {
  				//Save and Stop Compass Calibration(); 
  				// Needs to be written to stop and save calibration to EEPROM
  			} else if (inByte == 'C') {
  				//calibrateAccel());  
  				// Needs to be written to enable accel calibration during no motion
  			} else if (inByte == 'c') {
  				//save and stop accel calibration());  
  				// Needs to be written to stop accel calibration and save data
  			} else if (inByte == 'R') { // send raw
  				HEXsendRAW();
  			} else if (inByte == 'r') { // send residuals
  				HEXsendR();
  			} else if (inByte == 'e') { // send Euler
  				HEXsendE();
  			} else if (inByte == 'q') { // send Quaternion
  				HEXsendQ();
  			} else if (inByte == 'h') { // send residuals
  				HEXsendH();
  			} else if (inByte == 'p') { // send location
  				HEXsendWP();
  			} else if (inByte == 'v') { // send velocity
  				HEXsendWV();
  			} else if (inByte == 'P') { // reset position
  				worldLocation.zero();
  			} else if (inByte == '?') { // send STATE information
  				HEXsendIMUStatus();
  				HEXsendFusionStatus();
  			}
  		} // end if serial input available
    } // end INAVAIL
} // end of main loop

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

// REPORTING OF VARIABLES

void HEXsendIMUStatus(){
	// System Status
	systemStatusIMU = B00000000;
	systemStatusAUX = B00000000;

    //--IMU
	if (imuData.fusionPoseValid) { setBit(systemStatusIMU, STATE_FUSIONPOSEVALID, true); }
	if (imuData.fusionQPoseValid) { setBit(systemStatusIMU, STATE_FUSIONQPOSEVALID, true); }
	if (imuData.gyroValid) { setBit(systemStatusIMU, STATE_GYROVALID, true); }
	if (imuData.accelValid) { setBit(systemStatusIMU, STATE_ACCELVALID, true); }
	if (imuData.accelValid) { setBit(systemStatusIMU, STATE_COMPASSVALID, true); }
	if (imuData.IMUtemperatureValid) { setBit(systemStatusIMU, STATE_IMUTEMPVALID, true); }
    //--Aux
	if (imuData.pressureValid) { setBit(systemStatusAUX, STATE_PRESSUREVALID, true); }
	if (imuData.pressureTemperatureValid) { setBit(systemStatusAUX, STATE_PRESSURETEMPVALID, true); }
	if (imuData.humidityValid) { setBit(systemStatusAUX, STATE_HUMIDITYVALID, true); }
	if (imuData.humidityTemperatureValid) { setBit(systemStatusAUX, STATE_HUMIDITYTEMPVALID, true); }

	serialBytePrint(systemStatusIMU); Serial.print(',');
	serialBytePrint(systemStatusAUX); Serial.print(',');
}

void HEXsendFusionStatus(){
// send Fusion Status
// STATE_FUSION_GYROENABLE    0
// STATE_FUSION_COMPASSENABLE 1
// STATE_FUSION_ACCELENABLE   2
	serialBytePrint(fusionStatus); Serial.print(',');
}

void HEXsendRAW() {
  serialFloatPrint(imuData.accel.x()); Serial.print(',');
  serialFloatPrint(imuData.accel.y()); Serial.print(',');
  serialFloatPrint(imuData.accel.z()); Serial.print(',');
  serialFloatPrint(imuData.gyro.x());  Serial.print(',');
  serialFloatPrint(imuData.gyro.y());  Serial.print(',');
  serialFloatPrint(imuData.gyro.z());  Serial.print(',');
  serialFloatPrint(imuData.compass.x()); Serial.print(',');
  serialFloatPrint(imuData.compass.y()); Serial.print(',');
  serialFloatPrint(imuData.compass.z()); Serial.print(',');
}

void HEXsendQ() {
  serialFloatPrint(imuData.fusionQPose.scalar()); Serial.print(',');
  serialFloatPrint(imuData.fusionQPose.x()); Serial.print(',');
  serialFloatPrint(imuData.fusionQPose.y()); Serial.print(',');
  serialFloatPrint(imuData.fusionQPose.z()); Serial.print(',');
}

void HEXsendE() {
  serialFloatPrint(imuData.fusionPose.x()); Serial.print(',');
  serialFloatPrint(imuData.fusionPose.y()); Serial.print(',');
  serialFloatPrint(imuData.fusionPose.z()); Serial.print(',');
}

void HEXsendR() {
  serialFloatPrint(residuals.x()); Serial.print(',');
  serialFloatPrint(residuals.y()); Serial.print(',');
  serialFloatPrint(residuals.z()); Serial.print(',');
}

void HEXsendH() {
  serialFloatPrint(heading); Serial.print(',');
}

void HEXsendWR() {
  serialFloatPrint(worldResiduals.x()); Serial.print(',');
  serialFloatPrint(worldResiduals.y()); Serial.print(',');
  serialFloatPrint(worldResiduals.z()); Serial.print(',');
}

void HEXsendWP() {
  serialFloatPrint(worldLocation.x()); Serial.print(',');
  serialFloatPrint(worldLocation.y()); Serial.print(',');
  serialFloatPrint(worldLocation.z()); Serial.print(',');
}

void HEXsendWV() {
  serialFloatPrint(worldVelocity.x()); Serial.print(',');
  serialFloatPrint(worldVelocity.y()); Serial.print(',');
  serialFloatPrint(worldVelocity.z()); Serial.print(',');
}

// CONVERT FLOAT TO HEX AND SEND OVER SERIAL PORT
void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}

// CONVERT Byte TO HEX AND SEND OVER SERIAL PORT
void serialBytePrint(byte b) {
    
    byte b1 = (b >> 4) & 0x0f;
    byte b2 = (b & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
}

boolean getBit(byte myVarIn, byte whatBit) {
  boolean bitState;
  bitState = myVarIn & (1 << whatBit);
  return bitState;
}

byte setBit(byte myVarIn, byte whatBit, boolean s) {
  if (s) {
    myVarIn = myVarIn | (1 << whatBit);
  } 
  else {
    myVarIn = myVarIn & ~(1 << whatBit);
  }
  return myVarIn;
}
