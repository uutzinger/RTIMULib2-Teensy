////////////////////////////////////;////////////////////////////////////////
//
//  This file is part of RTIMULib2-Teensy
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
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
RTPressure    *pressure;                              // the pressure object
RTHumidity    *humidity;                              // the humidity object
RTIMUSettings *settings;                              // the settings object
RTIMU_DATA     imuData;                               // IMU Data Structure
PRESSURE_DATA  pressureData;                          // Pressure Data Structure
HUMIDITY_DATA  humidityData;                          // Humidity Data Structure

//  DISPLAY_INTERVAL sets the rate at which results are transferred to host
#define DISPLAY_INTERVAL      100                   // interval in microseconds between data updates over serial port: 100000=10Hz, Fusion update is 80Hz, lowpass is 40Hz
#define CHECKINPUT_INTERVAL   50000                 // 35000 will result in 40ms update time
#define LEDBLINK_INTERVAL     100000                // 100ms

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  250000

unsigned long IMUpollInterval = 1;
unsigned long pressurePollInterval = 1;
unsigned long humidityPollInterval = 1;

unsigned long lastDisplay;
unsigned long lastRate;
unsigned long lastInAvail;
unsigned long lastTimestamp;
unsigned long motionstart;
unsigned long lastBlink;
unsigned long lastPoll;
unsigned long lastIMUPoll;
unsigned long lastHumidityPoll;
unsigned long lastPressurePoll;
unsigned long timeout;
unsigned long currentTime;

int dt;       // time in between samples
int dtmotion; // time during which motion occured

int IMUsampleCount;
int IMUsampleRate;
int pressureSampleCount;
int pressureSampleRate;
int humiditySampleCount;
int humiditySampleRate;

// Controlling Reporting and Input Output
bool REPORT=false;                                    // do we need to produce output data?
bool STREAM=true;                                     // are we continously streaming output?
bool INAVAIL=false;                                   // is it time to check for user input ?
bool VERBOSE=false;                                   // set HEX or text mode
bool CUBE=false;                                      // set output format for 4183 Robot or CUBE display program
bool ACCELCALIBRATE = false;                          // enable runtime accel calibration
bool SERIAL_REPORTING = false; 	          					  // minimal output when boot
bool COMPASS_ENABLE = false;   					              // boot conditions for compass
bool PRESSURE_ENABLE = false;                         // disable pressure detection
bool HUMIDITY_ENABLE = false;                         // disable humidity detection
bool DEBUG_ENABLE = false;                            // No debug fusion parameters

// CALIBRATION STATUS
bool GYRO_CALIBRATED = false;                         // This will change to true if IMU had been still and gyro amplitude was below 0.003 once. 
                                                      // if GYRO_CALIBRATED is false that means the GYRO bias is not properly calibrated 
// FUSION CONTROL
float slerpPower = 0.02;                              // Fusion convergence

// Indicators
const int ledPin = 13;                                // Check on https://www.pjrc.com/teensy/pinout.html; pin should not interfere with I2C and SPI
bool ledStatus = false;                               // Led should be off at start up

// Serial
int  inByte = 0;                                      // serial input buffer, one byte

// Fusion Status bits
byte fusionStatus = B00000000;                       // Represents the enabled components of the fusion algorithm
// devices used for fusion 
#define STATE_FUSION_GYROENABLE    0                  // Use Gyroscope for Fusion
#define STATE_FUSION_COMPASSENABLE 1                  // Use Compass for Fusion
#define STATE_FUSION_ACCELENABLE   2                  // Use Accelerator for Fusion
#define STATE_FUSION_MOVING        7                  // System is moving

// IMU Status bits
byte systemStatusIMU = B00000000;
#define STATE_FUSIONPOSEVALID       0
#define STATE_FUSIONQPOSEVALID      1
#define STATE_GYROVALID             2
#define STATE_ACCELVALID            3
#define STATE_COMPASSVALID          4
#define STATE_IMUTEMPVALID          5

// IMU Update bits
byte systemUpdateIMU = B00000000;
#define STATE_ACCELRUNTIMEUPDATE    4
#define STATE_GYROMANUALUPDATE      5
#define STATE_GYRORUNTIMEUPDATE     6
#define STATE_COMPASSRUNTIMEUPDATE  7

// AUX Status bits
byte systemStatusAUX = B00000000;
#define STATE_PRESSUREVALID      0
#define STATE_PRESSURETEMPVALID  1
#define STATE_HUMIDITYVALID      2
#define STATE_HUMIDITYTEMPVALID  3

// Position Sensing, Physical Variables
const float accScale = 9.81f;                         // conversion to 9.81 m/s/s
const float magFieldNorm=47.12f;                      // conversion to microT
float magFieldNormScale =1.0f;                        // to adjust current magnetometer readings
float compassHeading;                                 // tilt compensated heading
float staticPressure = 1013.25f;
float pressure_avg = 1013.25f;
float humidity_avg = 0.0f;
// Motion variables
float lastCompassHeading;
float compassHeadingDelta;
float lastYaw;
float yawDelta;
float previousAccelLength;
float previousGyroLength;
float previousCompassLength;

RunningAverage compassHeading_avg(5);                 // Running average for heading (noise reduction)
RunningAverage residuals_avg(25);                     // Running average for residuals (debug)
RunningAverage gyro_avg(25);                          // Running average for gyro (debug)
RunningAverage compass_avg(25);                       // Running average for compass (debug)
RunningAverage accel_avg(25);                         // Running average for acceleration (debug)
RTQuaternion gravity;
RTVector3 tempVec;
RTVector3 comp;                                       // average earth field strength measured at startup
RTVector3 residuals;                                  // gravity subtracted acceleration
RTVector3 residualsBias;                              // average acceleration when device is stationary
RTVector3 residualsBiasTemp;                          // for computing average acceleration
RTVector3 residualsBiasCandidate;                     // for computing average acceleration
int intervalCount = 0;                                // interval loop o updating residual bias
bool firstTime = true;                                // first interval in residual calculations
RTVector3 worldResiduals;                             // residuals in the world coordinate system
RTVector3 worldResidualsPrevious;                     // for trapezoidal integration of acceleration
RTVector3 worldVelocity;                              // integrated acceleration
RTVector3 worldVelocityBias;                          // velocity bias computed at end of motion
RTVector3 worldVelocityPrevious;                      // for trapezoidal integration of velocity
RTVector3 worldLocation;                              // integrated velocity

bool lastMotion=false, motionStarted=false, motionEnded=false;

void setup()
{
  int errcode;

  // Setup monitor pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  ledStatus = false;
        
  Serial.begin(SERIAL_PORT_SPEED);
  while (!Serial) {
      ; // wait for serial port to connect. 
  }
  Wire.begin();
  settings = new RTIMUSettings();
  // change configurations here if you need to as there is no SD card and therefore no ini file.
  settings->setDeclination(-9.97f*3.141f/180.0f);           // Magnetic Declination in Tucson AZ
    
  imu = RTIMU::createIMU(settings);                        // create the imu object
  imu->setGyroRunTimeCalibrationEnable(false);             // turn off gyro bias calibration at startup, allow system to equiblirate first
  
  if (SERIAL_REPORTING) {Serial.print("TeensyIMU starting using device "); Serial.println(imu->IMUName());}

	if (PRESSURE_ENABLE == true) {
		pressure = RTPressure::createPressure(settings);        // create the pressure sensor
		if (pressure == 0) {
			if (SERIAL_REPORTING) {Serial.println("No pressure sensor has been configured."); }
		} else {
			if (SERIAL_REPORTING) {Serial.println(", pressure sensor "); Serial.print(pressure->pressureName());}
			if ((errcode = pressure->pressureInit()) < 0) {
				if (SERIAL_REPORTING) {Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);}
			} else {
			    pressurePollInterval = pressure->pressureGetPollInterval() * 1000;
            }
		}
	} // pressure

  if (HUMIDITY_ENABLE == true) {
		humidity = RTHumidity::createHumidity(settings);        // create the pressure sensor
		if (humidity == 0) {
			if (SERIAL_REPORTING) { Serial.println("No humidity sensor has been configured."); }
		} else {
		  if (SERIAL_REPORTING) { Serial.println(", humidity sensor "); Serial.println(humidity->humidityName()); }
		  if ((errcode = humidity->humidityInit()) < 0) {
			  if (SERIAL_REPORTING) { Serial.print("Failed to init humidity sensor: "); Serial.println(errcode);} 
		  } else {
              humidityPollInterval = humidity->humidityGetPollInterval() * 1000;
		  }
		}
	} // humidity

  if ((errcode = imu->IMUInit()) < 0) { 
       if (SERIAL_REPORTING) { Serial.print("Failed to init IMU: "); }
	     if (SERIAL_REPORTING) { Serial.println(errcode); } 
	}
  if (imu->getCompassCalibrationValid()) {
    if (SERIAL_REPORTING) {Serial.println("Using compass calibration");}
  } else {
    if (SERIAL_REPORTING) {Serial.println("No valid compass calibration data");}
  }
  if (imu->getAccelCalibrationValid()) {
    if (SERIAL_REPORTING) { Serial.println("Using accel calibration"); }
  } else {
    if (SERIAL_REPORTING) { Serial.println("No valid accel calibration data"); }
  }
  if (imu->getGyroCalibrationValid()) {
    if (SERIAL_REPORTING) { Serial.println("Using gyro calibration"); }
  } else {
    if (SERIAL_REPORTING) { Serial.println("No valid gyro calibration data"); }
  }
  ////////////////////////////////////////////////////////////////////////////////////////
  //// set up any fusion parameters here
  ////////////////////////////////////////////////////////////////////////////////////////
  imu->setSlerpPower(slerpPower);
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  // imu->setCompassEnable(COMPASS_ENABLE);
  imu->setCompassEnable(false);
  imu->setDebugEnable(DEBUG_ENABLE);
  
  IMUpollInterval = imu->IMUGetPollInterval() * 1000;
		
  //--Fusion
  bitWrite(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
  bitWrite(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
  bitWrite(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
  bitWrite(fusionStatus, STATE_FUSION_MOVING,        false );
    
  // House keeping & Initializing
  lastDisplay = lastRate = lastTimestamp = lastInAvail = micros();
  IMUsampleCount = 0;
  pressureSampleCount = 0;
  humiditySampleCount = 0;
  residuals.zero();
  worldResiduals.zero();
  worldResidualsPrevious.zero();
  worldVelocity.zero();
  worldVelocityPrevious.zero();
  worldVelocityBias.zero();
  compassHeading_avg.clear();

  residualsBias.zero();                             // average acceleration when device is stationary

  gravity.setScalar(0);
  gravity.setX(0);
  gravity.setY(0);
  gravity.setZ(1);

  // dry run of the system
  if (SERIAL_REPORTING) { Serial.println("Dry run of system."); }
  int i=0;
  while (i < 80) {
    currentTime = micros();
    int pollDelay = ((int) IMUpollInterval - (int)(currentTime - lastPoll));
    if (pollDelay > 0) { delayMicroseconds(pollDelay); }
    lastPoll = currentTime;
    if  (imu->IMURead()) { i++; }
    if (humidity != NULL) { humidity->humidityRead(); }
    if (pressure != NULL) { pressure->pressureRead(); }
  }
  
  pressureData = pressure->getPressureData();
  if (pressureData.pressureValid) {staticPressure = pressureData.pressure;}
	
  imu->setGyroRunTimeCalibrationEnable(true);     // enable background gyro calibration

  // Compute normalization factor for earts magnetic field
  comp.zero();
  i=0;
  while (i < 50) {
    currentTime = micros();
    int pollDelay = (IMUpollInterval - (int)(currentTime - lastPoll));
    if (pollDelay > 0) { delayMicroseconds(pollDelay); }
    lastPoll = currentTime;
    if  (imu->IMURead()) {                       // get the latest calibrated data 
      imuData = imu->getIMUData();
      comp = comp + imuData.compass;             // get compass
      i++;
    }
  }
  comp = comp / 50.0f; // compute average compass (devide by 50)
  magFieldNormScale=magFieldNorm/comp.length();
  
  // Ready for input commands
  if (SERIAL_REPORTING) { Serial.println("Send S to start streaming."); }

  // LED blinking status
  digitalWrite(ledPin, HIGH); // initialization completed
  ledStatus = true;
  lastBlink = micros();

  // setup timers
  lastPoll = lastIMUPoll = lastHumidityPoll = lastPressurePoll = micros();
  timeout= (unsigned long) (50*(imu->IMUGetPollInterval() * 1000));

  if (SERIAL_REPORTING) { Serial.println("System ready."); }

} // setup

/////////////////////////////////////////////////////////////

void loop()
{  
    RTQuaternion rotatedGravity;
    RTQuaternion fusedConjugate;
    RTQuaternion qTemp;
    float residualsAlpha;
    float biasAlpha;

    //  poll at the rate recommended by the IMU
    currentTime = micros();
    int pollDelay = ((imu->IMUGetPollInterval() * 1000) - (int)(currentTime - lastPoll));
    if (pollDelay > 0) { delayMicroseconds(pollDelay); }
    lastPoll = currentTime;

    // check IMU stalled
    if ( (currentTime - lastIMUPoll) > timeout ) {
      // We have IMU stalled and need to reset it
      Serial.println("!!!!!!!!!!!!!!!!!!!! IMU RESET: wait for data for too long !!!!!!!!!!!!!!!!!!!!");
      // Serial.printf("current time %i, lastIMUPoll %i, delta %i, timeout %i\n", currentTime, lastIMUPoll, (currentTime - lastIMUPoll), timeout);
      imu->IMUInit();
      lastIMUPoll = lastPoll = currentTime = micros();
    }

    if (imu->IMURead()) {                       // get the latest data if any avail
        imuData = imu->getIMUData();
        lastIMUPoll = micros();
        
        if ( (imuData.gyro.length() > 35.0) || (imuData.accel.length() > 16.0) || (imuData.compass.length() > 1000.0) ) {
          // IMU Data Error
          Serial.println("!!!!!!!!!!!!!!!!!!!! IMU RESET: Data out of range !!!!!!!!!!!!!!!!!!!!");
          imu->IMUInit();
          lastIMUPoll = micros();
        }

        IMUsampleCount++;
        dt=imuData.timestamp-lastTimestamp;
        lastTimestamp = imuData.timestamp;

        gyro_avg.addValue(imuData.gyro.length());
        accel_avg.addValue(imuData.accel.length());
        compass_avg.addValue(imuData.compass.length());

        bitWrite(fusionStatus, STATE_FUSION_MOVING, imuData.motion); // update moving reporting

        ////////////////////////////////////////////////////////////////////////////////
        // Detect Magnetic Anomaly
        ////////////////////////////////////////////////////////////////////////////////
        if ((fabs(imuData.compass.length()*magFieldNormScale - magFieldNorm)/magFieldNorm) > 0.19f) {
          //Magnetic anomaly detected because magnetic field differs more than 19% from target value
        };
        compassHeading=imuData.fusionQPose.toHeading(imuData.compass, settings->m_compassAdjDeclination);
        compassHeading_avg.addValue(compassHeading);
		    compassHeadingDelta = lastCompassHeading - compassHeading;
    		lastCompassHeading = compassHeading;
    		yawDelta = lastYaw - imuData.fusionPose.z();
    		lastYaw = imuData.fusionPose.z();
    		if (fabs(compassHeadingDelta-yawDelta) > 0.1f) {
    		  //Magnetic anomaly detected because gyro based heading changes are different from magnetic heading changes
    		}
   		  //if (magneticAnomaly {
   			//  Probably too late to adjust here as value was already inserted into fusion algorithm
   			//  imu->setCompassEnable(false);
    		//}
        
        ////////////////////////////////////////////////////////////////////////////////
        // Calibration
        ////////////////////////////////////////////////////////////////////////////////
        if (ACCELCALIBRATE) imu->runtimeAdjustAccelCal();
        // gyroscope calibrtion is built into imu software, therefore no step needed here if actiacted
        // magnetometer calibration is built into imu software, therefore no step needed here if activated

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

        // Motion Handling
        // At end of a motion period, acceleration needs to be zero
        // During no motion, acceleration should be zero
        motionStarted = (imuData.motion && !lastMotion); // is 1 if motion started
        motionEnded   = (!imuData.motion && lastMotion); // is 1 if motion ended
        lastMotion = imuData.motion;

        // Accumulate residual acceleration during no motion to compute an acceleration bias
        // prepare for bias computation
        if (motionStarted) { motionstart = imuData.timestamp; }

        if (motionEnded) {
          intervalCount=0; 
          residualsBiasTemp      = residualsBias; // update bias
          residualsBiasCandidate = residualsBias; // update bias candidate
          firstTime = true;
        }
		
        // Residual Bias
        // exclude first 10 readings and last 10 readings of noMotion-phase when computing residuals
        // same approach as in computing gyro bias
        ///////////////////////////
        if (imuData.motion == false) {     // if no motion, accumulate residual motion for background acceleration computation
          intervalCount++;
          if (intervalCount >= 10){         // work in intervals for 10, discard first and last interval.
            intervalCount = 0;
            residualsBias          = residualsBiasCandidate; 
            residualsBiasCandidate = residualsBiasTemp;
            firstTime = false;
          } // 
          if (firstTime==false) { // update residualsbias
              RTVector3 residualsTemp=(residuals-residualsBiasTemp); // should be close to zero as there is no motion
              if (residualsTemp.length() > 0.1f ) { residualsAlpha = 0.02f; } else { residualsAlpha = 0.002f; } // fast (not close to zero) or regual learning (when close to zero)
              residualsBiasTemp = residualsBiasTemp * (1.0f-residualsAlpha) + residuals * residualsAlpha;       // update average residual acceleration
          } // update bias
        } // no motion

        // subtract acceleration bias from residuals 
        residuals = (residuals - residualsBias); // subtract acceleration bias

        // Convert data into World Coordinate System
        // Compute Velocity
        worldResiduals = RTMath::toWorld(residuals, imuData.fusionQPose); // rotate residuals to world coordinate system
        // integrate acceleration and add to velocity (uses trapezoidal intergration technique
        worldVelocity = worldVelocityPrevious + ((worldResiduals + worldResidualsPrevious)*0.5f*(float(dt)  * 0.000001f)); // dt is in microseconds 
        // Update Velocity Bias
        // When motion ends, velocity should be zero
        if (motionEnded) {
          dtmotion = (float(imuData.timestamp-motionstart))* 0.000001f; // in seconds
          if (dtmotion > 0.5f) { // update velocity bias if we had at least half of second motion
            if (dtmotion > 10.0f) { biasAlpha = 0.2f; } else { biasAlpha = 0.2f * (dtmotion/10.0f); }
            worldVelocityBias = worldVelocityBias * (1.0f-biasAlpha) + (worldVelocity * biasAlpha);       // update average residual acceleration
          }
        }
        // Update Velocity
        worldVelocity = worldVelocity - worldVelocityBias;

        // Velocity is zero if there is no motion
        if (imuData.motion == false) {     // its not moving
          worldVelocity.zero();            // minimize error propagation
        }

        // Compute Position
        worldLocation = worldLocation + ((worldVelocity + worldVelocityPrevious)*0.5f)*((float)dt)*0.000001f;

        // Reset previous values
        worldResidualsPrevious = worldResiduals;
        worldVelocityPrevious  = worldVelocity;

        // Check if gyro bias has adapted
        // The noise on the 9150 gyroscope is about 0.002
        if ((imuData.motion == false) && (imuData.gyro.length() <= 0.003)) {
          GYRO_CALIBRATED = true;
        } else {
          GYRO_CALIBRATED = false;
        }
       
    } // imuread
     
    ///////////////////////////////////////////////////////////////
    // AUX Sensors
    ///////////////////////////////////////////////////////////////

    //  add the pressure data to the structure
    if (pressure != NULL) {
	    currentTime = micros();
      if ( (currentTime - lastPressurePoll) >= pressurePollInterval ) {
        lastPressurePoll = currentTime;
        if (pressure->pressureRead()){
			    pressureData=pressure->getPressureData();
			    if (pressureData.pressureValid) {
			      pressure_avg = pressure->updateAveragePressure(pressureData.pressure); // smooth it out
			    }
			    pressureSampleCount++;		    
		    }
      }
    }
    
    //  add the humidity data to the structure
    if (humidity != NULL) {
      if ( (int)(currentTime - lastHumidityPoll) >= (int)humidityPollInterval ) { // 12.5Hz = 80ms
        lastHumidityPoll = currentTime;
		    if (humidity->humidityRead()) {
          humidityData = humidity->getHumidityData();
          if (humidityData.humidityValid) {
             humidity_avg = humidity->updateAverageHumidity(humidityData.humidity); // smooth it out
          }
	        humiditySampleCount++;
       }
      }
    }

	if ((currentTime - lastRate) >= 1000000) { // 1 second
		lastRate = currentTime;
		IMUsampleRate = IMUsampleCount;
		IMUsampleCount = 0;
		pressureSampleRate = pressureSampleCount;
		pressureSampleCount = 0;
		humiditySampleRate = humiditySampleCount;
		humiditySampleCount = 0;
	}
    
	// Do we want to display data?
	if ((currentTime-lastDisplay) >= DISPLAY_INTERVAL) {
	  lastDisplay = currentTime;
	  if (STREAM) { REPORT = true; } else { REPORT = false; }
	}

	// Do we want to check input ?
	if ((currentTime-lastInAvail) >= CHECKINPUT_INTERVAL) {
	  lastInAvail = currentTime;
	  INAVAIL = true;
	} else {
	  INAVAIL = false;
	}

	///////////////////////////////////////////////////////////////
	// Data Reporting
	///////////////////////////////////////////////////////////////
	
	if (REPORT) {
	  if (VERBOSE) {
		Serial.println("-Data----");
		Serial.print(RTMath::displayRadians("Gyro [r/s]", imuData.gyro));      // gyro data
		Serial.print(RTMath::displayRadians("Accel  [g]", imuData.accel));     // accel data
		imuData.compass=imuData.compass*magFieldNormScale;                     // compass in uT
		Serial.print(RTMath::displayRadians("Mag   [uT]", imuData.compass));   // compass data
		Serial.print(RTMath::displayDegrees("Pose", imuData.fusionPose));      // fused output
		Serial.print(RTMath::display("Quat", imuData.fusionQPose));            // fused quaternion output
		//Serial.printf("Heading from Quat: %+4.3f\n", imuData.fusionQPose.toHeading(imuData.compass, settings->m_compassAdjDeclination)*RTMATH_RAD_TO_DEGREE);
		Serial.printf("Compass based Heading: %+4.3f\n", compassHeading_avg.getAverage()*RTMATH_RAD_TO_DEGREE);
		Serial.printf("Average Gyro: %+4.3f, %+4.3f, ", gyro_avg.getAverage(),imuData.gyro.length()-previousGyroLength);
		previousGyroLength=imuData.gyro.length();
		Serial.printf("Accel: %+4.3f, %+4.3f, ", accel_avg.getAverage(),imuData.accel.length()-previousAccelLength);
		previousAccelLength=imuData.accel.length();
		Serial.printf("Compass: %+4.3f, %+4.3f\n", compass_avg.getAverage(), imuData.compass.length()-previousCompassLength);
		previousCompassLength=imuData.compass.length();
		if (imuData.motion) { Serial.println("Sensor is moving."); } else { Serial.println("Sensor is still."); } // motion
		Serial.println("--Calib--");
		Serial.print(RTMath::displayRadians("Acc Max", settings->m_accelCalMax ));       // 
		Serial.print(RTMath::displayRadians("Acc Min", settings->m_accelCalMin ));       // 
		Serial.print(RTMath::displayRadians("Mag Max", settings->m_compassCalMax ));     // 
		Serial.print(RTMath::displayRadians("Mag Min", settings->m_compassCalMin ));     // 
		tempVec = imu->getCompassRunTimeMagCalMax();
		Serial.print(RTMath::displayRadians("Runtime Mag Max", tempVec ));     // 
		tempVec = imu->getCompassRunTimeMagCalMin();
		Serial.print(RTMath::displayRadians("Runtime Mag Min", tempVec ));     // 
		Serial.print(RTMath::displayRadians("Gyro Bias", settings->m_gyroBias ));        // 
		Serial.printf("Declination: %+4.3f \n", (settings->m_compassAdjDeclination/3.141f*180.0f));
		Serial.print("Runtime Calibrating: ");
		Serial.printf("%s", ACCELCALIBRATE ? "Accelerometer, " : "");  
		Serial.printf("%s", imu->getCompassRunTimeCalibrationEnable() ? "Compass, " : "");
		Serial.printf("%s", imu->getGyroManualCalibrationEnable() ? "Gyroscope Manual, " : "");
		Serial.printf("%s", imu->getGyroRunTimeCalibrationEnable() ? "Gyroscope" : "");  
		Serial.println(".");
		Serial.println("--Aux----");
		Serial.printf("Tempearture IMU %+4.2f, ", imuData.temperature);
		if (pressure != NULL) {	Serial.printf("Pressure %+4.2f, ",   pressureData.temperature);}
	  if (humidity != NULL) { Serial.printf("Humidity %+4.2f\n",   humidityData.temperature);}
		if (pressure != NULL) { Serial.printf("Pressure: %+4.2f, ",  pressureData.pressure);}
    if (humidity != NULL) { Serial.printf("Humidity: %+4.2f\n ", humidityData.humidity); }
		Serial.println("-Status--");
		IMUStatusUpdate();
		Serial.printf("IMUP:  %i ", bitRead(systemStatusIMU, STATE_FUSIONPOSEVALID));
		Serial.printf("IMUQP: %i ", bitRead(systemStatusIMU, STATE_FUSIONQPOSEVALID));
		Serial.printf("IMUG:  %i ", bitRead(systemStatusIMU, STATE_GYROVALID));
		Serial.printf("IMUA:  %i ", bitRead(systemStatusIMU, STATE_ACCELVALID));
		Serial.printf("IMUC:  %i ", bitRead(systemStatusIMU, STATE_COMPASSVALID));
		Serial.printf("IMUT:  %i ", bitRead(systemStatusIMU, STATE_IMUTEMPVALID));
		Serial.println();
		Serial.printf("AUXP:  %i ", bitRead(systemStatusAUX, STATE_PRESSUREVALID));
		Serial.printf("AUXPT: %i ", bitRead(systemStatusAUX, STATE_PRESSURETEMPVALID));
		Serial.printf("AUXH:  %i ", bitRead(systemStatusAUX, STATE_HUMIDITYVALID));
		Serial.printf("AUXHT: %i ", bitRead(systemStatusAUX, STATE_HUMIDITYTEMPVALID));
		Serial.println();
		Serial.println("-Fusion--Run Time-");
		Serial.print("Fusion using: ");
		Serial.printf("%s", bitRead(fusionStatus, STATE_FUSION_GYROENABLE) ? "Gyroscope " : "");  
		Serial.printf("%s", bitRead(fusionStatus, STATE_FUSION_ACCELENABLE) ? "Accelerometer " : "");
		Serial.printf("%s", bitRead(fusionStatus, STATE_FUSION_COMPASSENABLE) ? "Compass" : "");  
		Serial.println();
		Serial.println("-Motion--");
		Serial.print(RTMath::displayRadians("Residuals          [m/s2]", residuals));           // Residuals in device coordiante system
		Serial.print(RTMath::displayRadians("Residuals Bias     [m/s2]", residualsBias));       // Residuals bias in device coordiante system
		Serial.print(RTMath::displayRadians("World Residuals    [m/s2]", worldResiduals));      // Residuals in device world coordiante system
		Serial.print(RTMath::displayRadians("World Velocity      [m/s]", worldVelocity));       // Velocity in world coordiante system
		Serial.print(RTMath::displayRadians("World Velocity Bias [m/s]", worldVelocityBias));   // Velocity bias in world coordiante system
		Serial.print(RTMath::displayRadians("World Position        [m]", worldLocation));       // Location in world coordiante system
		residuals_avg.addValue(residuals.length());
		Serial.printf("Average Residuals Length: %+4.3f \n", residuals_avg.getAverage());
		Serial.println("-System--");
		Serial.print("Sample rate IMU: "); Serial.print(IMUsampleRate);
    if (humidity != NULL) {Serial.print(" humidity: "); Serial.print(humiditySampleRate);}
    if (pressure != NULL) {Serial.print(" pressure: "); Serial.print(pressureSampleRate);}
		if (imu->IMUGyroBiasValid())
			Serial.print(", Gyro bias valid");
		else
			Serial.print(", Calculating gyro bias");
		if (!imu->getCompassCalibrationValid()) {
			if (imu->getRuntimeCompassCalibrationValid())
				Serial.print(", Runtime mag cal valid");
			else     
				Serial.print(", Runtime mag cal not valid");
		} else {
				Serial.print(", EEPROM mag cal valid");
		}
		if (imu->getAccelCalibrationValid())
			Serial.println(", EEPROM Accel cal valid");
		else
			Serial.println(", No accel cal");
		Serial.print("Data Transmission Time [us]: ");
		Serial.println((micros()-currentTime)); // takes about 4ms to send the data
		Serial.println("-End-----");
	  } else {
		if (CUBE) {
		  CubeUpdate();
		} else {
		  BitBucketsUpdate();
		}  
	  } // ASCII
	} // report        

	
    ///////////////////////////////////////////////////////////////
    // Input Commands
    ///////////////////////////////////////////////////////////////
	
    if (INAVAIL) {
      if (Serial.available()) {
        inByte=Serial.read();
        // ENABLE/DISABLE FUSION ALGORITHM INPUTS
        if (inByte == 'm') {        // turn off Compass Fusion
          imu->setCompassEnable(false);;
          bitWrite(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
        } else if (inByte == 'M') { // turn on  Compass Fusion
          imu->setCompassEnable(true);
          bitWrite(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
        } else if (inByte == 'a') { // turn off Accelerometer Fusion
          imu->setAccelEnable(false);
          bitWrite(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
        } else if (inByte == 'A') { // turn on Accelerometer Fusion
          imu->setAccelEnable(true);
          bitWrite(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
        } else if (inByte == 'g') { // turn off Gyroscope Fusion
          imu->setGyroEnable(false);
          bitWrite(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
        } else if (inByte == 'G') { // turn on Gyroscope Fusion
          imu->setGyroEnable(true);
          bitWrite(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
        } else if (inByte == 's') { // turn off streaming
          STREAM=false;
        } else if (inByte == 'S') { // turn on  streaming
          STREAM=true;
        } else if (inByte == 'P') { // reset position
          worldLocation.zero();
          worldVelocityBias.zero();
        } else if (inByte == 'V') { // send verbose
          VERBOSE=true;
        } else if (inByte == 'v') { // send HEX
          VERBOSE=false;
        } else if (inByte == 'b') { // send Cube Processign Pogram HEX
          CUBE=true;
        } else if (inByte == 'B') { // send Bit Buckets HEX
          CUBE=false;
        } else if (inByte == 'w') { // save the calibration data
          settings->saveSettings();
        } else if (inByte == 'r') { // turn off Gyroscope runtime calibration
          imu->setGyroRunTimeCalibrationEnable(false);
          bitWrite(systemUpdateIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
        } else if (inByte == 'R') { // turn on Gyroscope runtime calibration
          imu->setGyroRunTimeCalibrationEnable(true);
          bitWrite(systemUpdateIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
        } else if (inByte == 'd') { // turn off Gyroscope manual calibration
          imu->setGyroManualCalibrationEnable(false);
          bitWrite(systemUpdateIMU, STATE_GYROMANUALUPDATE, imu->getGyroManualCalibrationEnable() );
        } else if (inByte == 'D') { // turn on Gyroscope manual calibration
          imu->setGyroRunTimeCalibrationEnable(false); // can not run runtime and manual same time
          bitWrite(systemUpdateIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
          imu->setGyroManualCalibrationEnable(true);
          bitWrite(systemUpdateIMU, STATE_GYROMANUALUPDATE, imu->getGyroManualCalibrationEnable() );
         } else if (inByte == 'c') { // accelerometer runtime calibration off
            ACCELCALIBRATE=false;
            bitWrite(systemUpdateIMU, STATE_ACCELRUNTIMEUPDATE, ACCELCALIBRATE );
        } else if (inByte == 'C') { // accelerometer runtime calibration on
            ACCELCALIBRATE=true;
            bitWrite(systemUpdateIMU, STATE_ACCELRUNTIMEUPDATE, ACCELCALIBRATE );
            // gyroscope saves settings automatically, turn it off
            imu->setGyroRunTimeCalibrationEnable(false);
            bitWrite(systemStatusIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
        } else if (inByte == 'u') { // compass max/min calibration off
            imu->setCompassRunTimeCalibrationEnable(false);
        } else if (inByte == 'U') { // compass max/min calibration on
            imu->setCompassRunTimeCalibrationEnable(true);
            // gyroscope saves settings automatically, turn it off
            imu->setGyroRunTimeCalibrationEnable(false);
            bitWrite(systemStatusIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
        } else if (inByte == 'i') {
            imu->resetCompassRunTimeMaxMin();
        } else if ((inByte == '?') || (inByte == 'h')) { // send HELP information
            // Menu
            Serial.println("--HELP---");
            Serial.println("You can turn on/off gyro, accelerometer, compass input into fusion algorithm.");  
            Serial.println("m/M to disable/enable compass       in fusion algorithem");
            Serial.println("g/G to disable/enable gyroscope     in fusion algorithem");
            Serial.println("a/A to disable/enable accelerometer in fusion algorithem");
            Serial.println("You can turn on/off data streaming and adjust data format");
            Serial.println("s/S to disable/enable data streaming");
            Serial.println("v/V to disable/enable human readable data display");
            Serial.println("b/B to set minimal/maximal HEX streaming (B) is needed for cube display");
            Serial.println("-----");
            Serial.println("P   to reset current position");
            Serial.println("--Calibration---");
            Serial.println("To calibrate accelerometer, move sensor to a few different postions and activate calibration.");
            Serial.println("Make sure accelerometer calibration is off when you move the sensor.");
            Serial.println("c/C to deactive/activate runtime acceleration calibration.");
            Serial.println("Gyroscope calibration can run in the background.");
            Serial.println("r/R to deactive/activate runtime gyroscope bias update");
            Serial.println("d/D to deactive/activate manual gyroscope bias calculation");
            Serial.println("Compass calibration needs to be completed for all axes before fusion operates correctly.");
            Serial.println("u/U to deactive/activate runtime compass Max/Min update");
            Serial.println("i to reset compass runtime max/min");
            Serial.println("w to save current calibration data. It will be loaded next time you restart the sensor");
            Serial.println("-----");
            Serial.println("Press S to continue. Streaming was turned off to hold this diplay.");
            Serial.println("--HELP END---");
            STREAM=false;
        }
      } // end if serial input available
    } // end INAVAIL

    ///////////////////////////////////////////////////////////////
    // Blink LED if gyro is calibrated otherwise keep it on
    ///////////////////////////////////////////////////////////////
	
    // But keep LED on if system is moving
    if (GYRO_CALIBRATED == true) {
      if ((currentTime - lastBlink) > LEDBLINK_INTERVAL) {
        if (ledStatus == false) {
          digitalWrite(ledPin, HIGH);
          ledStatus = true;       
        } else {
          digitalWrite(ledPin, LOW);
          ledStatus = false;
        }
        lastBlink = currentTime;
      }
    } else {
      if (ledStatus == false) {  
         digitalWrite(ledPin, HIGH);
         ledStatus = true;
      }  
    } // GYRO blinking
    
} // end of main loop

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

// REPORTING OF VARIABLES

void IMUStatusUpdate(){
    //--FUSION
    bitWrite(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
    bitWrite(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
    bitWrite(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
    //--IMU
    bitWrite(systemStatusIMU, STATE_FUSIONPOSEVALID, imuData.fusionPoseValid);
    bitWrite(systemStatusIMU, STATE_FUSIONQPOSEVALID, imuData.fusionQPoseValid);
    bitWrite(systemStatusIMU, STATE_GYROVALID, imuData.gyroValid); 
    bitWrite(systemStatusIMU, STATE_ACCELVALID, imuData.accelValid); 
    bitWrite(systemStatusIMU, STATE_COMPASSVALID, imuData.compassValid); 
    bitWrite(systemStatusIMU, STATE_IMUTEMPVALID, imuData.temperatureValid);
    //--Update
    bitWrite(systemUpdateIMU, STATE_ACCELRUNTIMEUPDATE, ACCELCALIBRATE );
    bitWrite(systemUpdateIMU, STATE_GYROMANUALUPDATE, imu->getGyroManualCalibrationEnable() );
    bitWrite(systemUpdateIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
    bitWrite(systemUpdateIMU, STATE_COMPASSRUNTIMEUPDATE, imu->getCompassRunTimeCalibrationEnable() );
    //--Aux
    bitWrite(systemStatusAUX, STATE_PRESSUREVALID, pressureData.pressureValid);
    bitWrite(systemStatusAUX, STATE_PRESSURETEMPVALID, pressureData.temperatureValid); 
    bitWrite(systemStatusAUX, STATE_HUMIDITYVALID, humidityData.humidityValid); 
    bitWrite(systemStatusAUX, STATE_HUMIDITYTEMPVALID, humidityData.temperatureValid); 
}

void BitBucketsUpdate(){
    serialLongPrint(imuData.timestamp); Serial.print(',');
    serialBytePrint(fusionStatus); Serial.print(',');
    serialFloatPrint(imuData.fusionPose.x()); Serial.print(',');
    serialFloatPrint(imuData.fusionPose.y()); Serial.print(',');
    serialFloatPrint(imuData.fusionPose.z()); Serial.print(',');
    Serial.println();
}

void CubeUpdate(){
    HEXsendRAW();
    HEXsendQ();
    HEXsendE();
    HEXsendR();
    HEXsendH();
    HEXsendWR();
    HEXsendWV();
    HEXsendWP();
    serialLongPrint(imuData.timestamp); Serial.print(',');
    HEXsendFusionStatus();
    Serial.println();
}    

void HEXsendIMUStatus(){
    IMUStatusUpdate();
    serialBytePrint(systemStatusIMU); Serial.print(',');
    serialBytePrint(systemStatusAUX); Serial.print(',');
}

void HEXsendFusionStatus(){
//  send Fusion Status
//  STATE_FUSION_GYROENABLE    0
//  STATE_FUSION_COMPASSENABLE 1
//  STATE_FUSION_ACCELENABLE   2
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
    serialFloatPrint(compassHeading_avg.getAverage()); Serial.print(',');
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
  for(int i=3; i>=0; i--) {
    
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

// CONVERT Byte TO HEX AND SEND OVER SERIAL PORT
void serialLongPrint(unsigned long l) {
  byte * b = (byte *) &l;
  for(int i=3; i>=0; i--) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}

