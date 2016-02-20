////////////////////////////////////////////////////////////////////////////
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
RTPressure *pressure;                                 // the pressure object
RTHumidity *humidity;                                 // the humidity object
RTIMUSettings *settings;                              // the settings object
RTIMU_DATA imuData;                                   // IMU Data Structure

//  DISPLAY_INTERVAL sets the rate at which results are transferred to host
#define DISPLAY_INTERVAL      100                   // interval in microseconds between data updates over serial port: 100000=10Hz, Fusion update is 80Hz, lowpass is 40Hz
#define CHECKINPUT_INTERVAL   50000                   // 35000 will result in 40ms update time

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
bool STREAM=true;                                     // are we continously streaming output?
bool INAVAIL=false;                                   // do we need to check for input ?
bool VERBOSE=false;                                   // set HEX mode
bool CUBE=false;                                      // set output format for 4183 Robot
int  inByte = 0;                                      // serial input buffer, one byte

// Fusion Status
byte fusionStatus = B00000000;                       // Represents the enabled components of the fusion algorithm
float slerpPower = 0.02;                             // Fusion convergence
// devices used for fusion 
#define STATE_FUSION_GYROENABLE    0                  // Use Gyroscope for Fusion
#define STATE_FUSION_COMPASSENABLE 1                  // Use Compass for Fusion
#define STATE_FUSION_ACCELENABLE   2                  // Use Accelerator for Fusion
#define STATE_FUSION_MOVING        7                  // System is moving

// IMU Status
byte systemStatusIMU = B00000000;
#define STATE_FUSIONPOSEVALID       0
#define STATE_FUSIONQPOSEVALID      1
#define STATE_GYROVALID             2
#define STATE_ACCELVALID            3
#define STATE_COMPASSVALID          4
#define STATE_IMUTEMPVALID          5
#define STATE_GYRORUNTIMEUPDATE     6
#define STATE_COMPASSRUNTIMEUPDATE  7

// AUX Status
byte systemStatusAUX = B00000000;
#define STATE_PRESSUREVALID      0
#define STATE_PRESSURETEMPVALID  1
#define STATE_HUMIDITYVALID      2
#define STATE_HUMIDITYTEMPVALID  3

// Position Sensing
const float accScale = 9.81f;                         // conversion to 9.81 m/s/s
const float magFieldNorm=47.12f;                      // conversion to microT
float magFieldNormScale =1.0f;                        // to adjust current magnetometer readings
float compassHeading;                                 // tilt compensated heading
float lastCompassHeading;
float compassHeadingDelta;
float lastYaw;
float yawDelta;
RunningAverage compassHeading_avg(5);                 // Running average for heading (noise reduction)
RTVector3 comp;                                       // to compute earh field strength
RTVector3 residuals;                                  // gravity subtracted acceleration
RTVector3 residualsBias;                              // average acceleration when device is stationary
RTVector3 residualsBiasTemp;                          // for computing average acceleration
RTVector3 residualsBiasCandidate;                     // for computing average acceleration
int intervalCount = 0;                                // interval loop o updating residual bias
bool firstTime = true;                                // first interval in residual calculations
RunningAverage residuals_avg(25);                     // Running average for residuals (debug)
RunningAverage gyro_avg(25);                          // Running average for gyro (debug)
RunningAverage compass_avg(25);                       // Running average for compass (debug)
RunningAverage accel_avg(25);                         // Running average for acceleration (debug)
RTVector3 worldResiduals;                             // residuals in the world coordinate system
RTVector3 worldResidualsPrevious;                     // for trapezoidal integration of acceleration
RTVector3 worldVelocity;                              // integrated acceleration
RTVector3 worldVelocityBias;                          // velocity bias computed at end of motion
RTVector3 worldVelocityPrevious;                      // for trapezoidal integration of velocity
RTVector3 worldLocation;                              // integrated velocity
RTQuaternion gravity;

bool lastMotion=false, motionStarted=false, noMotionStarted=false;

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

    if (imu->getAccelCalibrationValid())
        Serial.println("Using accel calibration");
    else
        Serial.println("No valid accel calibration data");

    if (imu->getGyroCalibrationValid())
        Serial.println("Using gyro calibration");
    else
        Serial.println("No valid gyro calibration data");

    ////////////////////////////////////////////////////////////////////////////////////////
    //// set up any fusion parameters here
    ////////////////////////////////////////////////////////////////////////////////////////
    imu->setSlerpPower(slerpPower);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);
    settings->setDeclination(9.97f*3.141f/180.0f); // Magnetic Declination in Tucson AZ
    
    //--Fusion
    bitWrite(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
    bitWrite(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
    bitWrite(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
    bitWrite(fusionStatus, STATE_FUSION_MOVING,        false );

    // House keeping & Initializing
    lastDisplay = lastRate = lastTimestamp = lastInAvail = micros();
    sampleCount = 0;
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
    int i=0; // dry run
    while (i < 80) {
      if  (imu->IMURead()) {
        i++;
      }
    }

    // Compute normalization factor for earts magnetic field
    comp.zero();
    i=0;
    while (i < 50) {
      if  (imu->IMURead()) {                       // get the latest calibrated data 
        imuData = imu->getIMUData();
        comp = comp + imuData.compass;             // get compass
        i++;
      }
    }
    comp = comp / 50.0f; // compute average compass (devide by 50)
    magFieldNormScale=magFieldNorm/comp.length();

    Serial.println("Send S to start streaming.");
}
  
void loop()
{  
    unsigned long currentTime = micros();
    
    RTQuaternion rotatedGravity;
    RTQuaternion fusedConjugate;
    RTQuaternion qTemp;
    float residualsAlpha;
    
    if (imu->IMURead()) {                       // get the latest data if any avail
        imuData = imu->getIMUData();
        sampleCount++;
        dt=imuData.timestamp-lastTimestamp;
        lastTimestamp = imuData.timestamp;

        bitWrite(fusionStatus, STATE_FUSION_MOVING, imuData.motion); // update moving reporting

        ////////////////////////////////////////////////////////////////////////////////
        // Detect Magnetic Anomaly
        ////////////////////////////////////////////////////////////////////////////////
        if ((fabs(imuData.compass.length()*magFieldNormScale - magFieldNorm)/magFieldNorm) > 0.19f) {
          //Magnetic anomaly detected because magnetic field differs more than 19% from target value
        };
        compassHeading=imuData.fusionQPose.toHeading(imuData.compass, settings->m_compassAdjDeclination);
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
        noMotionStarted   = (!imuData.motion && lastMotion); // is 1 if motion ended
        lastMotion = imuData.motion;

        // Accumulate residual acceleration during no motion to compute an acceleration bias
        // prepare for bias computation
        if (motionStarted) { motionstart = imuData.timestamp; }
        if (noMotionStarted) {
          intervalCount=0;
          residualsBiasTemp      = residualsBias;
          residualsBiasCandidate = residualsBias;
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
          if (firstTime==false) {
              RTVector3 residualsTemp=(residuals-residualsBiasTemp);
              if (residualsTemp.length() > 0.1f ) { residualsAlpha = 0.02f; } else { residualsAlpha = 0.002f; } // fast or regual learning
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
        if (noMotionStarted) {
          dtmotion = (float(imuData.timestamp-motionstart))/1.0E6; // in seconds
          if (dtmotion > 0.5f) { // update velocity bias if we had at least half of second motion
            worldVelocityBias=worldVelocity/dtmotion;
            // could use some learning averaging here with previous values and weigthing long motions more than short ones
          }
        }
        // Update Velocity
        worldVelocity = worldVelocity - ( worldVelocityBias * (float(dt) * 0.000001f) );
        // Avoid Velocity Bias
        if (imuData.motion == false) {     // its not moving
          worldVelocity.zero();            // minimize error propagation
        }

        // Compute Position
        worldLocation = worldLocation + ((worldVelocity + worldVelocityPrevious)*0.5f)*((float)dt)*0.000001f;

        // Reset previous values
        worldResidualsPrevious = worldResiduals;
        worldVelocityPrevious  = worldVelocity;

        // Do we want to update sampling rate?
        if ((currentTime - lastRate) >= 1000000) { // 1 second
            sampleRate=sampleCount;
            sampleCount = 0;
            lastRate = currentTime;
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
            // currentTime=micros();
            Serial.println("-System--");
            Serial.print("Sample rate: "); Serial.print(sampleRate);
            if (imu->IMUGyroBiasValid())
                Serial.print(", gyro bias valid");
            else
                Serial.print(", calculating gyro bias");
            if (!imu->getCompassCalibrationValid()) {
                if (imu->getRuntimeCompassCalibrationValid())
                    Serial.print(", runtime mag cal valid");
                else     
                    Serial.print(", runtime mag cal not valid");
            } else {
                    Serial.print(", EEPROM mag cal valid");
            }
            if (imu->getAccelCalibrationValid())
                Serial.println(", accel cal valid");
            else
                Serial.println(", no accel cal");
            Serial.println("-Data----");
            Serial.print(RTMath::displayRadians("Gyro [r/s]", imuData.gyro));      // gyro data
            Serial.print(RTMath::displayRadians("Accel  [g]", imuData.accel));     // accel data
            imuData.compass=imuData.compass*magFieldNormScale;                 // compass in uT
            Serial.print(RTMath::displayRadians("Mag [uT]", imuData.compass));     // compass data
            Serial.print(RTMath::displayDegrees("Pose", imuData.fusionPose)); // fused output
            gyro_avg.addValue(imuData.gyro.length());
            accel_avg.addValue(imuData.accel.length());
            compass_avg.addValue(imuData.compass.length());
            Serial.printf("Average Gyro: %+4.3f, ", gyro_avg.getAverage());
            Serial.printf("Accel: %+4.3f, ", accel_avg.getAverage());
            Serial.printf("Compass: %+4.3f \n", compass_avg.getAverage());

            Serial.println("--Calib--");
            if (imuData.motion) { Serial.println("Sensor is moving."); } else { Serial.println("Sensor is still."); } // motion
            Serial.print(RTMath::displayRadians("Acc Max", settings->m_accelCalMax ));       // 
            Serial.print(RTMath::displayRadians("Acc Min", settings->m_accelCalMin ));       // 
            Serial.print(RTMath::displayRadians("Mag Max", settings->m_compassCalMax ));       // 
            Serial.print(RTMath::displayRadians("Mag Min", settings->m_compassCalMin ));       // 
            Serial.print(RTMath::displayRadians("Gyro Bias", settings->m_gyroBias ));       // 
            Serial.printf("Declination: %+4.3f \n", (settings->m_compassAdjDeclination/3.141f*180.0f));
            Serial.println("--Aux----");
            Serial.printf("IMU_T %+4.2f, ", imuData.IMUtemperature);
            Serial.printf("P %+4.2f, ",     imuData.pressure);
            Serial.printf("P_T %+4.2f, ",   imuData.pressureTemperature);
            Serial.printf("H %+4.2f, ",     imuData.humidity);
            Serial.printf("H_T %+4.2f ",    imuData.humidityTemperature);
            Serial.println();
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
            Serial.printf("G:     %i ", bitRead(fusionStatus, STATE_FUSION_GYROENABLE));
            Serial.printf("A:     %i ", bitRead(fusionStatus, STATE_FUSION_ACCELENABLE));
            Serial.printf("C:     %i ", bitRead(fusionStatus, STATE_FUSION_COMPASSENABLE));
            Serial.printf("Grt:   %i ", bitRead(systemStatusIMU, STATE_GYRORUNTIMEUPDATE));
            Serial.printf("Crt:   %i ", bitRead(systemStatusIMU, STATE_COMPASSRUNTIMEUPDATE));
            Serial.println();
            Serial.println("-Motion--");
            Serial.print(RTMath::displayRadians("Residuals          [m/s2]", residuals));           // Residuals in device coordiante system
            Serial.print(RTMath::displayRadians("Residuals Bias     [m/s2]", residualsBias));       // Residuals bias in device coordiante system
            Serial.print(RTMath::displayRadians("World Residuals    [m/s2]", worldResiduals));      // Residuals in device world coordiante system
            Serial.print(RTMath::displayRadians("World Velocity      [m/s]", worldVelocity));       // Velocity in world coordiante system
            Serial.print(RTMath::displayRadians("World Velocity Bias [m/s]", worldVelocityBias));   // Velocity bias in world coordiante system
            Serial.print(RTMath::displayRadians("World Position        [m]", worldLocation));       // Location in world coordiante system
            residuals_avg.addValue(residuals.length());
            Serial.printf("Average Residuals Length: %+4.3f, ", residuals_avg.getAverage());
            Serial.print("Data Transmission Time [us]: ");
            Serial.println((micros()-currentTime)); // takes about 4ms to send the data
          } else {
            if (CUBE) {
              CubeUpdate();
            } else {
              BitBucketsUpdate();
            }  
          } // ASCII
        } // report
    } // imuread
  
    ///////////////////////////////////////////////////////////////
    // Input Commands
    ///////////////////////////////////////////////////////////////
    if (INAVAIL) {
      if (Serial.available()) {
        inByte=Serial.read();
        // ENABLE/DISABLE FUSION ALGORITHM INPUTS
        if        (inByte == 'm') {      // turn off Compass Fusion
          imu->setCompassEnable(false);;
          bitWrite(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
        } else if (inByte == 'M') { // turn on  Compass Fusion
          imu->setCompassEnable(true);
          bitWrite(fusionStatus, STATE_FUSION_COMPASSENABLE, imu->getCompassEnable() );
        } else if (inByte == 'a') {      // turn off Accelerometer Fusion
          imu->setAccelEnable(false);
          bitWrite(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
        } else if (inByte == 'A') { // turn on Accelerometer Fusion
          imu->setAccelEnable(true);
          bitWrite(fusionStatus, STATE_FUSION_ACCELENABLE,   imu->getAccelEnable() );
        } else if (inByte == 'g') {      // turn off Gyroscope Fusion
          imu->setGyroEnable(false);
          bitWrite(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
        } else if (inByte == 'G') { // turn on Gyroscope Fusion
          imu->setGyroEnable(true);
          bitWrite(fusionStatus, STATE_FUSION_GYROENABLE,    imu->getGyroEnable() );
        } else if (inByte == 'r') { // turn on Gyroscope Fusion
          imu->setGyroRunTimeCalibrationEnable(false);
          bitWrite(systemStatusIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
        } else if (inByte == 'R') { // turn on Gyroscope Fusion
          imu->setGyroRunTimeCalibrationEnable(true);
          bitWrite(systemStatusIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
        } else if (inByte == 's') { // turn off streaming
          STREAM=false;
        } else if (inByte == 'S') { // turn on  streaming
          STREAM=true;
        } else if (inByte == 'P') { // reset position
          worldLocation.zero();
        } else if (inByte == 'V') { // send verbose
          VERBOSE=true;
        } else if (inByte == 'v') { // send HEX
          VERBOSE=false;
        } else if (inByte == 'b') { // send Cube Processign Pogram HEX
          CUBE=true;
        } else if (inByte == 'B') { // send Bit Buckets HEX
          CUBE=false;
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
    bitWrite(systemStatusIMU, STATE_IMUTEMPVALID, imuData.IMUtemperatureValid);
    bitWrite(systemStatusIMU, STATE_GYRORUNTIMEUPDATE, imu->getGyroRunTimeCalibrationEnable() );
    bitWrite(systemStatusIMU, STATE_COMPASSRUNTIMEUPDATE, imu->getCompassRunTimeCalibrationEnable() );
    //--Aux
    bitWrite(systemStatusAUX, STATE_PRESSUREVALID, imuData.pressureValid);
    bitWrite(systemStatusAUX, STATE_PRESSURETEMPVALID, imuData.pressureTemperatureValid); 
    bitWrite(systemStatusAUX, STATE_HUMIDITYVALID, imuData.humidityValid); 
    bitWrite(systemStatusAUX, STATE_HUMIDITYTEMPVALID, imuData.humidityTemperatureValid); 
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
    serialFloatPrint(compassHeading); Serial.print(',');
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


