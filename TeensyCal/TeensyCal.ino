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
#include "RTIMUAccCal.h"
#include "RTIMUGyroCal.h"
#include "RTIMUTemperatureCal.h"

void pollIMUandDisplay();
void doResetAccelCal();
void pollIMUandDisplay();
void doMagMinMaxCal();
void doMagEllipsoidCal();
void doAccelEllipsoidCal();
void doTemperatureCal();
void doAccelMinMaxCal();
void doRuntimeAccelCal();

RTIMU *imu;                                           // the IMU object
RTIMU_DATA imuData;                                   // IMU Data Structure
RTIMUSettings *settings;                              // the settings object
RTIMUAccCal   *accCal;                                  // the accel calibration object
RTIMUMagCal   *magCal;                                  // the compass calibration object
RTIMUGyroCal  *gyrCal;                                  // the gyro calibration object
RTIMUTemperatureCal *temperatureCal;

bool magMinMaxDone;
bool accelMinMaxDone;
bool temperatureDone;
bool accelEnables[3];
bool accelEllipsoidEnable;
int  accelCurrentAxis;
bool doReport;

unsigned long lastTime;
unsigned long lastReport;
unsigned long currentTime;
int inByte = 0;

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200
//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  50000

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

    Serial.print("TeensyCal starting using device "); Serial.println(imu->IMUName());

    imu->setAccelCalibrationMode(false); 
    imu->setCompassCalibrationMode(false);
    imu->setGyroCalibrationMode(false);
    imu->setTemperatureCalibrationMode(false);

    Serial.print("TeensyCal configure IMU "); Serial.println(imu->IMUName());

    accCal = new RTIMUAccCal(settings);
    accCal->accelCalInit();

    Serial.print("TeensyCal Accel Init, ");
    
    magCal = new RTIMUMagCal(settings);
    magCal->magCalInit();

    Serial.print("TeensyCal Mag Init, ");
    
    gyrCal = new RTIMUGyroCal(settings);
    gyrCal->gyroCalInit();

    Serial.print("TeensyCal Gyro Init, ");

    temperatureCal = new RTIMUTemperatureCal(settings);
    temperatureCal->temperatureCalInit();

    Serial.print("TeensyCal Temp Init, ");

    magMinMaxDone = false;
    accelMinMaxDone = false;
    temperatureDone = false;

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);
  
    Serial.print("Fusion Set.");

    lastTime = lastReport = micros();
}

void loop()
{  
    currentTime = micros();
    if (currentTime-lastReport >= DISPLAY_INTERVAL) {
      doReport= true; 
      lastReport = currentTime; 
    } else {
      doReport = false;
    }

    if (doReport) {
      Serial.println("Options are:");
      Serial.println("  T - temperature calibration.");
      Serial.println("  m - calibrate magnetometer with min/max.");
      Serial.println("  M - calibrate magnetometer with ellipsoid (do min/max first).");
      Serial.println("  a - calibrate accelerometers with min/max.");
      Serial.println("  A - calibrate accelerometers with ellipsoid (do min/max first).");
      Serial.println("  G - enabel runtime gyro calibration.");
      Serial.println("  g - disable runtime gyro calibration.");
      Serial.println("  r - runtime accelerometer calibration.");
      Serial.println("  R - reset calibration.");
      Serial.println("  s - save settings to EEPROM.");
    }

    pollIMUandDisplay();

    if (Serial.available()) {
        inByte=Serial.read();

      switch (inByte) {
      case 'm' :
        doMagMinMaxCal();
        break;

      case 'M' :
        doMagEllipsoidCal();
        break;

      case 'a' :
        doAccelMinMaxCal();
        break;

      case 'A' :
        doAccelEllipsoidCal();
        break;

      case 'g' :
        imu->setGyroRunTimeCalibrationEnable(false);
        break;

      case 'G' :
        imu->setGyroRunTimeCalibrationEnable(true);
        break;

      case 'T' :
        doTemperatureCal();
        break;

      case 'r' :
        doRuntimeAccelCal();
        break;

      case 'R' :
        doResetAccelCal();
        break;
        
      case 's' :
        settings->saveSettings();
        break;
        
      } // switch
    } //serial
}

void pollIMUandDisplay()
{
    if (imu->IMURead()) {                                 // get the latest data
        imuData = imu->getIMUData();
    }
    
    if (doReport) {
      Serial.println("-Data----");
      Serial.print(RTMath::displayRadians("Gyro [r/s]", imuData.gyro));      // gyro data
      Serial.print(RTMath::displayRadians("Accel  [g]", imuData.accel));     // accel data
      Serial.print(RTMath::displayRadians("Mag   [uT]", imuData.compass));   // compass data
      Serial.println("--Calib--");
      if (imuData.motion) { Serial.println("Sensor is moving."); } else { Serial.println("Sensor is still."); } // motion
      Serial.print(RTMath::displayRadians("Acc Max", settings->m_accelCalMax ));       // 
      Serial.print(RTMath::displayRadians("Acc Min", settings->m_accelCalMin ));       // 
      Serial.print(RTMath::displayRadians("Mag Max", settings->m_compassCalMax ));     // 
      Serial.print(RTMath::displayRadians("Mag Min", settings->m_compassCalMin ));     // 
      Serial.print(RTMath::displayRadians("Gyro Bias", settings->m_gyroBias ));      // 
      Serial.printf("Declination: %+4.3f \n", (settings->m_compassAdjDeclination/3.141*180.0));
      if (imu->IMUGyroBiasValid()) Serial.print("Gyro bias valid"); else Serial.print("Calculating gyro bias");
      if (!imu->getCompassCalibrationValid()) {
        if (imu->getRuntimeCompassCalibrationValid()) Serial.println(", runtime mag cal valid"); else Serial.println(", runtime mag cal not valid");
      } else {
        Serial.println(", EEPROM mag cal valid");
      }
      if (imu->getAccelCalibrationValid())   Serial.print("Accel Calib Valid, ");    else Serial.print("Accel Calib Not Valid, ");
      if (imu->getGyroCalibrationValid())    Serial.print("Gyro Calib Valid, ");     else Serial.print("Gyro Calib Not Valid, ");
      if (imu->getCompassCalibrationValid()) Serial.println("Compass Calib Valid."); else Serial.println("Compass Calib Not Valid.");
    }
    
    int time_elapsed = (int)(micros() - lastTime);
    if ( time_elapsed < imu->IMUGetPollInterval()*1000) {
      delayMicroseconds((unsigned int)(imu->IMUGetPollInterval() * 1000 - time_elapsed));
    }
    lastTime = micros();

} // pool and display

void doResetAccelCal()
{
  while(1) {

    currentTime = micros();
    if (currentTime-lastReport >= DISPLAY_INTERVAL) {
      doReport= true; 
      lastReport = currentTime;
    } else {
      doReport = false;
    }

    if (doReport) {
      Serial.println("Resetting Calibration");
      Serial.println("-----------------------------------\n");
      Serial.println("Available options are:\n");
      Serial.println("  R - Reset Accel calibration XYZ.");
      Serial.println("  X - Reset Accel calibration X.");
      Serial.println("  Y - Reset Accel calibration X.");
      Serial.println("  Z - Reset Accel calibration X.");
      Serial.println("  x - to return to previous menu");
    }

    pollIMUandDisplay();
    
    if (Serial.available()) {
      inByte=Serial.read();
      
      switch (inByte) {

      case 'R':                   // X calibration reset
        accCal->accCalReset();
        break;

        case 'X':                   // X calibration reset
        accCal->accCalXReset();
        break;
      
      case 'Y':                  // Y calibration rest
        accCal->accCalYReset();
        break;
      
      case 'Z':                  // Z calibration rest
        accCal->accCalZReset();
          break;
        
      case 'x':
        return;
      } // switch
    } // serial
  } // while
} // reset accel cal


void doMagMinMaxCal()
{
    magCal->magCalInit();
    magMinMaxDone = false;
    imu->setCompassCalibrationMode(true); 

    while (1) {
      currentTime = micros();
      if (currentTime-lastReport >= DISPLAY_INTERVAL) {
        doReport= true; 
        lastReport = currentTime; 
      } else {
        doReport = false;
      }
      
      if (doReport) {
        Serial.println("Magnetometer min/max calibration");
        Serial.println("--------------------------------");
        Serial.println("Waggle the IMU chip around, ensuring that all six axes");
        Serial.println("(+x, -x, +y, -y and +z, -z) go through their extrema.");
        Serial.println("When all extrema have been achieved, enter 's' to save, 'r' to reset");
        Serial.println("or 'x' to abort and discard the data.");
        Serial.print(RTMath::displayRadians("Mag Max[uT]", magCal->m_magMax)); 
        Serial.print(RTMath::displayRadians("Mag Min[uT]", magCal->m_magMin)); 
      }      
      pollIMUandDisplay();
      magCal->newMinMaxData(imuData.compass);

      if (Serial.available()) {
        inByte=Serial.read();
  
        switch (inByte) {
           case 's' :
               Serial.println("Saving min/max data.");
               magCal->magCalSaveMinMax();
               magMinMaxDone = true;
               imu->setCompassCalibrationMode(false);
               return;
  
           case 'x' :
               Serial.println("\nAborting.\n");
               imu->setCompassCalibrationMode(false);
               return;
  
           case 'r' :
               Serial.println("Resetting min/max data.");
               bool temp = settings->m_compassCalValid;
               settings->m_compassCalValid = false;
               magCal->magCalReset();
               settings->m_compassCalValid = temp;
               break;
         } // switch
      } // serial
    } // while
} // mag max min

void doMagEllipsoidCal()
{
  Serial.println("Not implemented");
}

void doAccelEllipsoidCal()
{
  Serial.println("Not implemented");
}
void doTemperatureCal() {
  Serial.println("Not implemented");
}

void doAccelMinMaxCal()
{

  //  perform all axis reset
  for (int i = 0; i < 3; i++) accCal->accelCalEnable(i, true);
  accCal->accelCalReset();
  for (int i = 0; i < 3; i++) accCal->accelCalEnable(i, false);
  accelCurrentAxis = 0;
  for (int i = 0; i < 3; i++) accelEnables[i] = false;

  imu->setAccelCalibrationMode(true); 

  while(1) {
    currentTime = micros();
    
    if (currentTime-lastReport >= DISPLAY_INTERVAL) {
      doReport= true; 
      lastReport = currentTime;
    } else {
      doReport = false;
    }
    if (doReport) {
      Serial.println("Accelerometer Calibration");
      Serial.println("-------------------------");
      Serial.println("This code ignores calibration readings until an axis has been selected.");
      Serial.println("Orient the IMU near an extrema (+x, -x, +y, -y, +z, -z).");
      Serial.println("Then enable that axis, moving the IMU very gently around to find the");
      Serial.println("maximum reading. Then disable the axis so that the IMU can be inverted.");
      Serial.println("Enable the axis again and find the opposite maximum reading.");
      Serial.println("point. Disable the axis again and press the space bar to move to the next");
      Serial.println("axis and repeat. The software will display the current axis and enable state.");
      Serial.println("Available options are:");
      Serial.println("  e - enable the current axis.");
      Serial.println("  d - disable the current axis.");
      Serial.println("  space bar - move to the next axis (x then y then z then x etc.");
      Serial.println("  r - reset the current axis (if enabled).");
      Serial.println("  s - save the data once all 6 extrema have been collected.");
      Serial.println("  x - return to precious menu.");
      Serial.print(RTMath::displayRadians("Acc Max[g]", accCal->m_accelMax)); 
      Serial.print(RTMath::displayRadians("Acc Min[g]", accCal->m_accelMin)); 

    }
    
    pollIMUandDisplay();
    
    for (int i = 0; i < 3; i++)
      accCal->accelCalEnable(i, accelEnables[i]);
    
    accCal->newMinMaxData(imuData.accel);
    
    if (Serial.available()) {
      inByte=Serial.read();
      
      switch (inByte) {

      case 'e':
        accelEnables[accelCurrentAxis] = true;
        break;

      case 'd':
        accelEnables[accelCurrentAxis] = false;
        break;

      case 'r':
        accCal->accelCalReset();
        break;

      case ' ':
        accelCurrentAxis = accelCurrentAxis+1;
        if (accelCurrentAxis == 3) accelCurrentAxis = 0;
        break;

      case 's':
        accCal->accelCalSaveMinMax();
        accelMinMaxDone = true;
        Serial.println("\nAccelerometer calibration data saved.\n");
        imu->setAccelCalibrationMode(false); 
        return;
    
      case 'x':
        imu->setAccelCalibrationMode(false); 
        return;
        
      } // switch
    } // serial
  } // while
} // accel max min

void doRuntimeAccelCal()
{
  bool engageRuntimeCalib = false;

  while(1) {
    currentTime = micros();
    if (currentTime-lastReport >= DISPLAY_INTERVAL) {
      doReport= true; 
      lastReport = currentTime;
    } else {
      doReport = false;
    }
    if (doReport) {
      Serial.println("Accelerometer Runtime Calibration");
      Serial.println("-----------------------------------");
      Serial.println("Available options are:");
      Serial.println("  R - enable runtime calibration.");
      Serial.println("  r - disbale runtime calibration.");
      Serial.println("  s - save the data once all 6 extrema have been collected.");
      Serial.println("  x - return to previous menu.");
    }
   
    pollIMUandDisplay();

    if (engageRuntimeCalib) {
      imu->runtimeAdjustAccelCal();
    }

    if (Serial.available()) {
      inByte=Serial.read();
      
      switch (inByte) {

      case 'R':
        engageRuntimeCalib = true;
        break;

      case 'r':
        engageRuntimeCalib = false;
        break;

      case 's':
        settings->saveSettings();
        accelMinMaxDone = true;
        accCal->m_accelMin = settings->m_accelCalMin;
        accCal->m_accelMax = settings->m_accelCalMax;
        Serial.println("\nAccelerometer calibration data saved.\n");
        return;
    
      case 'x':
        return;
      } // swtich
    } // serial
  } // while
} // run time accel calib
