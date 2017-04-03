////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Teensy
//
//  Copyright (c) 2014-2015, richards-tech
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

//  The MPU-9250 driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)

// UU: This file was changed to
// include temperature from pressure, humidity and IMU in data structure

#ifndef _RTIMULIBDEFS_H
#define	_RTIMULIBDEFS_H

#include "RTMath.h"
#include "utility/RTIMUDefs.h"

//  these defines describe the various fusion filter options

#define RTFUSION_TYPE_NULL                  0                   // just a dummy to keep things happy if not needed
#define RTFUSION_TYPE_KALMANSTATE4          1                   // kalman state is the quaternion pose
#define RTFUSION_TYPE_RTQF                  2                   // RT quaternion fusion
#define RTFUSION_TYPE_AHRS                  3                   // AHRS quaternion fusion

#define RTFUSION_TYPE_COUNT                 4                   // number of fusion algorithm types

#define MAGFIELDNORM 47.118f									// Earths Magnetic Field Strength in Tucson
#define DECLINATION 9.98f * 3.1415926535f / 180.0f				// Declination in Tucson


//  This is a convenience structure that can be used to pass IMU data around

typedef struct
{
    uint64_t timestamp;
    bool fusionPoseValid;
    RTVector3 fusionPose;
    bool fusionQPoseValid;
    RTQuaternion fusionQPose;
    bool gyroValid;
    RTVector3 gyro;
    bool accelValid;
    RTVector3 accel;
    bool compassValid;
    RTVector3 compass;
    bool motion;
    bool temperatureValid;
    RTFLOAT temperature;
} RTIMU_DATA;

typedef struct
{
    uint64_t timestamp;
    bool humidityValid;
    RTFLOAT humidity;
    bool temperatureValid;
    RTFLOAT temperature;
} HUMIDITY_DATA;

typedef struct
{
    uint64_t timestamp;
    bool pressureValid;
    RTFLOAT pressure;
    bool temperatureValid;
    RTFLOAT temperature;
} PRESSURE_DATA;

typedef struct
{
    RTVector3 worldAcceleration;
    RTVector3 worldVelocity;
    RTVector3 worldPosition;
    RTVector3 worldVelocityDrift;
    RTVector3 residuals;
    bool motion;
} MOTION_DATA;

#endif // _RTIMULIBDEFS_H
