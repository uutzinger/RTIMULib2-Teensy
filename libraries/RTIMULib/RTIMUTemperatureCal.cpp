////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
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


// UU: This code was create to
// provide temperature bias compensation


#include "RTIMUTemperatureCal.h"

RTIMUTemperatureCal::RTIMUTemperatureCal(RTIMUSettings *settings)
{
    m_settings = settings;
}

RTIMUTemperatureCal::~RTIMUTemperatureCal()
{

}

void RTIMUTemperatureCal::temperatureCalInit()
{

}

void RTIMUTemperatureCal::temperatureCalReset()
{
}

bool RTIMUTemperatureCal::newData(const RTVector3& accel, const RTVector3& gyro, const RTVector3& mag, const RTFLOAT& temperature)
{
    return true;
}

bool RTIMUTemperatureCal::temperatureCalValid()
{
    return true;
}

TEMPERATURE_CAL_DATA RTIMUTemperatureCal::removeTemperatureCalData()
{
    TEMPERATURE_CAL_DATA ret;
    return ret;
}

bool RTIMUTemperatureCal::temperatureCalSaveRaw(const char *ellipsoidFitPath)
{
  return true;
}

bool RTIMUTemperatureCal::temperatureCalSaveCorr(const char *ellipsoidFitPath)
{
  return true;
}
