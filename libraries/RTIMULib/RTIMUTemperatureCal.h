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


#ifndef _RTIMUTEMPERATURECAL_H
#define	_RTIMUTEMPERATURECAL_H

#include "RTIMUCalDefs.h"
#include "RTIMULib.h"

typedef struct
{
    RTVector3 accel;
    RTVector3 mag;
    RTVector3 gyro;
    RTFLOAT temperature;
} TEMPERATURE_CAL_DATA;

//  RTIMUAccelCal is a helper class for performing accelerometer calibration

class RTIMUTemperatureCal
{
    
public:
    RTIMUTemperatureCal(RTIMUSettings *settings);
    virtual ~RTIMUTemperatureCal();

    //  This should be called at the start of the calibration process
    //  Loads previous values if available
    void temperatureCalInit();

    //  This should be called to clear for a new run
    void temperatureCalReset();

    // accelCalValid() checks if all values are reasonable. Should be called before saving
    bool temperatureCalValid();

    //  temperatureCalSave() should be called at the end of the process to save the cal data
    //  to the settings file. Returns false if invalid data
    bool temperatureCalSave();

    // temperatureCalSaveRaw saves the temperature data for later octave fitting
    // Returns true if everything worked correctly.
    bool temperatureCalSaveRaw(const char *ellipsoidFitPath);

    // temperatureCalSaveCorr loads the correction data produced by the octave fit program and saves it in the
    // .ini file
    bool temperatureCalSaveCorr(const char *ellipsoidFitPath);

    // newData is used to save data to the accel,gyro,compass sample array
    bool newData(const RTVector3& accel, const RTVector3& gyro, const RTVector3& mag, const RTFLOAT& temperature);

    // these vars used during the calibration process

    RTIMUSettings *m_settings;

    TEMPERATURE_CAL_DATA m_temperatureCalSamples[RTIMUCALDEFS_MAX_TEMPERATURE_SAMPLES];
    
    int m_temperatureCalInIndex;
    int m_temperatureCalOutIndex;
    int m_temperatureCalCount;
    
    RTFLOAT m_temperatureMax;
    RTFLOAT m_temperatureMin;;

private:
     TEMPERATURE_CAL_DATA removeTemperatureCalData();                           // takes an entry out of the buffer
};

#endif // _RTIMUACCELCAL_H
