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


#ifndef _RTIMUACCCAL_H
#define	_RTIMUACCCAL_H

#include "RTIMUCalDefs.h"
#include "RTIMULib.h"

class RTIMUAccCal
{

public:
    RTIMUAccCal(RTIMUSettings *settings);
    virtual ~RTIMUAccCal();

    //  This should be called at the start of the calibration process
    //  Loads previous values if available
    void accelCalInit();                                      // inits everything
	
    //  This should be called to clear enabled axes for a new run
    void accelCalReset();


    //  This clear settings not just accel calibration temporary vars
	void accCalReset();                                     // clears everything
    void accCalXReset();                                    // clears X Cal 
    void accCalYReset();                                    // clears Y Cal 
    void accCalZReset();                                    // clears Z Cal 

    //  accelCalEnable() controls which axes are active - largely so that each can be done separately
    void accelCalEnable(int axis, bool enable);

    // newAccalCalData() adds a new sample for processing but only the axes enabled previously
    void newMinMaxData(const RTVector3& data);            // adds a new accel sample

    // accelCalValid() checks if all values are reasonable. Should be called before saving
    bool accelCalValid();

    // magCalSaveMinMax() saves the current min/max values to settings
    bool accelCalSaveMinMax();

    // these vars used during the calibration process
	
    bool m_accelCalValid;                                   // true if the mag min/max data valid
    RTVector3 m_accelMin;                                   // the min values
    RTVector3 m_accelMax;                                   // the max values

    RTVector3 m_averageValue;                               // averaged value actually used

    bool m_accelCalEnable[3];                               // the enable flags

    RTIMUSettings *m_settings;

    const RTVector3& getMin()      { return m_accelMin; } // get accel data in gs
    const RTVector3& getMax()      { return m_accelMax; } // get accel data in gs

private:
};

#endif // _RTIMUACCCAL_H
