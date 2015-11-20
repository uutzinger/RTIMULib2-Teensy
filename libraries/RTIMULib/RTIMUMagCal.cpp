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


#include "RTIMUMagCal.h"

RTIMUMagCal::RTIMUMagCal(RTIMUSettings *settings)
{
    m_settings = settings;
}

RTIMUMagCal::~RTIMUMagCal()
{

}

void RTIMUMagCal::magCalInit()
{
    magCalReset();
}

void RTIMUMagCal::magCalReset()
{
    m_magMin = RTVector3(RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN);
    m_magMax = RTVector3(RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX);
}

void RTIMUMagCal::newMinMaxData(const RTVector3& data)
{
    for (int i = 0; i < 3; i++) {
	    if (m_magMin.data(i) > data.data(i)) {
		    m_magMin.setData(i, data.data(i));
	    }

	    if (m_magMax.data(i) < data.data(i)) {
		    m_magMax.setData(i, data.data(i));
	    }
    }
}

bool RTIMUMagCal::magCalValid()
{
    bool valid = true;

     for (int i = 0; i < 3; i++) {
        if (m_magMax.data(i) < m_magMin.data(i))
            valid = false;
    }
    return valid;

}

void RTIMUMagCal::magCalSaveMinMax()
{
    m_settings->m_compassCalValid = true;
    m_settings->m_compassCalMin = m_magMin;
    m_settings->m_compassCalMax = m_magMax;
    m_settings->m_compassCalEllipsoidValid = false;
    m_settings->saveSettings();
}
