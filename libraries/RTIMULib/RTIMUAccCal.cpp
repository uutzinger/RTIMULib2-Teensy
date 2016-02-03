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


#include "RTIMUAccCal.h"

RTIMUAccCal::RTIMUAccCal(RTIMUSettings *settings)
{
    m_settings = settings;
}

RTIMUAccCal::~RTIMUAccCal()
{
}

void RTIMUAccCal::accCalInit()
{
    accCalReset();
}

void RTIMUAccCal::accCalReset()
{
  if (m_settings->m_accelCalValid == false) {
	m_settings->m_accelCalMax.setX(1.0f);
	m_settings->m_accelCalMax.setY(1.0f);
	m_settings->m_accelCalMax.setZ(1.0f);
	m_settings->m_accelCalMin.setX(-1.0f);
	m_settings->m_accelCalMin.setY(-1.0f);
	m_settings->m_accelCalMin.setZ(-1.0f);
  }
}

void RTIMUAccCal::accCalSaveMinMax()
{
    m_settings->m_accelCalValid = true;
    m_settings->m_accelCalEllipsoidValid = false;
    m_settings->saveSettings();
}
