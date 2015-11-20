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

#ifndef _RTIMUHAL_H
#define	_RTIMUHAL_H

#include <math.h>
#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

#define HAL_QUIET

#ifndef HAL_QUIET
#define HAL_INFO(m) Serial.printf(m);
#define HAL_INFO1(m, x)  Serial.printf(m, x);
#define HAL_INFO2(m, x, y)  Serial.printf(m, x, y);
#define HAL_INFO3(m, x, y, z)  Serial.printf(m, x, y, z);
#define HAL_INFO4(m, x, y, z, a)  Serial.printf(m, x, y, z, a);
#define HAL_INFO5(m, x, y, z, a, b)  Serial.printf(m, x, y, z, a, b);
#define HAL_ERROR(m)     Serial.printf(m);
#define HAL_ERROR1(m, x)     Serial.printf(m, x);
#define HAL_ERROR2(m, x, y)     Serial.printf(m, x, y);
#define HAL_ERROR3(m, x, y, z)     Serial.printf(m, x, y, z);
#define HAL_ERROR4(m, x, y, z, a)     Serial.printf(m, x, y, z, a);

#else

#define HAL_INFO(m) 
#define HAL_INFO1(m, x)
#define HAL_INFO2(m, x, y)
#define HAL_INFO3(m, x, y, z)
#define HAL_INFO4(m, x, y, z, a)
#define HAL_INFO5(m, x, y, z, a, b)
#define HAL_ERROR(m)     Serial.printf(m);
#define HAL_ERROR1(m, x)     Serial.printf(m, x);
#define HAL_ERROR2(m, x, y)     Serial.printf(m, x, y);
#define HAL_ERROR3(m, x, y, z)     Serial.printf(m, x, y, z);
#define HAL_ERROR4(m, x, y, z, a)     Serial.printf(m, x, y, z, a);

#endif


class RTIMUHal
{
public:
    RTIMUHal();
    virtual ~RTIMUHal();

    bool m_busIsI2C;                                        // true if I2C bus in use, false if SPI in use
    unsigned char m_I2CBus;                                 // I2C bus of the imu
    unsigned char m_SPIBus;                                 // SPI bus of the imu
    unsigned char m_SPISelect;                              // SPI select line
    unsigned int m_SPISpeed;                                // speed of interface

    bool HALOpen();
    void HALClose();
    bool HALRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                 unsigned char *data, const char *errorMsg);
    bool HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                  unsigned char length, unsigned char const *data, const char *errorMsg);
    bool HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                  unsigned char const data, const char *errorMsg);

    void delayMs(int milliSeconds);

private:
    void I2CClose();
    void SPIClose();

    SPISettings m_SPISettings;
};

#endif // _RTIMUHAL_H
