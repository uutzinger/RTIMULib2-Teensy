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

#include "RTIMUHal.h"
#include "I2Cdev.h"
#include <SPI.h>

RTIMUHal::RTIMUHal()
{
}

RTIMUHal::~RTIMUHal()
{
}

bool RTIMUHal::HALOpen()
{
    if (m_busIsI2C)
        return true;

    SPI.begin();
    pinMode(m_SPISelect, OUTPUT);
    m_SPISettings = SPISettings(m_SPISpeed, MSBFIRST, SPI_MODE0);
    return true;
}

void RTIMUHal::HALClose()
{
    I2CClose();
    SPIClose();
}

void RTIMUHal::I2CClose()
{
}

void RTIMUHal::SPIClose()
{
    SPI.end();
}

bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                 unsigned char *data, const char *errorMsg)
{
    if (m_busIsI2C) {
        if (I2Cdev::readBytes(slaveAddr, regAddr, length, data, 10) == length)
             return true;

        if (strlen(errorMsg) > 0)
            HAL_ERROR1("I2C read failed - %s\n", errorMsg);

        return false;
    } else {
        SPI.beginTransaction(m_SPISettings);
        digitalWrite(m_SPISelect, LOW);
        SPI.transfer(regAddr | 0x80);
        for (int i = 0; i < length; i++)
            data[i] = SPI.transfer(0);
        digitalWrite(m_SPISelect, HIGH);
        SPI.endTransaction();
        return true;
    }
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                  unsigned char length, unsigned char const *data, const char *errorMsg)
{
    if (m_busIsI2C) {
        if (I2Cdev::writeBytes(slaveAddr, regAddr, length, (unsigned char *)data) > 0)
            return true;

        if (strlen(errorMsg) > 0)
            HAL_ERROR1("I2C write failed - %s\n", errorMsg);

        return false;
    } else {
        SPI.beginTransaction(m_SPISettings);
        digitalWrite(m_SPISelect, LOW);
        SPI.transfer(regAddr);
        for (int i = 0; i < length; i++)
            SPI.transfer(data[i]);
        digitalWrite(m_SPISelect, HIGH);
        SPI.endTransaction();
        return true;
    }
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                  unsigned char const data, const char *errorMsg)
{
    if (m_busIsI2C) {
        if (I2Cdev::writeByte(slaveAddr, regAddr, data))
            return true;

        if (strlen(errorMsg) > 0)
            HAL_ERROR1("I2C write failed - %s\n", errorMsg);

        return false;
    } else {
        SPI.beginTransaction(m_SPISettings);
        digitalWrite(m_SPISelect, LOW);
        SPI.transfer(regAddr);
        SPI.transfer(data);
        digitalWrite(m_SPISelect, HIGH);
        SPI.endTransaction();
        return true;
    }
}

void RTIMUHal::delayMs(int milliSeconds)
{
    delay(milliSeconds);
}

