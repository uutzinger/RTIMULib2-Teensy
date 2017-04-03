////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
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

// UU: This is new driver for RTIMULib

#include "RTPressureMS5803.h"

RTPressureMS5803::RTPressureMS5803(RTIMUSettings *settings) : RTPressure(settings)
{
}

RTPressureMS5803::~RTPressureMS5803()
{
}

int RTPressureMS5803::pressureGetPollInterval()
{
    // return (400 / 122); // available: 0.54 / 1.06 / 2.08 / 4.13 / 8.22 ms
    // osr 4096 takes 8.22ms
    return (3);
}

bool RTPressureMS5803::reset()
// Reset device I2C
{
    unsigned char cmd = MS5611_CMD_RESET;
    if (!m_settings->HALRead(m_pressureAddr, cmd, 0, 0, "Failed to reset MS5803")) {
        return false;
    } else {
        return true;
    }

}

bool RTPressureMS5803::pressureInit()
{
    unsigned char cmd  = MS5611_CMD_PROM;
    unsigned char data[2];

    m_pressureAddr = m_settings->m_I2CPressureAddress;

    // get calibration data
    // skip first and last entry in PROM table
    // C0= 0
    // C1= 44428
    // C2= 40264
    // C3= 27988
    // C4= 27027
    // C5= 32849
    // C6= 28630
    // C7= 5

    for (int i = 0; i <= 7; i++) {
        if (!m_settings->HALRead(m_pressureAddr, cmd, 2, data, "Failed to read MS5803 calibration data"))
            return false;
        m_calData[i] = (((uint16_t)data[0]) << 8) | ((uint16_t)data[1]);
        //printf("Cal index: %d, data: %d\n", i, m_calData[i]);
        cmd += 2;
    }
    m_state = MS5803_STATE_IDLE;
    
    return true;
}
    
bool RTPressureMS5803::pressureRead()
{
    uint8_t data[3];
    int32_t deltaT;
    int32_t temperature;
    int64_t offset;
    int64_t sens;
    int64_t T2;
    int64_t offset2;
    int64_t sens2;
    
    bool validReadings = false;

    switch (m_state) {
        case MS5803_STATE_IDLE:
        // start pressure conversion with maximum precision 4096
        if (!m_settings->HALWrite(m_pressureAddr, MS5611_CMD_CONV_D1, 0, 0, "Failed to start MS5803 pressure conversion")) {
            return false;
        } else {
            m_state = MS5803_STATE_PRESSURE;
            m_timer = RTMath::currentUSecsSinceEpoch();
        }
        break;

        case MS5803_STATE_PRESSURE:
        if ((RTMath::currentUSecsSinceEpoch() - m_timer) < 9040) // need to wait 10ms until conversion complete at highest precision
            return false;  // not time yet
        if (!m_settings->HALRead(m_pressureAddr, MS5611_CMD_ADC, 3, data, "Failed to read MS5803 pressure")) {
            return false;
        }
        m_D1 = ((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + (uint32_t)data[2];

        //printf("D1: %ld\n", m_D1);

        // start temperature conversion

        if (!m_settings->HALWrite(m_pressureAddr, MS5611_CMD_CONV_D2, 0, 0, "Failed to start MS5803 temperature conversion")) {
            return false;
        } else {
            m_state = MS5803_STATE_TEMPERATURE;
            m_timer = RTMath::currentUSecsSinceEpoch();
        }
        break;

        case MS5803_STATE_TEMPERATURE:
        if ((RTMath::currentUSecsSinceEpoch() - m_timer) < 9040) // takes 10ms until conversion at highest precision
           return false;  // not time yet
        if (!m_settings->HALRead(m_pressureAddr, MS5611_CMD_ADC, 3, data, "Failed to read MS58031 temperature")) {
           return false;
        }
		
        m_D2 = ((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + (uint32_t)data[2];
        //printf("D2: %ld\n", m_D2);

        //  now calculate temperature
        deltaT = m_D2 - ((int32_t)m_calData[5] << 8);
        temperature = (((int64_t)deltaT * m_calData[6]) >> 23) + 2000;
        
        //printf("deltaT: %ld\n", deltaT);
        //printf("temperature: %ld\n", temperature);


        //  do second order temperature compensation
        if (temperature < 2000) {
            // low temperature below 20C
            T2 = 3 * (((int64_t)deltaT * deltaT) >> 33);
            offset2 = 3 * ((temperature - 2000) * (temperature - 2000)) / 2;
            sens2 = 5 * ((temperature - 2000) * (temperature - 2000)) / 8;
            if (temperature < -1500) { // below -15C
                offset2 = offset2 + 7 * ((temperature + 1500) * (temperature + 1500));
                sens2 =  sens2    + 4 * ((temperature + 1500) * (temperature + 1500));
            }
        } else { // above 20C
            T2 = 7 * ((uint64_t)deltaT * deltaT)/pow(2,37); 
            offset2 = (temperature - 2000) * (temperature - 2000) / 16;
            sens2 = 0;
        }

        // Now bring all together and apply offsets
        offset = ((int64_t)m_calData[2] << 16) + (((m_calData[4] * (int64_t)deltaT)) >> 7);
        sens   = ((int64_t)m_calData[1] << 15) + (((m_calData[3] * (int64_t)deltaT)) >> 8);

        //printf("T2: %lld\n", T2);
        //printf("offset2: %lld\n", offset2);
        //printf("sens2: %lld\n", sens2);

        temperature = temperature - T2;
        offset = offset - offset2;
        sens = sens - sens2;

        //printf("temperature calib: %lld\n", temperature);
        //printf("offset calib: %lld\n", offset);
        //printf("sens calib: %lld\n", sens);

        // now lets calculate temperature compensated pressure
        m_pressureData.pressure = (RTFLOAT)(((m_D1 * sens)/ 2097152 - offset) / 32768) / 10.0;
        m_pressureData.temperature = (RTFLOAT)temperature/100.0;
        m_pressureData.temperatureValid = true;
        m_pressureData.pressureValid =  true;
		m_pressureData.timestamp = RTMath::currentUSecsSinceEpoch();

        //printf("Temp: %f, pressure: %f\n", m_pressureData.temperature, m_pressureData.pressure);

        validReadings = true;
        m_state = MS5803_STATE_IDLE;
        break;
    }

	if (validReadings) {
      return true;
	} else {
      return false;
    }
}
