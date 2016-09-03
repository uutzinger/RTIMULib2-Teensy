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

#include "RTPressureMS5837.h"

RTPressureMS5837::RTPressureMS5837(RTIMUSettings *settings) : RTPressure(settings)
{
    m_validReadings = false;
}

RTPressureMS5837::~RTPressureMS5837()
{
}

bool RTPressureMS5837::pressureReset()
// Reset device I2C
{
    unsigned char cmd = MS5837_CMD_RESET;
    if (!m_settings->HALRead(m_pressureAddr, cmd, 0, 0, "Failed to reset MS5837")) {
        return false;
    } else {
        return true;
    }// not expecting any data to return -> 0, 0
}

bool RTPressureMS5837::pressureInit()
{
    unsigned char cmd = MS5837_CMD_PROM;
    unsigned char data[2];
    m_pressureAddr = m_settings->m_I2CPressureAddress;

    pressureReset();

    // get calibration data

    for (int i = 0; i < 7; i++) {
        if (!m_settings->HALRead(m_pressureAddr, cmd, 2, data, "Failed to read MS5611 calibration data"))
            return false;
        m_calData[i] = (((uint16_t)data[0]) << 8) | ((uint16_t)data[1]);
        // printf("Cal index: %d, data: %d\n", i, m_calData[i]);
        cmd += 2;
    }

	// Verify that data is correct with CRC
	uint8_t crcRead = m_calData[0] >> 12;
	uint8_t crcCalculated = crc4(m_calData);

	if ( crcCalculated == crcRead ) {
		// Success
	} else {
		// Failure - try again?
	}
	
    m_state = MS5837_STATE_IDLE;

    return true;
}

bool RTPressureMS5837::pressureRead(RTIMU_DATA& data)
{
    data.pressureValid = false;
    data.pressureTemperatureValid = false;
    data.pressureTemperature = 0;
    data.pressure = 0;

    if (m_state == MS5837_STATE_IDLE) {
        // start pressure conversion
        if (!m_settings->HALWrite(m_pressureAddr, MS5837_CMD_CONV_D1, 0, 0, "Failed to start MS5837 pressure conversion")) {
            return false;
        } else {
            m_state = MS5837_STATE_PRESSURE;
            m_timer = RTMath::currentUSecsSinceEpoch();
        }
    }

    pressureBackground();

    if (m_validReadings) {
        data.pressureValid = true;
        data.pressureTemperatureValid = true;
        data.pressureTemperature = m_temperature;
        data.pressure = m_pressure;
    }
    return true;
}


void RTPressureMS5837::pressureBackground()
{
    uint8_t data[3];
	int64_t T2;
	int64_t offset2;
	int64_t sens2;
    int64_t deltaT;
    int64_t sens;
    int64_t offset;
    int32_t temperature;

    switch (m_state) {
        case MS5837_STATE_IDLE:
        break;

        case MS5837_STATE_PRESSURE:
        if ((RTMath::currentUSecsSinceEpoch() - m_timer) < 10000)
            break;                                          // not time yet
        if (!m_settings->HALRead(m_pressureAddr, MS5837_CMD_ADC, 3, data, "Failed to read MS5837 pressure")) {
            break;
        }
        m_D1 = (((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | ((uint32_t)data[2]);

        // start temperature conversion

        if (!m_settings->HALWrite(m_pressureAddr, MS5837_CMD_CONV_D2, 0, 0, "Failed to start MS5837 temperature conversion")) {
            break;
        } else {
            m_state = MS5837_STATE_TEMPERATURE;
            m_timer = RTMath::currentUSecsSinceEpoch();
        }
        break;

        case MS5837_STATE_TEMPERATURE:
        if ((RTMath::currentUSecsSinceEpoch() - m_timer) < 10000)
            break;                                          // not time yet
        if (!m_settings->HALRead(m_pressureAddr, MS5837_CMD_ADC, 3, data, "Failed to read MS5837 temperature")) {
            break;
        }
        m_D2 = (((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | ((uint32_t)data[2]);

        //  call this function for testing only
        //  should give T = 2000 (20.00C) and pressure 110002 (1100.02hPa)
        //  setTestData();

        //  now calculate the real values

        deltaT = (int32_t)m_D2 - ((int32_t)m_calData[5] << 8);
        sens   = (((int64_t)m_calData[1]) << 15) + (((int64_t)m_calData[3] * deltaT) >> 8);
        offset = (((int64_t)m_calData[2]) << 16) + (((int64_t)m_calData[4] * deltaT) >> 7);

        temperature = 2000L + ((deltaT * (int64_t)m_calData[6]) >> 23); // note - this still needs to be divided by 100
		
        //  do second order temperature compensation

        if (temperature < 2000) { // low temp
            T2 = (3 * (deltaT * deltaT)) >> 33;
            offset2 = (3 * ((temperature - 2000) * (temperature - 2000))) >> 1;
            sens2 = (5 * ((temperature - 2000) * (temperature - 2000))) >> 3;
            if (temperature < -1500) { // very low temp
                offset2 += 7 * (temperature + 1500) * (temperature + 1500);
                sens2 += 4 * ((temperature + 1500) * (temperature + 1500));
            }
        } else {
            T2 = (2 * (deltaT * deltaT)) >> 37;
			offset2 = (temperature - 2000) * (temperature - 2000) >> 4;
			sens2 = 0;
        }

        offset -= offset2;
        sens -=sens2;
		temperature -= T2;

        m_pressure = (RTFLOAT)(((((int64_t)m_D1 * sens) >> 21) - offset) >> 13) / (RTFLOAT)10.0; // mbar
        m_temperature = (RTFLOAT)temperature/(RTFLOAT)100; // deg C

        // printf("Temp: %f, pressure: %f\n", m_temperature, m_pressure);

        m_validReadings = true;
        m_state = MS5837_STATE_IDLE;
        break;
    }
}

void RTPressureMS5837::setTestData()
{
	
	m_calData[0] = 0;
	m_calData[1] = 34982;
	m_calData[2] = 36352;
	m_calData[3] = 20328;
	m_calData[4] = 22354;
	m_calData[5] = 26646;
	m_calData[6] = 26146;
	m_calData[7] = 0;

	m_D1 = 4958179;
	m_D2 = 6815414;

}

uint8_t RTPressureMS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF0);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}