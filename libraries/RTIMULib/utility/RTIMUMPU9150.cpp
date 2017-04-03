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

// UU: This code was modified to: 
// read IMU temperature
// read temperature and compass from FIFO
// attempted to repair error in cache mode
// changed FIFO reset to also reset signal path and DMP
// moved "wait 50ms" to location after reset
// added humidity, humidity sensor temperature, pressure, pressure sensor temperature
//  to imuData structure. This was added to imuData structure instead of 
//  separate structure because other sensor data had already been added there
//
/* RTLIB: Handling of IMU 9150 at startup
 * **************************************

 *  Init:
 *  -----
 - Set variables from ini file and boot conditions
 - Open I2C
 - Power Mgmt 1, pull out of sleep through reset
 - Wait 100ms
 - Power Mgmt 1, stop reset
 - Read ID of chip
 - Set bandwidth 260Hz
 - Set gyro 1000deg/s
 - Set accel +/-8g (no HPF, no motion detection)
 - * Compass Configure Subroutine (ends with bypass off)
 - Power Mgmt 1 set clock source to gyro x
 - Power Mgmt 2 set no standby
 - * Reset FIFO Subroutine
 - Set Variables for gyro bias computation
 
 * Config Compass (either AKA or 5833 chip)
 * --------------
 - Bypass ON
 - Powerdown Compass
 - Fuse ROM access
 - Read ASA calibration data, if not available, use other 5833 compass mode
   if 5833
 -  CONF_A 38
 -  CONF_B 20
 -  MODE 00
   if AKA
 -  Powerdown
 -  Bypass OFF
 -  Set I2C_MASTER_CTRL on 9150 to wait until data from external sensor (compass) is available
 -  Set MPU9150_I2C_SLV0_ADDR to compass
 -  Set MPU9150_I2C_SLV0_REG to compass data ready register, will read data ready register, data itself (6 bytes) and data error register
 -  Set MPU9150_I2C_SLV0_CTRL to x88, slave enable, number of bytes set to 8
 -  Set MPU9150_I2C_SLV1_ADDR to compass
 -  Set MPU9150_I2C_SLV1_REG to compass control register
 -  Set MPU9150_I2C_SLV1_CTRL x81, slave enable, number of bytes set to 1
 -  Set MPU9150_I2C_SLV1_DO (data out) to 1, which enables single measurement mode of compass

 *  BypassOFF
 * ----------
 - Read USER_CTRL 
 - Set USER_CTRL I2C Master Control bit ON, I2C uses SDA and SCL
 - delay 50ms
 - Set INT_PIN level to active low, user can not access auxiliary I2C bus

 *  BypassON
 * ---------
 - Read USER_CTRL 
 - Set USER_CTRL I2C Master Control OFF
 - Set register so that user can access auxiliary I2C bus
 
 *  Reset FIFO
 * -----------
 - Modify MPU9150_INT_ENABLE to disable FIFO interrupt
 - Modify MPU9150_FIFO_EN to disable FIFO
 - Set MPU9150_USER_CTRL 0 to disable FIFO
 - Set MPU9150_USER_CTRL 0x04 which resets FIFO, 
 * [MIGHT NEED TO ALSO RESET SIGNAL PATH AND DMP] 
 - Set MPU9150_USER_CTRL, 0x60 which sets sets I2C to master mode (bit 5) and sets FIFO ENABLE (bit 6)
 - Delay 50ms
 - Set MPU9150_INT_ENABLE to 1 which enables FIFO interrupt (but do not enable FIFO overflow, ic2 master, motion interrupt)
 - Set MPU9150_FIFO_EN, 0xf8, sets which registers to put into FIFO
   //e.g.TEMP, XG, YG, ZG, ACCEL, SLV2, SLV1, SLV0
   //f8  1     1   1   1   1      0     0     0
   //78  0     1   1   1   1      0     0     0
 */

#include "RTIMUMPU9150.h"
#include "RTIMUSettings.h"

RTIMUMPU9150::RTIMUMPU9150(RTIMUSettings *settings) : RTIMU(settings)
{

}

RTIMUMPU9150::~RTIMUMPU9150()
{
}

bool RTIMUMPU9150::setLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9150_LPF_256:
    case MPU9150_LPF_188:
    case MPU9150_LPF_98:
    case MPU9150_LPF_42:
    case MPU9150_LPF_20:
    case MPU9150_LPF_10:
    case MPU9150_LPF_5:
        m_lpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal MPU9150 lpf %d\n", lpf);
        return false;
    }
}


bool RTIMUMPU9150::setSampleRate(int rate)
{
    if ((rate < MPU9150_SAMPLERATE_MIN) || (rate > MPU9150_SAMPLERATE_MAX)) {
        HAL_ERROR1("Illegal sample rate %d\n", rate);
        return false;
    }
    m_sampleRate = rate;
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    return true;
}

bool RTIMUMPU9150::setCompassRate(int rate)
{
    if ((rate < MPU9150_COMPASSRATE_MIN) || (rate > MPU9150_COMPASSRATE_MAX)) {
        HAL_ERROR1("Illegal compass rate %d\n", rate);
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUMPU9150::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9150_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case MPU9150_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case MPU9150_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case MPU9150_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        HAL_ERROR1("Illegal MPU9150 gyro fsr %d\n", fsr);
        return false;
    }
}

bool RTIMUMPU9150::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9150_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case MPU9150_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case MPU9150_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case MPU9150_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        HAL_ERROR1("Illegal MPU9150 accel fsr %d\n", fsr);
        return false;
    }
}


bool RTIMUMPU9150::IMUInit()
{
    unsigned char result;

    m_firstTime = true;

#ifdef MPU9150_CACHE_MODE
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif

    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.motion = true;
    m_imuData.temperatureValid = false;
    m_imuData.temperature = 0.0;

    //  configure IMU configuration variables

    m_slaveAddr = m_settings->m_I2CSlaveAddress;
    
    m_temperature_previous = 0.0;

    setSampleRate(m_settings->m_MPU9150GyroAccelSampleRate);
    setCompassRate(m_settings->m_MPU9150CompassSampleRate);
    setLpf(m_settings->m_MPU9150GyroAccelLpf);
    setGyroFsr(m_settings->m_MPU9150GyroFsr);
    setAccelFsr(m_settings->m_MPU9150AccelFsr);

    setCalibrationData(); // adjust calibration data

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;

    //  reset the MPU9150

    // 0x80: RESET (1),SLEEP (0), CYCLE (1), -, TEMP_DIS (0), CLKSEL[0..2] (0,0,0)]
    // Pull the IMU out of sleep mode.
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_PWR_MGMT_1, 0x80, "Failed to initiate MPU9150 reset"))
        return false;

    m_settings->delayMs(100);

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_PWR_MGMT_1, 0x00, "Failed to stop MPU9150 reset"))
        return false;

    if (!m_settings->HALRead(m_slaveAddr, MPU9150_WHO_AM_I, 1, &result, "Failed to read MPU9150 id"))
        return false;

    if (result != MPU9150_ID) {
        HAL_ERROR1("Incorrect MPU9150 id %d\n", result);
        return false;
    }

    //  now configure the various components

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_LPF_CONFIG, m_lpf, "Failed to set lpf")) 
        return false; // sets DLPF filters, external sync input disabled

    if (!setSampleRate())
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_GYRO_CONFIG, m_gyroFsr, "Failed to set gyro fsr"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_ACCEL_CONFIG, m_accelFsr, "Failed to set accel fsr"))
         return false;

    //  now configure compass

    if (!configureCompass())
        return false;

    //  enable the sensors

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_PWR_MGMT_1, 1, "Failed to set pwr_mgmt_1")) 
        return false; // select clock source, X axis PLL, allows for sleep cycling

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_PWR_MGMT_2, 0, "Failed to set pwr_mgmt_2")) 
         return false; // no standby

    //  select the data to go into the FIFO and enable

    if (!resetFifo())
        return false;

    gyroBiasInit();

    HAL_INFO("MPU9150 init complete\n");
    return true;
}

bool RTIMUMPU9150::configureCompass()
{
    unsigned char asa[3];
    unsigned char id;

    m_compassIs5883 = false;
    m_compassDataLength = 8; // registers: status one, mx, my, mz and status two for AKA
    m_fifoChunkLength = MPU9150_FIFO_CHUNK_SIZE; // adjust FIFO chunk length, either AKA or 5883

    bypassOn();

    // get fuse ROM data

    if (!m_settings->HALWrite(AK8975_ADDRESS, AK8975_CNTL, 0, "Failed to set compass in power down mode 1")) {
        bypassOff();
        return false;
    }

    if (!m_settings->HALWrite(AK8975_ADDRESS, AK8975_CNTL, 0x0f, "Failed to set compass in fuse ROM mode")) {
        bypassOff();
        return false;
    }

    if (!m_settings->HALRead(AK8975_ADDRESS, AK8975_ASAX, 3, asa, "")) {

        //  check to see if an HMC5883L is fitted

        if (!m_settings->HALRead(HMC5883_ADDRESS, HMC5883_ID, 1, &id, "Failed to find 5883")) {
            bypassOff();

            //  this is returning true so that MPU-6050 by itself will work

            HAL_INFO("Detected MPU-6050 without compass\n");

            m_imuData.compassValid = false;
            m_compassDataLength = 0;
            return true;
        }
        if (id != 0x48) {                                   // incorrect id for HMC5883L

            bypassOff();

            //  this is returning true so that MPU-6050 by itself will work

            HAL_INFO("Detected MPU-6050 without compass\n");

            m_imuData.compassValid = false;
            m_compassDataLength = 0;
            return true;
        }

        // HMC5883 is present - use that

        if (!m_settings->HALWrite(HMC5883_ADDRESS, HMC5883_CONFIG_A, 0x38, "Failed to set HMC5883 config A")) {
            bypassOff();
            return false;
        }

        if (!m_settings->HALWrite(HMC5883_ADDRESS, HMC5883_CONFIG_B, 0x20, "Failed to set HMC5883 config B")) {
            bypassOff();
            return false;
        }

        if (!m_settings->HALWrite(HMC5883_ADDRESS, HMC5883_MODE, 0x00, "Failed to set HMC5883 mode")) {
            bypassOff();
            return false;
        }

        HAL_INFO("Detected MPU-6050 with HMC5883\n");

        m_compassDataLength = 6;
        m_fifoChunkLength -= 2; // 5833 does not have status registers
        m_compassIs5883 = true;
    } else {

        //  convert asa to usable scale factor

        m_compassAdjust[0] = ((float)asa[0] - 128.0) / 256.0 + 1.0f;
        m_compassAdjust[1] = ((float)asa[1] - 128.0) / 256.0 + 1.0f;
        m_compassAdjust[2] = ((float)asa[2] - 128.0) / 256.0 + 1.0f;

        if (!m_settings->HALWrite(AK8975_ADDRESS, AK8975_CNTL, 0, "Failed to set compass in power down mode 2")) {
            bypassOff();
            return false;
        }
    }

    bypassOff();

    //  now set up MPU9150 to talk to the compass chip

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_MST_CTRL, 0x40, "Failed to set I2C master mode"))
        return false;

    if (m_compassIs5883) {
        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV0_ADDR, 0x80 | HMC5883_ADDRESS, "Failed to set slave 0 address"))
                return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV0_REG, HMC5883_DATA_X_HI, "Failed to set slave 0 reg"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV0_CTRL, 0x86, "Failed to set slave 0 ctrl"))
            return false;
    } else {
        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV0_ADDR, 0x80 | AK8975_ADDRESS, "Failed to set slave 0 address"))
                return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV0_REG, AK8975_ST1, "Failed to set slave 0 reg"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV0_CTRL, 0x88, "Failed to set slave 0 ctrl")) 
            return false; // read 8 bytes, enable I2C slave 

        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV1_ADDR, AK8975_ADDRESS, "Failed to set slave 1 address"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV1_REG, AK8975_CNTL, "Failed to set slave 1 reg"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV1_CTRL, 0x81, "Failed to set slave 1 ctrl"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV1_DO, 0x1, "Failed to set slave 1 DO"))
            return false;

    }

    // 0x03: enables decreased access rate for external sensor (RT code)
    // 0x80: external sensor data is delayed until all data is available.
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_MST_DELAY_CTRL, 0x3, "Failed to set mst delay"))
        return false;
    
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_YG_OFFS_TC, 0x80, "Failed to set yg offs tc"))
        return false; // Auxiliary I2C Voltage Level

    if (!setCompassRate())
        return false;
    
    return true;
}

bool RTIMUMPU9150::resetFifo()
{
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_INT_ENABLE, 0, "Writing int enable")) 
        return false; // disable FIFO interrupt
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_FIFO_EN, 0, "Writing fifo enable")) 
        return false; // disable FIFO
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_USER_CTRL, 0, "Writing user control")) 
        return false; // disable FIFO and master modes
                                                            //0x04 resets FIFO only (ORIGINAL CODE)
                                                            //0x0c resets FIFO and I2C_MST 
                                                            //0x0d resets FIFO, DMP and signal path
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_USER_CTRL, 0x0d, "Resetting fifo")) 
        return false; // reset FIFO while FIFO disabled

	m_settings->delayMs(50);

														   // 0x60 FIFO EN and I2C Master Mode
														   // 0x40 FIFO EN 
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_USER_CTRL, 0x60, "Enabling the fifo")) 
        return false; // set bit 5 (sets I2C to master mode) bit 6 (sets FIFO ENABLE)

	// m_settings->delayMs(50);
		
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_INT_ENABLE, 1, "Writing int enable")) 
        return false; // enable FIFO interrupt (but do not enable FIFO overflow, ic2 master, motion interrupt)

    //    TEMP, XG, YG, ZG, ACCEL, SLV2, SLV1, SLV0
    // f9 1     1   1   1   1      0     0     1
    // f8 1     1   1   1   1      0     0     0
    // 79 0     1   1   1   1      0     0     1
    // 78 0     1   1   1   1      0     0     0
    #if MPU9150_FIFO_WITH_TEMP == 1
        #if MPU9150_FIFO_WITH_COMPASS == 1 // compass and temp in fifo
			if (!m_settings->HALWrite(m_slaveAddr, MPU9150_FIFO_EN, 0xf9, "Failed to set FIFO enables"))  // with temperature and compass
				return false;
        #else // temp in fifo
			if (!m_settings->HALWrite(m_slaveAddr, MPU9150_FIFO_EN, 0xf8, "Failed to set FIFO enables"))  // with temperature
				return false;
        #endif
    #else
        #if MPU9150_FIFO_WITH_COMPASS == 1 // compass in fifo
            if (!m_settings->HALWrite(m_slaveAddr, MPU9150_FIFO_EN, 0x79, "Failed to set FIFO enables"))  // without temperature
                return false;
        #else // no compass and no temp in fifo
            if (!m_settings->HALWrite(m_slaveAddr, MPU9150_FIFO_EN, 0x78, "Failed to set FIFO enables"))  // without temperature
                return false;
        #endif
    #endif

    return true;
}

bool RTIMUMPU9150::bypassOn()
{
    unsigned char userControl;

    if (!m_settings->HALRead(m_slaveAddr, MPU9150_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl &= ~0x20;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_INT_PIN_CFG, 0x82, "Failed to write int_pin_cfg reg"))
        return false;

    m_settings->delayMs(50);
    return true;
}


bool RTIMUMPU9150::bypassOff()
{
    unsigned char userControl;

    if (!m_settings->HALRead(m_slaveAddr, MPU9150_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl |= 0x20;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_INT_PIN_CFG, 0x80, "Failed to write int_pin_cfg reg"))
         return false;

    m_settings->delayMs(50);
    return true;
}

bool RTIMUMPU9150::setSampleRate()
{
    int clockRate = 1000;

    if (m_lpf == MPU9150_LPF_256) // 256/260 Hz Gyro/Accel
        clockRate = 8000;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_SMPRT_DIV, (unsigned char)(clockRate / m_sampleRate - 1),
                  "Failed to set sample rate"))
        return false;

    return true;
}

bool RTIMUMPU9150::setCompassRate()
{
    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31)
        rate = 31;
    if (!m_settings->HALWrite(m_slaveAddr, MPU9150_I2C_SLV4_CTRL, rate, "Failed to set slave ctrl 4"))
         return false;
    return true;
}

int RTIMUMPU9150::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUMPU9150::IMURead()
{
    unsigned char fifoCount[2];
    unsigned int  count;
    unsigned char fifoData[MPU9150_FIFO_CHUNK_SIZE]; // FIFO CHUNK SIZE is maximum size
    #if MPU9150_FIFO_WITH_COMPASS == 0
    unsigned char compassData[8]; // compass data goes here if it is not coming in through FIFO
    #endif
    #if MPU9150_FIFO_WITH_TEMP == 0
    unsigned char temperatureData[2]; // if temperature data is not coming in through FIFO
    #endif

    if (!m_settings->HALRead(m_slaveAddr, MPU9150_FIFO_COUNT_H, 2, fifoCount, "Failed to read fifo count")) 
         return false;

    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];

    // Debug
    // printf("FIFO Count: %d, Cache Count: %d, FIFO Chunk Length: %d, Max Cache Size: %d\n",count, m_cacheCount, m_fifoChunkLength, MPU9150_CACHE_SIZE);
    // printf("Compass Data Length: %d \n", m_compassDataLength);  
	
    if (count == 1024) {
        HAL_INFO("MPU9150 fifo has overflowed");
        resetFifo();
        m_imuData.timestamp += m_sampleInterval * (1024 / m_fifoChunkLength + 1); // try to fix timestamp
        return false;
    }

#ifdef MPU9150_CACHE_MODE
    if ( (m_cacheCount == 0) && (count  < m_fifoChunkLength) ) 
        return false; // no new set of data available
    
    if ( (m_cacheCount == 0) && (count >= m_fifoChunkLength)  && (count < (MPU9150_CACHE_SIZE * m_fifoChunkLength)) )  {

        // special case of a small fifo and nothing cached - just handle as simple read
        if (!m_settings->HALRead(m_slaveAddr, MPU9150_FIFO_R_W, m_fifoChunkLength, fifoData, "Failed to read fifo data"))
            return false;

        #if MPU9150_FIFO_WITH_TEMP == 0 // read temp from registers
        if (!m_settings->HALRead(m_slaveAddr, MPU9150_TEMP_OUT_H, 2,
                            temperatureData, "Failed to read temperature data"))
            return false; 
        #endif

        #if MPU9150_FIFO_WITH_COMPASS == 0 // read compass without fifo
        if (!m_settings->HALRead(m_slaveAddr, MPU9150_EXT_SENS_DATA_00, m_compassDataLength, compassData, "Failed to read compass data"))
            return false;
        #endif

    } else {
        if (count >= (MPU9150_CACHE_SIZE * m_fifoChunkLength)) {
            if (m_cacheCount == MPU9150_CACHE_BLOCK_COUNT) {
                // all cache blocks are full - discard oldest and update timestamp to account for lost samples
                m_imuData.timestamp += m_sampleInterval * m_cache[m_cacheOut].count;
                if (++m_cacheOut == MPU9150_CACHE_BLOCK_COUNT)
                    m_cacheOut = 0;
                m_cacheCount--;
            }

            int blockCount = count / m_fifoChunkLength;   // number of chunks in fifo

            if (blockCount > MPU9150_CACHE_SIZE)
                blockCount = MPU9150_CACHE_SIZE;

            if (!m_settings->HALRead(m_slaveAddr, MPU9150_FIFO_R_W, m_fifoChunkLength * blockCount,
                                m_cache[m_cacheIn].data, "Failed to read fifo data"))
                return false;

            #if MPU9150_FIFO_WITH_TEMP == 0 // read temp from registers
            if (!m_settings->HALRead(m_slaveAddr, MPU9150_TEMP_OUT_H, 2,
                                m_cache[m_cacheIn].temperature, "Failed to read temperature data"))
                return false; 
            #endif
            
            #if MPU9150_FIFO_WITH_COMPASS == 0 // read compass from register
            if (!m_settings->HALRead(m_slaveAddr, MPU9150_EXT_SENS_DATA_00, m_compassDataLength, m_cache[m_cacheIn].compass, "Failed to read compass data"))
               return false;
            # endif

            m_cache[m_cacheIn].count = blockCount;
            m_cache[m_cacheIn].index = 0;

            m_cacheCount++;
            if (++m_cacheIn == MPU9150_CACHE_BLOCK_COUNT)
                m_cacheIn = 0;
        }

        //  now fifo has been read if necessary, get something to process

        if (m_cacheCount == 0) {
            printf("No data in Cache\n");
            return false; }

        memcpy(fifoData, m_cache[m_cacheOut].data + m_cache[m_cacheOut].index, m_fifoChunkLength);
        #if MPU9150_FIFO_WITH_COMPASS == 0
        memcpy(compassData, m_cache[m_cacheOut].compass, m_compassDataLength);
        #endif
        #if MPU9150_FIFO_WITH_TEMP == 0
        memcpy(temperatureData, m_cache[m_cacheOut].temperature, 2);            
        #endif

        m_cache[m_cacheOut].index += m_fifoChunkLength;

        if (--m_cache[m_cacheOut].count == 0) {
            //  this cache block is now empty

            if (++m_cacheOut == MPU9150_CACHE_BLOCK_COUNT)
                m_cacheOut = 0;
            m_cacheCount--;
        }
    }

#else

    if (count > m_fifoChunkLength * 40) {
        // more than 40 samples behind - going too slowly so discard some samples but maintain timestamp correctly
        while (count >= m_fifoChunkLength * 10) {
            if (!m_settings->HALRead(m_slaveAddr, MPU9150_FIFO_R_W, m_fifoChunkLength, fifoData, "Failed to read fifo data"))
                return false;
            count -= m_fifoChunkLength;
            m_imuData.timestamp += m_sampleInterval;
        }
    }

    if (count < m_fifoChunkLength)
        return false;

    if (!m_settings->HALRead(m_slaveAddr, MPU9150_FIFO_R_W, m_fifoChunkLength, fifoData, "Failed to read fifo data"))
        return false;

    #if MPU9150_FIFO_WITH_TEMP == 0
    if (!m_settings->HALRead(m_slaveAddr, MPU9150_TEMP_OUT_H, 2, temperatureData, "Failed to read temperature data"))
        return false;
    #endif

    #if MPU9150_FIFO_WITH_COMPASS == 0
    if (!m_settings->HALRead(m_slaveAddr, MPU9150_EXT_SENS_DATA_00, m_compassDataLength, compassData, "Failed to read compass data"))
        return false;
    #endif

#endif

    // FIFO contains data from register 59 up to register 96 in that order
    // (given sensor path was reset, otherwise the data order is not correct)
    // ACC (6), TEMP(2), GYRO(6), EXT SENS(8, up to 24))

    /*
    // Debug
    printf("FIFO: ");
    for (unsigned int i=0; i < m_fifoChunkLength; i++) {
        printf("%x, ", fifoData[i] ); }
    #if MPU9150_FIFO_WITH_COMPASS == 0
    printf("Compass: ");
    for (unsigned int i=0; i < m_compassDataLength; i++) {
        printf("%x, ", compassData[i] ); }
    #endif
    #if MPU9150_FIFO_WITH_TEMP == 0
        printf("Temperature: ");
        printf("%x, ", temperatureData[0] );
        printf("%x, ", temperatureData[1] );
   #endif
   printf("\n");
   */
           
   // Accelerometer
    RTMath::convertToVector(fifoData, m_imuData.accel, m_accelScale, true);
    
    #if MPU9150_FIFO_WITH_TEMP == 1
        // Temperature
        m_imuData.temperature =  (RTFLOAT) ((int16_t)( ((uint16_t)fifoData[6] << 8) | (uint16_t)fifoData[7] )) / 340.0f + 36.51f;  // combined registers and convert to temperature
        m_imuData.temperatureValid = true;
        // Gyroscope
        RTMath::convertToVector(fifoData + 8, m_imuData.gyro, m_gyroScale, true);
        // Compass
        #if MPU9150_FIFO_WITH_COMPASS == 1
            if (m_compassIs5883)
                RTMath::convertToVector(fifoData + 14, m_imuData.compass, 0.092f, true);
           else
                RTMath::convertToVector(fifoData + 14 + 1, m_imuData.compass, 0.3f, false);
        #else
            if (m_compassIs5883)
                RTMath::convertToVector(compassData, m_imuData.compass, 0.092f, true);
            else
                RTMath::convertToVector(compassData + 1, m_imuData.compass, 0.3f, false);
        #endif
    #else // no temperature in fifo
        // Temperature
        m_imuData.temperature =  (RTFLOAT) ((int16_t)( ((uint16_t)temperatureData[0] << 8 ) | (uint16_t)temperatureData[1] )) / 340.0f + 36.51f;  // combined registers and convert to temperature
        m_imuData.temperatureValid = true;
        // Gyroscope
        RTMath::convertToVector(fifoData + 6, m_imuData.gyro, m_gyroScale, true);
        //Compass
        #if MPU9150_FIFO_WITH_COMPASS == 1 // without temp but with compass in FIFO
            if (m_compassIs5883)
                RTMath::convertToVector(fifoData + 12, m_imuData.compass, 0.092f, true);
            else
                RTMath::convertToVector(fifoData + 12 + 1, m_imuData.compass, 0.3f, false);
        #else
            if (m_compassIs5883)
                RTMath::convertToVector(compassData, m_imuData.compass, 0.092f, true);
            else
                RTMath::convertToVector(compassData + 1, m_imuData.compass, 0.3f, false);
        #endif
    #endif

    //  sort out gyro axes

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());


    if (m_compassIs5883) {
        //  sort out compass axes

        float temp;

        temp = m_imuData.compass.y();
        m_imuData.compass.setY(-m_imuData.compass.z());
        m_imuData.compass.setZ(-temp);

    } else {

        //  use the compass fuse data adjustments

        m_imuData.compass.setX(m_imuData.compass.x() * m_compassAdjust[0]);
        m_imuData.compass.setY(m_imuData.compass.y() * m_compassAdjust[1]);
        m_imuData.compass.setZ(m_imuData.compass.z() * m_compassAdjust[2]);

        //  sort out compass axes

        float temp;

        temp = m_imuData.compass.x();
        m_imuData.compass.setX(m_imuData.compass.y());
        m_imuData.compass.setY(-temp);
    }
    
    if (m_firstTime)
        m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    else
        m_imuData.timestamp += m_sampleInterval;

    m_firstTime = false;

    //  now do standard processing
    if (m_imuData.temperatureValid == true) {
        // Check if temperature changed
        if (fabs(m_imuData.temperature - m_temperature_previous) >= TEMPERATURE_DELTA) {
            // If yes, update bias
            updateTempBias(m_imuData.temperature);
            m_temperature_previous = m_imuData.temperature;
        }
        // Then do
        handleTempBias(); 	// temperature Correction
    }
    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    //  now update the filter
    updateFusion();

    return true;
}
