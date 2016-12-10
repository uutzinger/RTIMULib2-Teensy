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

// UU: This code was changed to
// provide temperature compensation to IMU sensor data
// update: recomputes to temperature offsets
// handle: applies to offsets to raw data
// 
// There is also attempt to better handle gyrio bias updates.
// When no motion is detected the gyro is updated. A low pass filter for
// initial bias estimate passes higher frequencies but after some learning
// it switches to a slower process so that the bias changes at slower pace.
// The gyro readings from the sensor are only used for updates 100ms after motion
// stopped and 100ms before it started again.

#include "RTIMU.h"
#include "RTFusionKalman4.h"
#include "RTFusionRTQF.h"
#include "RTFusionAHRS.h"
#include "RTIMUNull.h"
#include "RTIMUMPU9150.h"
#include "RTIMUMPU9250.h"
#include "RTIMUMPU9255.h"
#include "RTIMUGD20HM303D.h"
#include "RTIMUGD20M303DLHC.h"
#include "RTIMUGD20HM303DLHC.h"
#include "RTIMULSM9DS0.h"
#include "RTIMULSM9DS1.h"
#include "RTIMUBMX055.h"
#include "RTIMUBNO055.h"
#include "RTMotion.h"
#include "RunningAverage.h"

//  this sets the learning rate for compass and accelerometer running average calculation
// 0.2 original
#define COMPASS_ALPHA 0.1f

// this sets the learning rate for the acceleration length to become 1 g during nomotion
#define ACCEL_ALPHA 0.01f

//  this sets the min range (max - min) of values to trigger runtime mag calibration
#define RTIMU_RUNTIME_MAGCAL_RANGE  30

//  Axis rotation arrays
float RTIMU::m_axisRotation[RTIMU_AXIS_ROTATION_COUNT][9] = {
    {1, 0, 0, 0, 1, 0, 0, 0, 1},                    // RTIMU_XNORTH_YEAST
    {0, -1, 0, 1, 0, 0, 0, 0, 1},                   // RTIMU_XEAST_YSOUTH
    {-1, 0, 0, 0, -1, 0, 0, 0, 1},                  // RTIMU_XSOUTH_YWEST
    {0, 1, 0, -1, 0, 0, 0, 0, 1},                   // RTIMU_XWEST_YNORTH

    {1, 0, 0, 0, -1, 0, 0, 0, -1},                  // RTIMU_XNORTH_YWEST
    {0, 1, 0, 1, 0, 0, 0, 0, -1},                   // RTIMU_XEAST_YNORTH
    {-1, 0, 0, 0, 1, 0, 0, 0, -1},                  // RTIMU_XSOUTH_YEAST
    {0, -1, 0, -1, 0, 0, 0, 0, -1},                 // RTIMU_XWEST_YSOUTH

    {0, 1, 0, 0, 0, -1, -1, 0, 0},                  // RTIMU_XUP_YNORTH
    {0, 0, 1, 0, 1, 0, -1, 0, 0},                   // RTIMU_XUP_YEAST
    {0, -1, 0, 0, 0, 1, -1, 0, 0},                  // RTIMU_XUP_YSOUTH
    {0, 0, -1, 0, -1, 0, -1, 0, 0},                 // RTIMU_XUP_YWEST

    {0, 1, 0, 0, 0, 1, 1, 0, 0},                    // RTIMU_XDOWN_YNORTH
    {0, 0, -1, 0, 1, 0, 1, 0, 0},                   // RTIMU_XDOWN_YEAST
    {0, -1, 0, 0, 0, -1, 1, 0, 0},                  // RTIMU_XDOWN_YSOUTH
    {0, 0, 1, 0, -1, 0, 1, 0, 0},                   // RTIMU_XDOWN_YWEST

    {1, 0, 0, 0, 0, 1, 0, -1, 0},                   // RTIMU_XNORTH_YUP
    {0, 0, -1, 1, 0, 0, 0, -1, 0},                  // RTIMU_XEAST_YUP
    {-1, 0, 0, 0, 0, -1, 0, -1, 0},                 // RTIMU_XSOUTH_YUP
    {0, 0, 1, -1, 0, 0, 0, -1, 0},                  // RTIMU_XWEST_YUP

    {1, 0, 0, 0, 0, -1, 0, 1, 0},                   // RTIMU_XNORTH_YDOWN
    {0, 0, 1, 1, 0, 0, 0, 1, 0},                    // RTIMU_XEAST_YDOWN
    {-1, 0, 0, 0, 0, 1, 0, 1, 0},                   // RTIMU_XSOUTH_YDOWN
    {0, 0, -1, -1, 0, 0, 0, 1, 0}                   // RTIMU_XWEST_YDOWN
};

RTIMU *RTIMU::createIMU(RTIMUSettings *settings)
{
    switch (settings->m_imuType) {
    case RTIMU_TYPE_MPU9150:
        return new RTIMUMPU9150(settings);

    case RTIMU_TYPE_GD20HM303D:
        return new RTIMUGD20HM303D(settings);

    case RTIMU_TYPE_GD20M303DLHC:
        return new RTIMUGD20M303DLHC(settings);

    case RTIMU_TYPE_LSM9DS0:
        return new RTIMULSM9DS0(settings);
		
    case RTIMU_TYPE_LSM9DS1:
        return new RTIMULSM9DS1(settings);

    case RTIMU_TYPE_MPU9250:
        return new RTIMUMPU9250(settings);

    case RTIMU_TYPE_MPU9255:
        return new RTIMUMPU9255(settings);
		
    case RTIMU_TYPE_GD20HM303DLHC:
        return new RTIMUGD20HM303DLHC(settings);

    case RTIMU_TYPE_BMX055:
        return new RTIMUBMX055(settings);

    case RTIMU_TYPE_BNO055:
        return new RTIMUBNO055(settings);

    case RTIMU_TYPE_AUTODISCOVER:
        if (settings->discoverIMU(settings->m_imuType, settings->m_busIsI2C, settings->m_I2CSlaveAddress)) {
            settings->saveSettings();
            return RTIMU::createIMU(settings);
        }
        return new RTIMUNull(settings);

    case RTIMU_TYPE_NULL:
        return new RTIMUNull(settings);

    default:
        return NULL;
    }
}

RTIMU::RTIMU(RTIMUSettings *settings)
{
    m_settings = settings;
    m_compassCalibrationMode = false;
    m_accelCalibrationMode = false;
    m_runtimeMagCalValid = false;
 
    m_runtimeMagCalMax = -1000.0f;
    m_runtimeMagCalMin =  1000.0f;
    //m_runtimeMagCalMax.setX(-1000);
    //m_runtimeMagCalMax.setY(-1000);
    //m_runtimeMagCalMax.setZ(-1000);
    //m_runtimeMagCalMin.setX( 1000);
    //m_runtimeMagCalMin.setY( 1000);
    //m_runtimeMagCalMin.setZ( 1000);

	m_gyroCalibrationMode = false;
    m_temperatureCalibrationMode = false;

	m_gyroRunTimeCalibrationEnable = true;
	m_gyroManualCalibrationEnable = false;
	m_accelRunTimeCalibrationEnable = false;
	m_compassRunTimeCalibrationEnable = false;
	
    switch (m_settings->m_fusionType) {
    case RTFUSION_TYPE_KALMANSTATE4:
        m_fusion = new RTFusionKalman4();
        break;

    case RTFUSION_TYPE_RTQF:
        m_fusion = new RTFusionRTQF();
        break;

    case RTFUSION_TYPE_AHRS:
        m_fusion = new RTFusionAHRS();
        break;
        
    default:
        m_fusion = new RTFusion();
        break;
    }
    HAL_INFO1("Using fusion algorithm %s\n", RTFusion::fusionName(m_settings->m_fusionType));
	
	m_compassAverageX = new RunningAverage(20); // 0.1f * m_sampleRate;
	m_compassAverageY = new RunningAverage(20);
	m_compassAverageZ = new RunningAverage(20);
}

RTIMU::~RTIMU()
{
    delete m_fusion;
    delete m_compassAverageX;
    delete m_compassAverageY;
    delete m_compassAverageZ;
    m_fusion = NULL;
	m_compassAverageX = NULL;
	m_compassAverageY = NULL;
	m_compassAverageZ = NULL;
}

	
void RTIMU::setCalibrationData()
{
    float maxDelta = -1;
    float delta;

    if (m_settings->m_compassCalValid) {
        //  find biggest range

        for (int i = 0; i < 3; i++) {
            if ((m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) > maxDelta)
                maxDelta = m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i);
        }
        if (maxDelta < 0) {
            HAL_ERROR("Error in compass calibration data\n");
            return;
        }
        maxDelta /= 2.0f;                                       // this is the max +/- range

        for (int i = 0; i < 3; i++) {
            delta = (m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) / 2.0f;
            m_compassCalScale[i] = maxDelta / delta;            // makes everything the same range
            m_compassCalOffset[i] = (m_settings->m_compassCalMax.data(i) + m_settings->m_compassCalMin.data(i)) / 2.0f;
        }
    }

    if (m_settings->m_temperatureCalValid) {
        HAL_INFO("Using temperature bias calibration\n");
    } else {
        HAL_INFO("Temperature bias calibration not in use\n");
    }

    if (m_settings->m_compassCalValid) {
        HAL_INFO("Using min/max compass calibration\n");
    } else {
        HAL_INFO("min/max compass calibration not in use\n");
    }

    if (m_settings->m_compassCalEllipsoidValid) {
        HAL_INFO("Using ellipsoid compass calibration\n");
    } else {
        HAL_INFO("Ellipsoid compass calibration not in use\n");
    }

    if (m_settings->m_accelCalValid) {
        HAL_INFO("Using accel calibration\n");
    } else {
        HAL_INFO("Accel calibration not in use\n");
    }

    if (m_settings->m_accelCalEllipsoidValid) {
        HAL_INFO("Using ellipsoid accelerometer calibration\n");
    } else {
        HAL_INFO("Ellipsoid accelerometer calibration not in use\n");
    }
}

void RTIMU::updateTempBias(float senTemp)
{
    if(m_settings->m_temperatureCalValid == true) {
	if(senTemp < m_settings->m_senTemp_break) {
            for(int i = 0; i < 9; i++) { 
                m_settings->m_temperaturebias[i] = m_settings->m_c3[i]*(senTemp*senTemp*senTemp) + m_settings->m_c2[i]*(senTemp*senTemp) + m_settings->m_c1[i]*senTemp + m_settings->m_c0[i];
            }		
	} else {
            for(int i = 0; i < 9; i++) { 
                m_settings->m_temperaturebias[i] = 0.0f;
            }
	}
    }
}

void RTIMU::handleTempBias()
{
    if(getTemperatureCalibrationValid()) {
        // Accelerometer
        m_imuData.accel.setX(m_imuData.accel.x() - m_settings->m_temperaturebias[0]);
        m_imuData.accel.setY(m_imuData.accel.y() - m_settings->m_temperaturebias[1]);
        m_imuData.accel.setZ(m_imuData.accel.z() - m_settings->m_temperaturebias[2]);
        // Gyroscope
        m_imuData.gyro.setX(m_imuData.gyro.x() - m_settings->m_temperaturebias[3]);
        m_imuData.gyro.setY(m_imuData.gyro.y() - m_settings->m_temperaturebias[4]);
        m_imuData.gyro.setZ(m_imuData.gyro.z() - m_settings->m_temperaturebias[5]);
        // Compass
        m_imuData.compass.setX(m_imuData.compass.x() - m_settings->m_temperaturebias[6]);
        m_imuData.compass.setY(m_imuData.compass.y() - m_settings->m_temperaturebias[7]);
        m_imuData.compass.setZ(m_imuData.compass.z() - m_settings->m_temperaturebias[8]);
    }
}

bool RTIMU::setGyroContinuousLearningAlpha(RTFLOAT alpha)
{
    if ((alpha < 0.0) || (alpha >= 1.0))
        return false;

    m_gyroContinuousAlpha = alpha;
    return true;
}


void RTIMU::gyroBiasInit()
{
    m_gyroLearningAlpha = 2.0f / m_sampleRate;
    m_gyroContinuousAlpha = 0.02f / m_sampleRate;
	m_imuData.motion = true; // ensure that when system starts without motion the gyro bias learning is triggerd also
	m_EEPROMCount = 0;
	m_intervalCount = 0;
	m_previousMotion = false;
}

//  Note - code assumes that this is the first thing called after axis swapping
//  for each specific IMU chip has occurred.

void RTIMU::handleGyroBias()
{
    // do axis rotation

    if ((m_settings->m_axisRotation > 0) && (m_settings->m_axisRotation < RTIMU_AXIS_ROTATION_COUNT)) {
        // need to do an axis rotation
        float *matrix = m_axisRotation[m_settings->m_axisRotation];
        RTIMU_DATA tempIMU = m_imuData;

        // do new x value
        if (matrix[0] != 0) {
            m_imuData.gyro.setX(tempIMU.gyro.x() * matrix[0]);
            m_imuData.accel.setX(tempIMU.accel.x() * matrix[0]);
            m_imuData.compass.setX(tempIMU.compass.x() * matrix[0]);
        } else if (matrix[1] != 0) {
            m_imuData.gyro.setX(tempIMU.gyro.y() * matrix[1]);
            m_imuData.accel.setX(tempIMU.accel.y() * matrix[1]);
            m_imuData.compass.setX(tempIMU.compass.y() * matrix[1]);
        } else if (matrix[2] != 0) {
            m_imuData.gyro.setX(tempIMU.gyro.z() * matrix[2]);
            m_imuData.accel.setX(tempIMU.accel.z() * matrix[2]);
            m_imuData.compass.setX(tempIMU.compass.z() * matrix[2]);
        }

        // do new y value
        if (matrix[3] != 0) {
            m_imuData.gyro.setY(tempIMU.gyro.x() * matrix[3]);
            m_imuData.accel.setY(tempIMU.accel.x() * matrix[3]);
            m_imuData.compass.setY(tempIMU.compass.x() * matrix[3]);
        } else if (matrix[4] != 0) {
            m_imuData.gyro.setY(tempIMU.gyro.y() * matrix[4]);
            m_imuData.accel.setY(tempIMU.accel.y() * matrix[4]);
            m_imuData.compass.setY(tempIMU.compass.y() * matrix[4]);
        } else if (matrix[5] != 0) {
            m_imuData.gyro.setY(tempIMU.gyro.z() * matrix[5]);
            m_imuData.accel.setY(tempIMU.accel.z() * matrix[5]);
            m_imuData.compass.setY(tempIMU.compass.z() * matrix[5]);
        }

        // do new z value
        if (matrix[6] != 0) {
            m_imuData.gyro.setZ(tempIMU.gyro.x() * matrix[6]);
            m_imuData.accel.setZ(tempIMU.accel.x() * matrix[6]);
            m_imuData.compass.setZ(tempIMU.compass.x() * matrix[6]);
        } else if (matrix[7] != 0) {
            m_imuData.gyro.setZ(tempIMU.gyro.y() * matrix[7]);
            m_imuData.accel.setZ(tempIMU.accel.y() * matrix[7]);
            m_imuData.compass.setZ(tempIMU.compass.y() * matrix[7]);
        } else if (matrix[8] != 0) {
            m_imuData.gyro.setZ(tempIMU.gyro.z() * matrix[8]);
            m_imuData.accel.setZ(tempIMU.accel.z() * matrix[8]);
            m_imuData.compass.setZ(tempIMU.compass.z() * matrix[8]);
        }
    } // end axis rotations

    // Motion Detection
	// ----------------
    RTVector3 deltaAccel = m_previousAccel;
    deltaAccel -= m_imuData.accel;          // compute accel variations
    m_previousAccel = m_imuData.accel;

    RTVector3 deltaGyro = m_previousGyro;
    deltaGyro -= m_imuData.gyro;          // compute accel variations
    m_previousGyro = m_imuData.gyro;
	
    // Serial.printf("Delta Accel: %f, Gyration: %f, Delta Gyration: %f\n", deltaAccel.length(), m_imuData.gyro.length(), deltaGyro.length());
	m_previousMotion = m_imuData.motion;  // to keep track of motion transitions
	
	// is the IMU moving?
    if ((deltaAccel.length() < RTIMU_FUZZY_ACCEL_ZERO) && (deltaGyro.length() < RTIMU_FUZZY_DELTA_GYRO_ZERO) && (m_imuData.gyro.length() < RTIMU_FUZZY_GYRO_ZERO)) {
		m_imuData.motion = false;
    } else {
		m_imuData.motion = true;
	}
    // if (m_imuData.motion) { Serial.println("Sensor is moving."); } else { Serial.println("Sensor is still."); } 
    
	// GyroBias
	// The goal is to discard the first 0.1 sec of no motion and the last 0.1 sec of no motion
	// from bias updates
	// bias update is passed through low pass filter
    //
    // Start of still phase?
	//   Is this the start of a still phase?
	//   Then prepare for potential bias updates
	if ((m_previousMotion == true) && (m_imuData.motion==false)) { 
		// initialize potential bias update
		m_intervalCount=0; //
		// initialize the temporary bias with current bias
		m_gyroBiasTemp = m_settings->m_gyroBias;
		// if system was not still for more than 0.1 seconds, it will use initial bias as candidate 
		m_gyroBiasCandidate = m_settings->m_gyroBias;
	    // Serial.println("IMU transitioned from motion to still");
		m_noMotionStarted = true;
	}
	
	// GyroBias
    if ( m_gyroRunTimeCalibrationEnable ) {
		if (!m_imuData.motion) { // Update Gyro Bias if there is no motion
			m_intervalCount++; 
			// if device was still for 0.1 seconds 
			//    update current bias with candidate
			//    update candidate bias with temporary bias
			//    when noMotionStarted do not update bias for first interval
			//    when IMU moves again, the candidate will not be applied, discarding the last interval
			if (m_intervalCount >= 0.1 * m_sampleRate) {
				m_intervalCount = 0;
				m_settings->m_gyroBias = m_gyroBiasCandidate; 	// activate candidate bias
				m_gyroBiasCandidate = m_gyroBiasTemp; 			// update candidate
				m_noMotionStarted = false;                      // first interval passed
				// Serial.println("Gyro bias and candidate updated");
			}
			if (m_noMotionStarted == false) {
				// update temporary bias
				RTVector3 gyroTemp = m_imuData.gyro - m_gyroBiasTemp;
				// Serial.printf("%s: ", "Gyro"); Serial.printf(", x:%+4.5f", gyroTemp.x()); Serial.printf(", y:%+4.5f", gyroTemp.y()); Serial.printf(", z:%+4.5f", gyroTemp.z()); Serial.printf(", s:%+4.5f\n", gyroTemp.length()); 
				// Serial.printf("%s: ", "Gyro Bias"); Serial.printf(", x:%+4.5f", m_settings->m_gyroBias.x()); Serial.printf(", y:%+4.5f", m_settings->m_gyroBias.y()); Serial.printf(", z:%+4.5f", m_settings->m_gyroBias.z()); Serial.printf(", s:%+4.5f\n", m_settings->m_gyroBias.length());
				if (gyroTemp.length() > RTIMU_FUZZY_GYRO_BIAS) { // Aggressive Bias Update
					m_gyroBiasTemp.setX( (1.0 - m_gyroLearningAlpha) * m_gyroBiasTemp.x() + m_gyroLearningAlpha * m_imuData.gyro.x() );
					m_gyroBiasTemp.setY( (1.0 - m_gyroLearningAlpha) * m_gyroBiasTemp.y() + m_gyroLearningAlpha * m_imuData.gyro.y() );
					m_gyroBiasTemp.setZ( (1.0 - m_gyroLearningAlpha) * m_gyroBiasTemp.z() + m_gyroLearningAlpha * m_imuData.gyro.z() );
					// Serial.println("Gyro learning fast");
				} else { // Slow Bias Update
					m_gyroBiasTemp.setX( (1.0 - m_gyroContinuousAlpha) * m_gyroBiasTemp.x() + m_gyroContinuousAlpha * m_imuData.gyro.x() );
					m_gyroBiasTemp.setY( (1.0 - m_gyroContinuousAlpha) * m_gyroBiasTemp.y() + m_gyroContinuousAlpha * m_imuData.gyro.y() );
					m_gyroBiasTemp.setZ( (1.0 - m_gyroContinuousAlpha) * m_gyroBiasTemp.z() + m_gyroContinuousAlpha * m_imuData.gyro.z() );
					// Serial.println("Gyro learning slow");
				}
			} // end noMotion has been going on for more than interval
		} //  no motion

		// store new bias every 60 seconds in EEPROM
		m_EEPROMCount++;
		if (m_EEPROMCount >= (60 * m_sampleRate)) {
			m_EEPROMCount = 0;
			m_settings->m_gyroBiasValid = true;
			m_settings->saveSettings();
			// Serial.println("Gyro bias saved in EEPROM");
		} 
	} // runtimeCalibrationEnable

    if ( m_gyroManualCalibrationEnable ) {
		m_settings->m_gyroBias = m_settings->m_gyroBias * (1.0 - m_gyroLearningAlpha) + m_imuData.gyro * m_gyroLearningAlpha;
	}

	// Apply Gyro Bias
	if (getGyroCalibrationValid()) {
		m_imuData.gyro -= m_settings->m_gyroBias;
	}
}

void RTIMU::calibrateAverageCompass()
{
    //  see if need to do runtime mag calibration (i.e. no stored calibration data)

    if ((!m_compassCalibrationMode && !m_settings->m_compassCalValid) || (m_compassRunTimeCalibrationEnable)) {
        // try runtime calibration
        bool changed = false;

        // see if there is a new max or min

        if (m_runtimeMagCalMax.x() < m_imuData.compass.x()) {
            m_runtimeMagCalMax.setX( m_imuData.compass.x());
            changed = true;
        }
        if (m_runtimeMagCalMax.y() < m_imuData.compass.y()) {
            m_runtimeMagCalMax.setY( m_imuData.compass.y());
            changed = true;
        }
        if (m_runtimeMagCalMax.z() < m_imuData.compass.z()) {
            m_runtimeMagCalMax.setZ( m_imuData.compass.z());
            changed = true;
        }

        if (m_runtimeMagCalMin.x() > m_imuData.compass.x()) {
            m_runtimeMagCalMin.setX( m_imuData.compass.x());
            changed = true;
        }
        if (m_runtimeMagCalMin.y() > m_imuData.compass.y()) {
            m_runtimeMagCalMin.setY( m_imuData.compass.y());
            changed = true;
        }
        if (m_runtimeMagCalMin.z() > m_imuData.compass.z()) {
            m_runtimeMagCalMin.setZ( m_imuData.compass.z());
            changed = true;
        }

        //  now see if ranges are sufficient

        if (changed) {

            float delta;

            if (!m_runtimeMagCalValid) {
                m_runtimeMagCalValid = true;

                for (int i = 0; i < 3; i++)
                {
                    delta = m_runtimeMagCalMax.data(i) - m_runtimeMagCalMin.data(i);
                    if ((delta < RTIMU_RUNTIME_MAGCAL_RANGE) || (m_runtimeMagCalMin.data(i) > 0) || (m_runtimeMagCalMax.data(i) < 0))
                    {
                        m_runtimeMagCalValid = false;
                        break;
                    }
                }
            }

            //  find biggest range and scale to that

            if (m_runtimeMagCalValid) {
                float magMaxDelta = -1;

                for (int i = 0; i < 3; i++) {
                    if ((m_runtimeMagCalMax.data(i) - m_runtimeMagCalMin.data(i)) > magMaxDelta)
                    {
                        magMaxDelta = m_runtimeMagCalMax.data(i) - m_runtimeMagCalMin.data(i);
                    }
                }

                // adjust for + and - range

                magMaxDelta /= 2.0;

                for (int i = 0; i < 3; i++)
                {
                    delta = (m_runtimeMagCalMax.data(i) - m_runtimeMagCalMin.data(i)) / 2.0;
                    m_compassCalScale[i] = magMaxDelta / delta;
                    m_compassCalOffset[i] = (m_runtimeMagCalMax.data(i) + m_runtimeMagCalMin.data(i)) / 2.0;
                }
				
				m_settings->m_compassCalMax = m_runtimeMagCalMax;
				m_settings->m_compassCalMin = m_runtimeMagCalMin;
				
            }
        }
    }
    //  calibrate if required

  if (getCompassCalibrationValid() || getRuntimeCompassCalibrationValid()) {
        m_imuData.compass.setX((m_imuData.compass.x() - m_compassCalOffset[0]) * m_compassCalScale[0]);
        m_imuData.compass.setY((m_imuData.compass.y() - m_compassCalOffset[1]) * m_compassCalScale[1]);
        m_imuData.compass.setZ((m_imuData.compass.z() - m_compassCalOffset[2]) * m_compassCalScale[2]);

        if (m_settings->m_compassCalEllipsoidValid) {
            RTVector3 ev = m_imuData.compass;
            ev -= m_settings->m_compassCalEllipsoidOffset;

            m_imuData.compass.setX(ev.x() * m_settings->m_compassCalEllipsoidCorr[0][0] +
                ev.y() * m_settings->m_compassCalEllipsoidCorr[0][1] +
                ev.z() * m_settings->m_compassCalEllipsoidCorr[0][2]);

            m_imuData.compass.setY(ev.x() * m_settings->m_compassCalEllipsoidCorr[1][0] +
                ev.y() * m_settings->m_compassCalEllipsoidCorr[1][1] +
                ev.z() * m_settings->m_compassCalEllipsoidCorr[1][2]);

            m_imuData.compass.setZ(ev.x() * m_settings->m_compassCalEllipsoidCorr[2][0] +
                ev.y() * m_settings->m_compassCalEllipsoidCorr[2][1] +
                ev.z() * m_settings->m_compassCalEllipsoidCorr[2][2]);
        }
    }

    //  update running average

	m_compassAverageX->addValue(m_imuData.compass.x());
	m_compassAverageY->addValue(m_imuData.compass.y());
	m_compassAverageZ->addValue(m_imuData.compass.z());
	
	//Serial.println(m_compassAverageX->getAverage() - m_imuData.compass.x());
	//Serial.println(m_compassAverageY->getAverage() - m_imuData.compass.y());
	//Serial.println(m_compassAverageZ->getAverage() - m_imuData.compass.z());

	m_imuData.compass.setX(m_compassAverageX->getAverage());
	m_imuData.compass.setY(m_compassAverageY->getAverage());
	m_imuData.compass.setZ(m_compassAverageZ->getAverage());
	
    //m_compassAverage.setX(m_imuData.compass.x() * COMPASS_ALPHA + m_compassAverage.x() * (1.0 - COMPASS_ALPHA));
    //m_compassAverage.setY(m_imuData.compass.y() * COMPASS_ALPHA + m_compassAverage.y() * (1.0 - COMPASS_ALPHA));
    //m_compassAverage.setZ(m_imuData.compass.z() * COMPASS_ALPHA + m_compassAverage.z() * (1.0 - COMPASS_ALPHA));
    //
    //m_imuData.compass = m_compassAverage;
	
}

void RTIMU::resetCompassRunTimeMaxMin()
{
    m_runtimeMagCalMax = -1000.0f;
    m_runtimeMagCalMin =  1000.0f;

    //m_runtimeMagCalMax.setX(-1000.0);
    //m_runtimeMagCalMax.setY(-1000.0);
    //m_runtimeMagCalMax.setZ(-1000.0);
    //m_runtimeMagCalMin.setX( 1000.0);
    //m_runtimeMagCalMin.setY( 1000.0);
    //m_runtimeMagCalMin.setZ( 1000.0);
	//m_runtimeMagCalValid = false;
}

void RTIMU::calibrateAccel()
{

    if (getAccelCalibrationValid() ) {

		// printf("%s", RTMath::displayRadians("Accel 1)", m_imuData.accel));

		if (m_imuData.accel.x() >= 0)
			m_imuData.accel.setX(m_imuData.accel.x() / m_settings->m_accelCalMax.x());
		else
			m_imuData.accel.setX(m_imuData.accel.x() / -m_settings->m_accelCalMin.x());

		if (m_imuData.accel.y() >= 0)
			m_imuData.accel.setY(m_imuData.accel.y() / m_settings->m_accelCalMax.y());
		else
			m_imuData.accel.setY(m_imuData.accel.y() / -m_settings->m_accelCalMin.y());

		if (m_imuData.accel.z() >= 0)
			m_imuData.accel.setZ(m_imuData.accel.z() / m_settings->m_accelCalMax.z());
		else
			m_imuData.accel.setZ(m_imuData.accel.z() / -m_settings->m_accelCalMin.z());

		// printf("%s", RTMath::displayRadians("Accel 2)", m_imuData.accel));

		if (m_settings->m_accelCalEllipsoidValid) {
			RTVector3 ev = m_imuData.accel;
			ev -= m_settings->m_accelCalEllipsoidOffset;

			m_imuData.accel.setX(ev.x() * m_settings->m_accelCalEllipsoidCorr[0][0] +
				ev.y() * m_settings->m_accelCalEllipsoidCorr[0][1] +
				ev.z() * m_settings->m_accelCalEllipsoidCorr[0][2]);

			m_imuData.accel.setY(ev.x() * m_settings->m_accelCalEllipsoidCorr[1][0] +
				ev.y() * m_settings->m_accelCalEllipsoidCorr[1][1] +
				ev.z() * m_settings->m_accelCalEllipsoidCorr[1][2]);

			m_imuData.accel.setZ(ev.x() * m_settings->m_accelCalEllipsoidCorr[2][0] +
				ev.y() * m_settings->m_accelCalEllipsoidCorr[2][1] +
				ev.z() * m_settings->m_accelCalEllipsoidCorr[2][2]);
		}
		// printf("%s", RTMath::displayRadians("Accel 3)", m_imuData.accel));
	}
}

// UU automatic Accel Max/Min calibration/adjustment
// When there is no motion the sensor should experience 1g
 // Adjust Max/Min weighted by the magnitude of the acceleration in x/y/z
// so that the Max/Min is adjusted most where acceleration is largest
// Pass adjustment through low pass filter

void RTIMU::runtimeAdjustAccelCal()
{
    // printf("%s\n", m_imuData.motion ? "IMU is moving\n" : "IMU is still \n");  

    if (!m_imuData.motion) {
        
        RTFLOAT l = m_imuData.accel.length();  // This should be 1 g
        RTFLOAT c = (1.0 / l) - (1.0 / l / l); // adjust calibration values (empirically)
        // printf("AccelLength: %f correction: %f\n", l ,c);

        // printf("%s", RTMath::displayRadians("AccelMax", m_settings->m_accelCalMax));
        // printf("%s", RTMath::displayRadians("AccelMin", m_settings->m_accelCalMin));
         
        if (m_imuData.accel.x() >= 0)
            m_settings->m_accelCalMax.setX( (1.0 - ACCEL_ALPHA) * m_settings->m_accelCalMax.x() + ACCEL_ALPHA * ( m_settings->m_accelCalMax.x() * (1.0 + (m_imuData.accel.x() * c)) ));
        else
            m_settings->m_accelCalMin.setX( (1.0 - ACCEL_ALPHA) * m_settings->m_accelCalMin.x() + ACCEL_ALPHA * ( m_settings->m_accelCalMin.x() * (1.0 - (m_imuData.accel.x() * c)) ));

        if (m_imuData.accel.y() >= 0)
            m_settings->m_accelCalMax.setY( (1.0 - ACCEL_ALPHA) * m_settings->m_accelCalMax.y() + ACCEL_ALPHA * ( m_settings->m_accelCalMax.y() * (1.0 + (m_imuData.accel.y() * c)) ));
        else
            m_settings->m_accelCalMin.setY( (1.0 - ACCEL_ALPHA) * m_settings->m_accelCalMin.y() + ACCEL_ALPHA * ( m_settings->m_accelCalMin.y() * (1.0 - (m_imuData.accel.y() * c)) ));

        if (m_imuData.accel.z() >= 0)
            m_settings->m_accelCalMax.setZ( (1.0 - ACCEL_ALPHA) * m_settings->m_accelCalMax.z() + ACCEL_ALPHA * ( m_settings->m_accelCalMax.z() * (1.0 + (m_imuData.accel.z() * c)) ));
        else
            m_settings->m_accelCalMin.setZ( (1.0 - ACCEL_ALPHA) * m_settings->m_accelCalMin.z() + ACCEL_ALPHA * ( m_settings->m_accelCalMin.z() * (1.0 - (m_imuData.accel.z() * c)) ));

        //printf("%s", RTMath::displayRadians("AccelMax", m_settings->m_accelCalMax));
        //printf("%s", RTMath::displayRadians("AccelMin", m_settings->m_accelCalMin));
        
    }
}
void RTIMU::updateFusion()
{
    m_fusion->newIMUData(m_imuData, m_settings);
}

bool RTIMU::IMUGyroBiasValid()
{
    return m_settings->m_gyroBiasValid;
}

void RTIMU::setExtIMUData(RTFLOAT gx, RTFLOAT gy, RTFLOAT gz, RTFLOAT ax, RTFLOAT ay, RTFLOAT az,
        RTFLOAT mx, RTFLOAT my, RTFLOAT mz, uint64_t timestamp)
 {
     m_imuData.gyro.setX(gx);
     m_imuData.gyro.setY(gy);
     m_imuData.gyro.setZ(gz);
     m_imuData.accel.setX(ax);
     m_imuData.accel.setY(ay);
     m_imuData.accel.setZ(az);
     m_imuData.compass.setX(mx);
     m_imuData.compass.setY(my);
     m_imuData.compass.setZ(mz);
     m_imuData.timestamp = timestamp;
     updateFusion();
}
