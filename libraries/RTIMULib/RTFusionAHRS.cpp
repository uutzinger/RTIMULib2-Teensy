////////////////////////////////////////////////////////////////////////////
//
//  This file was added to RTIMULib and includes:
//  This routine does not correct for compass declination
//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author         Notes
// 29/09/2011   SOH Madgwick   Initial release
// 02/10/2011   SOH Madgwick   Optimised for reduced CPU load
// 19/02/2012   SOH Madgwick   Magnetometer measurement is normalised
//
//=====================================================================================================

#include "RTFusionAHRS.h"
#include "RTIMUSettings.h"

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
# define GyroMeasError M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
# define GyroMeasDrift M_PI * (1.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a trade-off in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError
// of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a 
// stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not 
// fast enough for a quadcopter or robot car! By increasing beta (GyroMeasError) 
// by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the
// I coefficient in a PID control sense;  the bigger the feedback coefficient,
// the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

RTFusionAHRS::RTFusionAHRS()
{
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    m_beta = beta;
    m_zeta = zeta;

    reset();

    m_enableAccel = true;
    m_enableCompass = true;
    m_enableGyro = true;
    
    if (m_debug) {
        HAL_INFO("\n------\n");
        HAL_INFO1("IMU beta %f\n", m_beta);
        HAL_INFO1("IMU zeta %f\n", m_zeta);
    }

}

RTFusionAHRS::~RTFusionAHRS()
{
}

void RTFusionAHRS::reset()
{
    m_firstTime = true;
    m_fusionPose = RTVector3();
    m_fusionQPose.fromEuler(m_fusionPose);
    m_gyro = RTVector3();
    m_accel = RTVector3();
    m_compass = RTVector3();
    m_measuredPose = RTVector3();
    m_measuredQPose.fromEuler(m_measuredPose);
}

void RTFusionAHRS::newIMUData(RTIMU_DATA& data, const RTIMUSettings *settings)
{

    if (m_debug) {
        HAL_INFO("\n------\n");
        HAL_INFO1("IMU update delta time: %f\n", m_timeDelta);
    }

    m_gyro = data.gyro;
    m_accel = data.accel;
    m_compass = data.compass;
    m_compassValid = data.compassValid;

    if (m_firstTime) {
        
        // adjust for compass declination, compute the correction data only at beginning
        m_sin_theta_half = sin(-settings->m_compassAdjDeclination/2.0f);
        m_cos_theta_half = cos(-settings->m_compassAdjDeclination/2.0f);
        
        m_lastFusionTime = data.timestamp;
        calculatePose(m_accel, m_compass, settings->m_compassAdjDeclination);

        //  initialize the poses

        m_stateQ.fromEuler(m_measuredPose);
        m_fusionQPose = m_stateQ;
        m_fusionPose = m_measuredPose;
        m_firstTime = false;

    } else { // not first time
        m_timeDelta = (RTFLOAT)(data.timestamp - m_lastFusionTime) / (RTFLOAT)1000000;
        m_lastFusionTime = data.timestamp;
        if (m_timeDelta <= 0)
            return;

        calculatePose(data.accel, data.compass, settings->m_compassAdjDeclination);
    
        // =================================================
        //    AHRS
        //
        // Previous Q Pose; short name local variables for readability
        float q1 = m_stateQ.scalar();
        float q2 = m_stateQ.x();
        float q3 = m_stateQ.y();
        float q4 = m_stateQ.z();   

        float ax, ay, az, mx, my, mz, gx, gy, gz;           // accelerometer, magnetometer, gyroscope
        		
        float gerrx, gerry, gerrz; // gyro bias error

        float norm;
        float hx, hy, _2bx, _2bz;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        float _2q1mx,_2q1my, _2q1mz,_2q2mx;
        float _4bx, _4bz;
        float _2q1, _2q2, _2q3, _2q4;
        float q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q4, q3q3, q4q4;  
        float _4q1, _4q2, _4q3, _8q2, _8q3;
        float _2q3q4, _2q1q3;

        if (m_enableCompass) {
          mx = m_compass.x();
          my = m_compass.y();
          mz = m_compass.z();
	  
        } else {
          mx = 0.0f;
          my = 0.0f;
          mz = 0.0f; 
		}   

        if (m_enableAccel) {
          ax = m_accel.x();
          ay = m_accel.y();
          az = m_accel.z();
        } else {
          ax = 0.0f;
          ay = 0.0f;
          az = 0.0f; }   

       if (m_enableGyro) {
          gx=m_gyro.x();
          gy=m_gyro.y();
          gz=m_gyro.z();
        } else { return; }   // We need to have valid gyroscope data

        ////////////////////////////////////////////////////////////////////////////
        // Regular Algorithm

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz); // s
        qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy); // x
        qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx); // y
        qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx); // z

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // Use this algorithm if accelerometer is valid 
            // If accelerometer is not valid, update pose based on previous qDot

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0) return; // handle NaN
            ax /= norm;
            ay /= norm;
            az /= norm;  

            // Auxiliary variables to avoid repeated arithmetic
             _2q1 = 2.0f * q1;
             _2q2 = 2.0f * q2;
             _2q3 = 2.0f * q3;
             _2q4 = 2.0f * q4;
             q1q1 = q1 * q1;
             q2q2 = q2 * q2;
             q3q3 = q3 * q3;
             q4q4 = q4 * q4;  

            if(((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
                // If magnetometer is invalid

                // Auxiliary variables to avoid repeated arithmetic
                _4q1 = 4.0f * q1;
                _4q2 = 4.0f * q2;
                _4q3 = 4.0f * q3;
                _8q2 = 8.0f * q2;
                _8q3 = 8.0f * q3;

                // Gradient decent algorithm corrective step
                s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
                s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
                s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
                s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;

             } else { 
                // Valid magnetometer available use this code

                // Normalise magnetometer measurement
                norm = sqrt(mx * mx + my * my + mz * mz);
                if (norm == 0.0) return; // handle NaN
                mx /= norm;
                my /= norm;
                mz /= norm;

                // Auxiliary variables to avoid repeated arithmetic
                q1q2 = q1 * q2;
                q1q3 = q1 * q3;
                q1q4 = q1 * q4;
                q2q3 = q2 * q3;
                q2q4 = q2 * q4;
                q3q4 = q3 * q4;
                _2q1q3 = 2.0f * q1q3;
                _2q3q4 = 2.0f * q3q4;
                _2q1mx = 2.0f * q1 * mx;
                _2q1my = 2.0f * q1 * my;
                _2q1mz = 2.0f * q1 * mz;
                _2q2mx = 2.0f * q2 * mx;

                // Reference direction of Earth's magnetic field
                 hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
                 hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
                _2bx = sqrt(hx * hx + hy * hy);
                _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
                _4bx = 2.0f * _2bx;
                _4bz = 2.0f * _2bz;  

                // Gradient decent algorithm corrective step
                s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s2 =  _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s4 =  _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

             } // end valid magnetometer

            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            if (norm == 0.0) return; // handle NaN
            s1 /= norm;
            s2 /= norm;
            s3 /= norm;
            s4 /= norm; 

            // Compute estimated gyroscope biases
            gerrx = _2q1 * s2 - _2q2 * s1 - _2q3 * s4 + _2q4 * s3;
            gerry = _2q1 * s3 + _2q2 * s4 - _2q3 * s1 - _2q4 * s2;
            gerrz = _2q1 * s4 - _2q2 * s3 + _2q3 * s2 - _2q4 * s1;

            // Compute and remove gyroscope biases
            m_gbiasx += gerrx * m_timeDelta * m_zeta;
            m_gbiasy += gerry * m_timeDelta * m_zeta;
            m_gbiasz += gerrz * m_timeDelta * m_zeta;

            gx -= m_gbiasx;
            gy -= m_gbiasy;
            gz -= m_gbiasz;

             // Apply feedback step
            qDot1 -= m_beta * s1;
            qDot2 -= m_beta * s2;
            qDot3 -= m_beta * s3;
            qDot4 -= m_beta * s4;

        } // end if valid accelerometer

        // Integrate to yield quaternion
        q1 += qDot1 * m_timeDelta;
        q2 += qDot2 * m_timeDelta;
        q3 += qDot3 * m_timeDelta;
        q4 += qDot4 * m_timeDelta;

        // normalise quaternion
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    
        if (norm == 0.0) return; // handle NaN
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
        q4 /= norm;
	
        m_stateQ.setScalar(q1);
        m_stateQ.setX(q2);
        m_stateQ.setY(q3);
        m_stateQ.setZ(q4);

        // Rotate Quaternion by Magnetic Declination
        // m_stateQ = q_declination * m_statqQ;

        /*
        SAGE
                N.<c,d,q1,q2,q3,q4,cos_theta_half, sin_theta_half> = QQ[]
                H.<i,j,k> = QuaternionAlgebra(c,d)
                q = q1 + q2 * i + q3 * j + q4 * k
                // here rotation is around gravity vector by theta
                mag_declination = cos_theta_half + sin_theta_half * (0 * i + 0 * j+ 1*k)
                q * mag_declination

                s : -q4*sin_theta_half + q1*cos_theta_half  
                x :  q3*sin_theta_half + q2*cos_theta_half 
                y : -q2*sin_theta_half + q3*cos_theta_half
                z :  q4*cos_theta_half + q1*sin_theta_half
                */

        m_stateQdec.setScalar(q1*m_cos_theta_half - q4*m_sin_theta_half);
        m_stateQdec.setX(q3*m_sin_theta_half + q2*m_cos_theta_half);
        m_stateQdec.setY(q3*m_cos_theta_half - q2*m_sin_theta_half);
        m_stateQdec.setZ(q4*m_cos_theta_half + q1*m_sin_theta_half);

    } // end not first time

	
    // =================================================

    if (m_enableCompass || m_enableAccel) {
            m_stateQError = m_measuredQPose - m_stateQ;
    } else {
            m_stateQError = RTQuaternion();
    }

    m_stateQdec.toEuler(m_fusionPose);
    m_fusionQPose = m_stateQdec;

    if (m_debug | settings->m_fusionDebug) {
        HAL_INFO(RTMath::displayRadians("Measured pose", m_measuredPose));
        HAL_INFO(RTMath::displayRadians("AHRS pose", m_fusionPose));
        HAL_INFO(RTMath::displayRadians("Measured quat", m_measuredPose));
        HAL_INFO(RTMath::display("AHRS quat", m_stateQ));
        HAL_INFO(RTMath::display("Error quat", m_stateQError));
        HAL_INFO3("AHRS Gyro Bias: %+f, %+f, %+f\n", m_gbiasx, m_gbiasy, m_gbiasz);
     }

    data.fusionPoseValid = true;
    data.fusionQPoseValid = true;
    data.fusionPose = m_fusionPose;
    data.fusionQPose = m_fusionQPose;
} //
