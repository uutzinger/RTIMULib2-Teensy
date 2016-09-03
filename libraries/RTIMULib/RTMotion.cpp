#include "RTMotion.h"

RTMotion::RTMotion(RTIMUSettings *settings)
{
    m_settings = settings;
    m_accnorm_avg = new RunningAverage(ACCEL_AVG_HISTORY);
    m_accnorm_var = new RunningAverage(ACCEL_VAR_HISTORY);
    m_heading_X_avg = new RunningAverage(HEADING_AVG_HISTORY);
    m_heading_Y_avg = new RunningAverage(HEADING_AVG_HISTORY);
}

RTMotion::~RTMotion()
{

}

void RTMotion::motionInit()
{
    m_timestamp_previous = RTMath::currentUSecsSinceEpoch();
    m_residualsBias.zero();
    m_worldResiduals.zero();
    m_worldResiduals_previous.zero();
    m_worldResiduals.zero();
    m_worldResiduals_previous.zero();
    m_worldVelocity.zero();
    m_worldVelocity_drift.zero();
    m_worldVelocity_previous.zero();
    m_worldPosition.zero();
    m_motion = false;
    m_motion_previous = false;
    
    m_MotionData.worldAcceleration.zero();
    m_MotionData.worldVelocity.zero();
    m_MotionData.worldPosition.zero();
    m_MotionData.motion = false;
}

void RTMotion::motionReset()
{
    m_residualsBias.zero();
    m_worldVelocity.zero();
    m_worldPosition.zero();
    m_worldVelocity_previous.zero();
    m_worldVelocity_drift.zero();
    m_motion = false;
    
    m_MotionData.worldVelocity.zero();
    m_MotionData.worldPosition.zero();
    m_MotionData.motion = false;
}

void RTMotion::motionResetPosition()
{
    m_worldPosition.zero();
    m_MotionData.worldPosition.zero();
}

RTFLOAT RTMotion::updateAverageHeading(RTFLOAT& heading) 
{
    // this needs two component because of 0 - 360 jump at North 
    m_heading_X_avg->addValue(cos(heading));
    m_heading_Y_avg->addValue(sin(heading));
    float t = atan2(m_heading_Y_avg->getAverage(),m_heading_X_avg->getAverage());
    return t > 0 ? t : 2 * RTMATH_PI + t;
}

bool RTMotion::detectMotion(RTVector3& acc, RTVector3& gyr) {
// Three Stage Motion Detection
// Original Code is from FreeIMU Processing example
// Some modifications and tuning
//
// 1. Change in Acceleration Variance
// 2. Gyro activity
// Any of the two triggers motion
//
  
  bool accnorm_var_test, omega_test;
  
  // ACCELEROMETER
  ////////////////

  // Test for sudden accelerometer changes
  RTVector3 deltaAccel = m_previousAccel;
  deltaAccel -= acc;   // compute difference
  m_previousAccel = acc;
  // printf("Delta Acc: %4.3f ",  deltaAccel.length());
  if (deltaAccel.length() < RTIMU_FUZZY_ACCEL_ZERO) { accnorm_var_test = false; } else { accnorm_var_test = true; }

  // GYRO
  ////////////////
  // printf("Gyration: %4.3f ",  gyr.length());
  if (gyr.length() < RTIMU_FUZZY_GYRO_ZERO) { omega_test = false; } else { omega_test = true; }
  //printf("Test: %d %d\n",  accnorm_var_test, omega_test );
  
  // Combine acceleration test, acceleration deviation test and gyro test
  ///////////////////////////////////////////////////////////////////////
  if (omega_test || accnorm_var_test) { 
    return true; 
  } else { 
    return false;
  }
 
}
	
void RTMotion::updateVelocityPosition(RTVector3& residuals, RTQuaternion& q, float accScale, uint64_t& timestamp, bool& motion)
{
// Input:
//  Acceleration Residuals
//  Quaternion
//  Acceleration Scale, usually 9.81 m/s/s
//  Motion status
// Output:
//  Acceleration in world coordinate system
//  Velocity in world coordinate system
//  Position in world coordinate system

    float dt;
    bool motion_ended = false;
    RTVector3 residuals_cal;
    
    // Integration Time Step
    ////////////////////////
    dt = ((float)(timestamp - m_timestamp_previous)) * 0.000001f; // in seconds
    m_timestamp_previous = timestamp;

    // Check on Motion
    ///////////////////
    //  Did it start?
    if (m_motion_previous == false && motion == true) {
            m_motionStart_time = timestamp; }
    //  Did it end?
    if (m_motion_previous == true && motion == false) {
            m_dtmotion = ((float)(timestamp - m_motionStart_time))* 0.000001f; // motion time is in seconds
            motion_ended = true;
    } else {
            motion_ended = false; 
    }
    // Keep track of previous status
    m_motion_previous = motion;

    // printf(" %4.4f, ", m_worldVelocity_drift.x());
    // printf(" %4.4f, ", m_worldVelocity_drift.y());
    // printf(" %4.4f, ", m_worldVelocity_drift.z());
    // printf(" time: %f, %d\n", m_dtmotion, motion_ended); 

    // operate in the world coordinate system and spatial units
    residuals_cal = (residuals-m_residualsBias) * accScale; // scale from g to real spatial units
    m_worldResiduals = RTMath::toWorld(residuals_cal, q); // rotate residuals to world coordinate system

    if ( motion == true) { 
        // integrate acceleration and add to velocity (uses trapezoidal integration technique
        m_worldVelocity = m_worldVelocity_previous + ((m_worldResiduals + m_worldResiduals_previous)*0.5f * dt );

        // Update Velocity
        m_worldVelocity = m_worldVelocity - ( m_worldVelocity_drift * dt );

        // integrate velocity and add to position
        m_worldPosition = m_worldPosition + ((m_worldVelocity + m_worldVelocity_previous)*0.5f)*dt;

        // keep history of previous values
        m_worldResiduals_previous = m_worldResiduals;
        m_worldVelocity_previous  = m_worldVelocity;

    } else {
        // Update Velocity Bias
        // When motion ends, velocity should be zero
        if ((motion_ended == true) && (m_dtmotion > 0.5f)) { // update velocity bias if we had at least half of second motion
            // m_worldVelocity_bias=m_worldVelocity/m_dtmotion;
            // the velocity bias is a drift
            m_worldVelocity_drift = ( (m_worldVelocity_drift * (1.0f - velocityDriftLearningAlpha)) + ((m_worldVelocity / m_dtmotion) * velocityDriftLearningAlpha ) );
        }

        // Reset Velocity
        m_worldVelocity.zero();    // minimize error propagation

        // Update acceleration bias
        m_residualsBias = ( (m_residualsBias * (1.0f - velocityDriftLearningAlpha)) + (residuals * velocityDriftLearningAlpha ) );
        // printf("%s", RTMath::displayRadians("Residuals Bias", m_residualsBias));
    }

    m_MotionData.worldPosition = m_worldPosition;
    m_MotionData.worldVelocity = m_worldVelocity;
    m_MotionData.worldAcceleration = m_worldResiduals;
    m_MotionData.worldVelocityDrift = m_worldVelocity_drift;
    m_MotionData.residuals          = residuals_cal;
    
}
		 