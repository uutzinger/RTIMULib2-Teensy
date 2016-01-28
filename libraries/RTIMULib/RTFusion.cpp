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


#include "RTFusion.h"
#include "RTIMUHal.h"

//  The slerp power valule controls the influence of the measured state to correct the predicted state
//  0 = measured state ignored (just gyros), 1 = measured state overrides predicted state.
//  In between 0 and 1 mixes the two conditions

#define RTQF_SLERP_POWER (RTFLOAT)0.02;

const char *RTFusion::m_fusionNameMap[] = {
    "NULL",
    "Kalman STATE4",
    "RTQF",
    "AHRS"};

RTFusion::RTFusion()
{
    m_debug = false;
    m_firstTime = true;
    m_enableGyro = true;
    m_enableAccel = true;
    m_enableCompass = true;

    m_gravity.setScalar(0);
    m_gravity.setX(0);
    m_gravity.setY(0);
    m_gravity.setZ(1);

    m_slerpPower = RTQF_SLERP_POWER;
}

RTFusion::~RTFusion()
{
}

void RTFusion::calculatePose(const RTVector3& accel, const RTVector3& mag, float magDeclination)
{
    RTQuaternion m;
    RTQuaternion q;

    if (m_enableAccel) {
        accel.accelToEuler(m_measuredPose);
    } else {
        m_measuredPose = m_fusionPose;
        m_measuredPose.setZ(0);
    }

    if (m_enableCompass && m_compassValid) {
        q.fromEuler(m_measuredPose);
        m.setScalar(0);
        m.setX(mag.x());
        m.setY(mag.y());
        m.setZ(mag.z());

        m = q * m * q.conjugate();
        m_measuredPose.setZ(-atan2(m.y(), m.x()) - magDeclination);
    } else {
        m_measuredPose.setZ(m_fusionPose.z());
    }

    m_measuredQPose.fromEuler(m_measuredPose);

    //  check for quaternion aliasing. If the quaternion has the wrong sign
    //  the kalman filter will be very unhappy.

    int maxIndex = -1;
    RTFLOAT maxVal = -1000;

    for (int i = 0; i < 4; i++) {
        if (fabs(m_measuredQPose.data(i)) > maxVal) {
            maxVal = fabs(m_measuredQPose.data(i));
            maxIndex = i;
        }
    }

    //  if the biggest component has a different sign in the measured and kalman poses,
    //  change the sign of the measured pose to match.

    if (((m_measuredQPose.data(maxIndex) < 0) && (m_fusionQPose.data(maxIndex) > 0)) ||
            ((m_measuredQPose.data(maxIndex) > 0) && (m_fusionQPose.data(maxIndex) < 0))) {
        m_measuredQPose.setScalar(-m_measuredQPose.scalar());
        m_measuredQPose.setX(-m_measuredQPose.x());
        m_measuredQPose.setY(-m_measuredQPose.y());
        m_measuredQPose.setZ(-m_measuredQPose.z());
        m_measuredQPose.toEuler(m_measuredPose);
    }
}


RTVector3 RTFusion::getAccelResiduals()
{
    RTQuaternion rotatedGravity;
    // RTQuaternion fusionQPoseConjugate;
    //RTQuaternion qTemp;
    RTVector3 residuals;

    //  do gravity rotation and subtraction

    // create the conjugate of the pose

    // fusionQPoseConjugate = m_fusionQPose.conjugate();

    // now do the rotation - takes two steps with qTemp as the intermediate variable
    // rotatedGravity = fusionQPoseConjugate * m_gravity * m_fusionQPose;;
    // qTemp = m_gravity * m_fusionQPose; 
    // Above code is replace with this:
    // qTemp.setScalar(-m_fusionQPose.z());
    // qTemp.setX(-m_fusionQPose.y());
    // qTemp.setY(m_fusionQPose.x());
    // qTemp.setZ(m_fusionQPose.scalar());	
    // rotatedGravity = fusionQPoseConjugate * qTemp;
    // residuals.setX( - (rotatedGravity.x() - m_accel.x() ) );
    // residuals.setY( - (rotatedGravity.y() - m_accel.y() ) );
    // residuals.setZ( - (rotatedGravity.z() - m_accel.z() ) );
	
    /** because gravity is zero except z is 1, we can simplify
    SAGE CODE	
	N.<c,d,qas,qax,qay,qaz,qbs,qbx,qby,qbz,s> = QQ[]
	H.<i,j,k> = QuaternionAlgebra(c,d)
	a = qas + qax * i + qay * j + qaz * k
	b = qbs + qbx * i + qby * j + qbz * k
	gravity = 0 + 0 * i + 0 * j + 1 * k
	roatedGravity = a.conjugate() * gravity * a
	
	Results:
	s : 0
	x : (2*qax*qaz - 2*qas*qay)*i 
	y : (2*qay*qaz + 2*qas*qax)*j
	z : (  qaz^2  - qax^2 -qay^2 + qas^2)*k
    **/
    
	rotatedGravity.setX(2.0f*m_fusionQPose.x() * m_fusionQPose.z() - 2.0f*m_fusionQPose.scalar() * m_fusionQPose.y());
	rotatedGravity.setY(2.0f*m_fusionQPose.y() * m_fusionQPose.z() + 2.0f*m_fusionQPose.scalar() * m_fusionQPose.x());
	rotatedGravity.setZ(m_fusionQPose.z() * m_fusionQPose.z() - m_fusionQPose.x() * m_fusionQPose.x() - m_fusionQPose.y() * m_fusionQPose.y() + m_fusionQPose.scalar() * m_fusionQPose.scalar());
	
	// making sure gravity remains 1g after rotation
    rotatedGravity.normalize();
    
	// now get the residuals
    residuals.setX( m_accel.x() - rotatedGravity.x() );
    residuals.setY( m_accel.y() - rotatedGravity.y() );
    residuals.setZ( m_accel.z() - rotatedGravity.z() );
    return residuals;
}
