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

// UU: This file was changed
// include underwater depth calculation
// added a few more operations to vectors and quaternions
// added tilt compensated heading

#include "RTMath.h"
#include <Arduino.h>

//  Strings are put here. So the display functions are no re-entrant!

char RTMath::m_string[1000];

uint64_t RTMath::currentUSecsSinceEpoch()
{
    return micros();
}

const char *RTMath::displayRadians(const char *label, RTVector3& vec)
{
    sprintf(m_string, "%s: x:%+4.3f, y:%+4.3f, z:%+4.3f, s:%+4.3f\n", label, vec.x(), vec.y(), vec.z(), vec.length());
    // sprintf(m_string, "%s: x:%f, y:%f, z:%f\n", label, vec.x(), vec.y(), vec.z());
    return m_string;
}

const char *RTMath::displayDegrees(const char *label, RTVector3& vec)
{
    sprintf(m_string, "%s: roll:%+3.2f, pitch:%+3.2f, yaw:%+3.2f\n", label, vec.x() * RTMATH_RAD_TO_DEGREE,
            vec.y() * RTMATH_RAD_TO_DEGREE, vec.z() * RTMATH_RAD_TO_DEGREE);
    return m_string;
}

const char *RTMath::display(const char *label, RTQuaternion& quat)
{
    sprintf(m_string, "%s: scalar: %+4.4f, x:%+4.4f, y:%+4.4f, z:%+4.4f\n", label, quat.scalar(), quat.x(), quat.y(), quat.z());
    // sprintf(m_string, "%s: scalar: %f, x:%f, y:%f, z:%f\n", label, quat.scalar(), quat.x(), quat.y(), quat.z());
    return m_string;
}

const char *RTMath::display(const char *label, RTMatrix4x4& mat)
{
    sprintf(m_string, "%s(0): %f %f %f %f\n%s(1): %f %f %f %f\n%s(2): %f %f %f %f\n%s(3): %f %f %f %f\n",
            label, mat.val(0,0), mat.val(0,1), mat.val(0,2), mat.val(0,3),
            label, mat.val(1,0), mat.val(1,1), mat.val(1,2), mat.val(1,3),
            label, mat.val(2,0), mat.val(2,1), mat.val(2,2), mat.val(2,3),
            label, mat.val(3,0), mat.val(3,1), mat.val(3,2), mat.val(3,3));
    return m_string;
}

//  convertPressureToHeight() - the conversion uses the formula:
//
//  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
//
//  where:
//  h  = height above sea level
//  T0 = standard temperature at sea level = 288.15
//  L0 = standard temperature elapse rate = -0.0065
//  p  = measured pressure
//  P0 = static pressure = 1013.25 (but can be overridden)
//  g0 = gravitational acceleration = 9.80665
//  M  = molecular mass of earth's air = 0.0289644
//  R* = universal gas constant = 8.31432
//
//  Given the constants, this works out to:
//
//  h = 44330.8 * (1 - (p / P0)**0.190263)

RTFLOAT RTMath::convertPressureToHeight(RTFLOAT pressure, RTFLOAT staticPressure)
{
    return 44330.8 * (1 - pow(pressure / staticPressure, (RTFLOAT)0.190263));
}

RTFLOAT RTMath::convertPressureToDepth(RTFLOAT pressure, RTFLOAT staticPressure)
{
    return (pressure - staticPressure) * 0.01019716;	
    //http://www.seabird.com/document/an69-conversion-pressure-depth
    // pressure is in mbar and depth in meters
}

RTFLOAT RTMath::convertPressureLatitudeToDepth(RTFLOAT pressure, RTFLOAT staticPressure, RTFLOAT latitude)
{
    RTFLOAT temp = sin(latitude / 57.29578);
    RTFLOAT x = temp * temp ; 
    RTFLOAT g = 9.780318 * ( 1.0 + ( 5.2788E-3  + 2.36E-5  * x) * x ) + 1.092E-6 * (pressure / 10000.0);
    RTFLOAT p = pressure - staticPressure;
    return ((((-1.82E-15  * p + 2.279E-10 ) * p - 2.251E-5 ) * p + 9.72659) * p) / g;
    // http://www.seabird.com/document/an69-conversion-pressure-depth
}
RTFLOAT RTMath::clamp2PI(RTFLOAT x) {
	while ((x) >= (2.0f*RTMATH_PI)) (x) -= (2.0f*RTMATH_PI); 
	while ((x) < 0) (x) += (2.0f*RTMATH_PI); 
	return x;
}
RTVector3 RTMath::toWorld(const RTVector3& vec, const RTQuaternion& q)
{
    // Vector rotation by quaternion
    // P_out = q * P_in * conj(q)
    // - P_out is the output vector
    // - q is the orientation quaternion
    // - P_in is the input vector
    // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
	
    RTQuaternion q_tmp;
    RTQuaternion q_con;
    
    RTVector3 world_vec;
	
    q_tmp.setScalar(0.0f);
    q_tmp.setX(vec.x());
    q_tmp.setY(vec.y());
    q_tmp.setZ(vec.z());
    
    // Backwards
    q_tmp = q * q_tmp * q.conjugate();
    
    world_vec.setX(q_tmp.x());
    world_vec.setY(q_tmp.y());
    world_vec.setZ(q_tmp.z());

    return world_vec;
}
RTVector3 RTMath::poseFromAccelMag(const RTVector3& accel, const RTVector3& mag)
{
    // Estimate Pose Vector from Accelerometer and Compass
    // 1) Acceleration to Roll Pitch Yaw, Yaw is zero/undefined
    // 2) Convert RPY to estimated Pose Quaternion
    // 3) Rotate Compass to World form estimated Pose
    // 4) Update estimated Yaw in Euler and return result
    RTVector3 result;
    RTQuaternion m;
    RTQuaternion q;

    accel.accelToEuler(result);
    // result is roll/pitch/yaw, yaw is zero

//  q.fromEuler(result);
//  since result.z() is always 0, this can be optimized a little

    RTFLOAT cosX2 = cos(result.x() / 2.0f);
    RTFLOAT sinX2 = sin(result.x() / 2.0f);
    RTFLOAT cosY2 = cos(result.y() / 2.0f);
    RTFLOAT sinY2 = sin(result.y() / 2.0f);

    q.setScalar(cosX2 * cosY2);
    q.setX(sinX2 * cosY2);
    q.setY(cosX2 * sinY2);
    q.setZ(-sinX2 * sinY2);
//    q.normalize();

    // rotate Magnetometer to Q pose
    m.setScalar(0);
    m.setX(mag.x());
    m.setY(mag.y());
    m.setZ(mag.z());

    m = q * m * q.conjugate();
    // Update Yaw in RPY from Magnetometer
    result.setZ(-atan2(m.y(), m.x()));
    return result;
}

void RTMath::convertToVector(unsigned char *rawData, RTVector3& vec, RTFLOAT scale, bool bigEndian)
{
    if (bigEndian) {
        vec.setX((RTFLOAT)((int16_t)(((uint16_t)rawData[0] << 8) | (uint16_t)rawData[1])) * scale);
        vec.setY((RTFLOAT)((int16_t)(((uint16_t)rawData[2] << 8) | (uint16_t)rawData[3])) * scale);
        vec.setZ((RTFLOAT)((int16_t)(((uint16_t)rawData[4] << 8) | (uint16_t)rawData[5])) * scale);
    } else {
        vec.setX((RTFLOAT)((int16_t)(((uint16_t)rawData[1] << 8) | (uint16_t)rawData[0])) * scale);
        vec.setY((RTFLOAT)((int16_t)(((uint16_t)rawData[3] << 8) | (uint16_t)rawData[2])) * scale);
        vec.setZ((RTFLOAT)((int16_t)(((uint16_t)rawData[5] << 8) | (uint16_t)rawData[4])) * scale);
     }
}



//----------------------------------------------------------
//
//  The RTVector3 class

RTVector3::RTVector3()
{
    zero();
}

RTVector3::RTVector3(RTFLOAT x, RTFLOAT y, RTFLOAT z)
{
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

RTVector3& RTVector3::operator =(const RTVector3& vec)
{
    if (this == &vec)
        return *this;

    m_data[0] = vec.m_data[0];
    m_data[1] = vec.m_data[1];
    m_data[2] = vec.m_data[2];

    return *this;
}

RTVector3& RTVector3::operator =(const RTFLOAT val)
{
    for (int i = 0; i < 3; i++)
        m_data[i] = val;
    return *this;
}

RTVector3& RTVector3::operator +=(const RTFLOAT val)
{
    for (int i = 0; i < 3; i++)
        m_data[i] += val;
    return *this;
}

RTVector3& RTVector3::operator +=(const RTVector3& vec)
{
    for (int i = 0; i < 3; i++)
        m_data[i] += vec.m_data[i];
    return *this;
}

RTVector3& RTVector3::operator -=(const RTFLOAT val)
{
    for (int i = 0; i < 3; i++)
        m_data[i] -= val;
    return *this;
}

RTVector3& RTVector3::operator -=(const RTVector3& vec)
{
    for (int i = 0; i < 3; i++)
        m_data[i] -= vec.m_data[i];
    return *this;
}

RTVector3& RTVector3::operator *=(const RTFLOAT val)
{
    m_data[0] *= val;
    m_data[1] *= val;
    m_data[2] *= val;

    return *this;
}

RTVector3& RTVector3::operator *=(const RTVector3& vec)
{
    RTVector3 va;
    RTVector3 vb;
    //RTFLOAT dotAB;
    RTVector3 crossAB;

    va.setX(m_data[0]);
    va.setY(m_data[1]);
    va.setZ(m_data[2]);

    //dotAB = RTVector3::dotProduct(va, vec);
    RTVector3::crossProduct(va, vec, crossAB);

    m_data[0] = vb.x() + va.x() + crossAB.x();
    m_data[1] = vb.y() + va.y() + crossAB.y();
    m_data[2] = vb.z() + va.z() + crossAB.z();

    return *this;
}

RTVector3& RTVector3::operator /=(const RTFLOAT val)
{
    m_data[0] /= val;
    m_data[1] /= val;
    m_data[2] /= val;

    return *this;
}

const RTVector3 RTVector3::operator *(const RTVector3& vec) const
{
    RTVector3 result = *this;
    result *= vec;
    return result;
}

const RTVector3 RTVector3::operator *(const RTFLOAT val) const
{
    RTVector3 result = *this;
    result *= val;
    return result;
}

const RTVector3 RTVector3::operator /(const RTFLOAT val) const
{
    RTVector3 result = *this;
    result /= val;
    return result;
}


const RTVector3 RTVector3::operator -(const RTVector3& vec) const
{
    RTVector3 result = *this;
    result -= vec;
    return result;
}

const RTVector3 RTVector3::operator -(const RTFLOAT val) const
{
    RTVector3 result = *this;
    result -= val;
    return result;
}

const RTVector3 RTVector3::operator +(const RTVector3& vec) const
{
    RTVector3 result = *this;
    result += vec;
    return result;
}

const RTVector3 RTVector3::operator +(const RTFLOAT val) const
{
    RTVector3 result = *this;
    result += val;
    return result;
}

void RTVector3::zero()
{
    for (int i = 0; i < 3; i++)
        m_data[i] = 0;
}


RTFLOAT RTVector3::dotProduct(const RTVector3& a, const RTVector3& b)
{
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

void RTVector3::crossProduct(const RTVector3& a, const RTVector3& b, RTVector3& d)
{
    d.setX(a.y() * b.z() - a.z() * b.y());
    d.setY(a.z() * b.x() - a.x() * b.z());
    d.setZ(a.x() * b.y() - a.y() * b.x());
}


void RTVector3::accelToEuler(RTVector3& rollPitchYaw) const
{
    RTVector3 normAccel = *this;

    normAccel.normalize();

    rollPitchYaw.setX(atan2(normAccel.y(), normAccel.z()));
    rollPitchYaw.setY(-atan2(normAccel.x(), sqrt(normAccel.y() * normAccel.y() + normAccel.z() * normAccel.z())));
    rollPitchYaw.setZ(0);
}


void RTVector3::accelToQuaternion(RTQuaternion& qPose) const
{
    RTVector3 normAccel = *this;
    RTVector3 vec;
    RTVector3 z(0, 0, 1.0);

    normAccel.normalize();

    RTFLOAT angle = acos(RTVector3::dotProduct(z, normAccel));
    RTVector3::crossProduct(normAccel, z, vec);
    vec.normalize();

    qPose.fromAngleVector(angle, vec);
}


void RTVector3::normalize()
{
    RTFLOAT length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2]);

    if (length == 0)
        return;

    m_data[0] /= length;
    m_data[1] /= length;
    m_data[2] /= length;
}

RTFLOAT RTVector3::length()
{
    return sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2]);
}
RTFLOAT RTVector3::squareLength()
{
   return m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2];
}
RTFLOAT RTVector3::toHeading(const RTVector3& mag, const float declination)
{
  // Tilt compensated heading from compass, skips conversion to RPY, as those are provided as input
  RTVector3 rpy = *this;
  float heading;
  
  float cos_roll = cos(rpy.x());
  float sin_roll = sin(rpy.x());
  float cos_pitch = cos(rpy.y());
  float sin_pitch = sin(rpy.y());
  
  // Tilt compensated Magnetic field X component:
  float Head_X = mag.x()*cos_pitch + mag.y()*sin_roll*sin_pitch + mag.z()*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y component:
  float Head_Y = mag.y()*cos_roll - mag.z()*sin_roll;
  // Magnetic Heading
  heading = -atan2(Head_Y,Head_X) -declination;
  if(heading < -9990) { heading = 0; }
  heading = RTMath::clamp2PI(heading);

  return (heading);
}
//----------------------------------------------------------
//
//  The RTQuaternion class

RTQuaternion::RTQuaternion()
{
    zero();
}

RTQuaternion::RTQuaternion(RTFLOAT scalar, RTFLOAT x, RTFLOAT y, RTFLOAT z)
{
    m_data[0] = scalar;
    m_data[1] = x;
    m_data[2] = y;
    m_data[3] = z;
}

RTQuaternion& RTQuaternion::operator =(const RTQuaternion& quat)
{
    if (this == &quat)
        return *this;

    m_data[0] = quat.m_data[0];
    m_data[1] = quat.m_data[1];
    m_data[2] = quat.m_data[2];
    m_data[3] = quat.m_data[3];

    return *this;
}

RTQuaternion& RTQuaternion::operator =(const RTFLOAT val)
{
    for (int i = 0; i < 4; i++)
        m_data[i] = val;
    return *this;
}

RTQuaternion& RTQuaternion::operator +=(const RTFLOAT val)
{
    for (int i = 0; i < 4; i++)
        m_data[i] += val;
    return *this;
}

RTQuaternion& RTQuaternion::operator +=(const RTQuaternion& quat)
{
    for (int i = 0; i < 4; i++)
        m_data[i] += quat.m_data[i];
    return *this;
}

RTQuaternion& RTQuaternion::operator -=(const RTFLOAT val)
{
    for (int i = 0; i < 4; i++)
        m_data[i] -= val;
    return *this;
}

RTQuaternion& RTQuaternion::operator -=(const RTQuaternion& quat)
{
    for (int i = 0; i < 4; i++)
        m_data[i] -= quat.m_data[i];
    return *this;
}

RTQuaternion& RTQuaternion::operator *=(const RTFLOAT val)
{
    m_data[0] *= val;
    m_data[1] *= val;
    m_data[2] *= val;
    m_data[3] *= val;
    return *this;
}

RTQuaternion& RTQuaternion::operator *=(const RTQuaternion& qb)
{
    RTQuaternion qa;

    qa = *this;

    m_data[0] = qa.scalar() * qb.scalar() - qa.x() * qb.x() - qa.y() * qb.y() - qa.z() * qb.z();
    m_data[1] = qa.scalar() * qb.x() + qa.x() * qb.scalar() + qa.y() * qb.z() - qa.z() * qb.y();
    m_data[2] = qa.scalar() * qb.y() - qa.x() * qb.z() + qa.y() * qb.scalar() + qa.z() * qb.x();
    m_data[3] = qa.scalar() * qb.z() + qa.x() * qb.y() - qa.y() * qb.x() + qa.z() * qb.scalar();
    
	/*
	SAGE CODE
	N.<c,d,qas,qax,qay,qaz,qbs,qbx,qby,qbz,s> = QQ[]
	H.<i,j,k> = QuaternionAlgebra(c,d)
	a = qas + qax * i + qay * j + qaz * k
	b = qbs + qbx * i + qby * j + qbz * k
	a*b
	
	-qaz*qbz  - qax*qbx - qay*qby + qas*qbs  
	(-qaz*qby + qay*qbz + qax*qbs + qas*qbx)*i 
	(qaz*qbx  - qax*qbz + qay*qbs + qas*qby)*j
	(qaz*qbs  - qay*qbx + qax*qby + qas*qbz)*k
    */
	
    return *this;
}


RTQuaternion& RTQuaternion::operator /=(const RTFLOAT val)
{
    m_data[0] /= val;
    m_data[1] /= val;
    m_data[2] /= val;
    m_data[3] /= val;

    return *this;
}

RTQuaternion& RTQuaternion::operator /=(const RTQuaternion& qb)
{
    RTQuaternion qa;
    
    qa = *this;
    
    RTFLOAT qsql = qb.scalar()*qb.scalar() + qb.x() * qb.x() +
            qb.y() * qb.y() + qb.z() * qb.z();
    qa *= (qb.conjugate() / qsql);
    
    m_data[0] =   qa.scalar();
    m_data[1] =   qa.x();
    m_data[2] =   qa.y();
    m_data[3] =   qa.z();
  
    return *this;
}

const RTQuaternion RTQuaternion::operator *(const RTQuaternion& qb) const
{
    RTQuaternion result = *this;
    result *= qb;
    return result;
}

const RTQuaternion RTQuaternion::operator *(const RTFLOAT val) const
{
    RTQuaternion result = *this;
    result *= val;
    return result;
}

const RTQuaternion RTQuaternion::operator /(const RTQuaternion& qb) const
{
    RTQuaternion result = *this;
    result /= qb;
    return result;
}

const RTQuaternion RTQuaternion::operator /(const RTFLOAT val) const
{
    RTQuaternion result = *this;
    result /= val;
    return result;
}

const RTQuaternion RTQuaternion::operator -(const RTQuaternion& qb) const
{
    RTQuaternion result = *this;
    result -= qb;
    return result;
}

const RTQuaternion RTQuaternion::operator -(const RTFLOAT val) const
{
    RTQuaternion result = *this;
    result -= val;
    return result;
}

const RTQuaternion RTQuaternion::operator +(const RTQuaternion& qb) const
{
    RTQuaternion result = *this;
    result += qb;
    return result;
}

const RTQuaternion RTQuaternion::operator +(const RTFLOAT val) const
{
    RTQuaternion result = *this;
    result += val;
    return result;
}

void RTQuaternion::zero()
{
    for (int i = 0; i < 4; i++)
        m_data[i] = 0;
}

void RTQuaternion::normalize()
{
    RTFLOAT length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2] + m_data[3] * m_data[3]);

    if ((length == 0) || (length == 1))
        return;

    m_data[0] /= length;
    m_data[1] /= length;
    m_data[2] /= length;
    m_data[3] /= length;
}

void RTQuaternion::toEuler(RTVector3& vec)
{
    // RT Code same as Wikipedia
    // Ideally quaternion is normalized before calling this function

    vec.setX(atan2(2.0 * (m_data[2] * m_data[3] + m_data[0] * m_data[1]),
            1 - 2.0 * (m_data[1] * m_data[1] + m_data[2] * m_data[2])));

    vec.setY(asin(2.0 * (m_data[0] * m_data[2] - m_data[1] * m_data[3])));

    vec.setZ(atan2(2.0 * (m_data[1] * m_data[2] + m_data[0] * m_data[3]),
            1 - 2.0 * (m_data[2] * m_data[2] + m_data[3] * m_data[3])));

    // Matlab Code, uses more multiplications, no normalization needed
    //vec.setX(atan2(2.0f * (m_data[2] * m_data[3] + m_data[0] * m_data[1]),
    //        (m_data[0] * m_data[0] - m_data[1] * m_data[1] - m_data[2] * m_data[2] + m_data[3] * m_data[3] )));
    //
    //vec.setY(asin(2.0f * (m_data[0] * m_data[2] - m_data[1] * m_data[3])));
    // 
    //vec.setZ(atan2(2.0f * (m_data[1] * m_data[2] + m_data[0] * m_data[3]),
    //        (m_data[0] * m_data[0] + m_data[1] * m_data[1] - m_data[2] * m_data[2] - m_data[3] * m_data[3])));
	
}

void RTQuaternion::toGravity(RTVector3& vec) 
{
// Creates Gravity vector from pose quaternion
  vec.setX( 2.0f * (m_data[1]*m_data[3] - m_data[0]*m_data[2]));
  vec.setY( 2.0f * (m_data[0]*m_data[1] + m_data[2]*m_data[3]));
  vec.setZ( m_data[0]*m_data[0] - m_data[1]*m_data[1] - m_data[2]*m_data[2] + m_data[3]*m_data[3]);
}

void RTQuaternion::fromEuler(RTVector3& vec)
{
    // RT Code same as Wikipedia and Matlab
    RTFLOAT cosX2 = cos(vec.x() / 2.0f);
    RTFLOAT sinX2 = sin(vec.x() / 2.0f);
    RTFLOAT cosY2 = cos(vec.y() / 2.0f);
    RTFLOAT sinY2 = sin(vec.y() / 2.0f);
    RTFLOAT cosZ2 = cos(vec.z() / 2.0f);
    RTFLOAT sinZ2 = sin(vec.z() / 2.0f);

    m_data[0] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
    m_data[1] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
    m_data[2] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
    m_data[3] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
    normalize();
}

RTQuaternion RTQuaternion::conjugate() const
{
    RTQuaternion q;
    q.setScalar(m_data[0]);
    q.setX(-m_data[1]);
    q.setY(-m_data[2]);
    q.setZ(-m_data[3]);
    return q;
}

RTFLOAT RTQuaternion::length()
{
   return sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2] + m_data[3] * m_data[3]);
}

RTFLOAT RTQuaternion::squareLength()
{
   return m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2] + m_data[3] * m_data[3];
}


void RTQuaternion::toAngleVector(RTFLOAT& angle, RTVector3& vec)
{
    // Converts quaternion to vector and rotation around that vector
    // is reverse from Angle Vector
    
    RTFLOAT halfTheta;
    RTFLOAT sinHalfTheta;

    halfTheta = acos(m_data[0]);
    sinHalfTheta = sin(halfTheta);

    if (sinHalfTheta == 0) {
        vec.setX(1.0);
        vec.setY(0);
        vec.setZ(0);
    } else {
        vec.setX(m_data[1] / sinHalfTheta);
        vec.setY(m_data[1] / sinHalfTheta);
        vec.setZ(m_data[1] / sinHalfTheta);
    }
    angle = 2.0 * halfTheta;
}

void RTQuaternion::fromAngleVector(const RTFLOAT& angle, const RTVector3& vec)
{
    // Converts rotation of angle around vector to quaternion
    RTFLOAT sinHalfTheta = sin(angle / 2.0);
    m_data[0] = cos(angle / 2.0);
    m_data[1] = vec.x() * sinHalfTheta;
    m_data[2] = vec.y() * sinHalfTheta;
    m_data[3] = vec.z() * sinHalfTheta;
}

RTFLOAT RTQuaternion::toHeading(const RTVector3& mag, const float declination)
{
  // Tilt compensated heading from compass
  // Corrected for local magnetic declination
  RTVector3 rpy;
  float heading;

  // Convert pose quaternion to RPY
  RTQuaternion q = *this;
  q.toEuler(rpy);
  
  float cos_roll = cos(rpy.x());
  float sin_roll = sin(rpy.x());
  float cos_pitch = cos(rpy.y());
  float sin_pitch = sin(rpy.y());
  
  // Tilt compensated Magnetic field X component:
  float Head_X = mag.x()*cos_pitch + mag.y()*sin_roll*sin_pitch + mag.z()*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y component:
  float Head_Y = mag.y()*cos_roll - mag.z()*sin_roll;
  // Magnetic Heading
  heading = -atan2(Head_Y,Head_X) - declination;

  if(heading < -9990) { heading = 0; }
  heading = RTMath::clamp2PI(heading);

  return (heading);
}

//----------------------------------------------------------
//
//  The RTMatrix4x4 class

RTMatrix4x4::RTMatrix4x4()
{
    fill(0);
}

RTMatrix4x4& RTMatrix4x4::operator =(const RTMatrix4x4& mat)
{
    if (this == &mat)
        return *this;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] = mat.m_data[row][col];

    return *this;
}


void RTMatrix4x4::fill(RTFLOAT val)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] = val;
}


RTMatrix4x4& RTMatrix4x4::operator +=(const RTMatrix4x4& mat)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] += mat.m_data[row][col];

    return *this;
}

RTMatrix4x4& RTMatrix4x4::operator -=(const RTMatrix4x4& mat)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] -= mat.m_data[row][col];

    return *this;
}

RTMatrix4x4& RTMatrix4x4::operator *=(const RTFLOAT val)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] *= val;

    return *this;
}

const RTMatrix4x4 RTMatrix4x4::operator +(const RTMatrix4x4& mat) const
{
    RTMatrix4x4 result = *this;
    result += mat;
    return result;
}

const RTMatrix4x4 RTMatrix4x4::operator *(const RTFLOAT val) const
{
    RTMatrix4x4 result = *this;
    result *= val;
    return result;
}


const RTMatrix4x4 RTMatrix4x4::operator *(const RTMatrix4x4& mat) const
{
    RTMatrix4x4 res;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            res.m_data[row][col] =
                    m_data[row][0] * mat.m_data[0][col] +
                    m_data[row][1] * mat.m_data[1][col] +
                    m_data[row][2] * mat.m_data[2][col] +
                    m_data[row][3] * mat.m_data[3][col];

    return res;
}


const RTQuaternion RTMatrix4x4::operator *(const RTQuaternion& q) const
{
    RTQuaternion res;

    res.setScalar(m_data[0][0] * q.scalar() + m_data[0][1] * q.x() + m_data[0][2] * q.y() + m_data[0][3] * q.z());
    res.setX(m_data[1][0] * q.scalar() + m_data[1][1] * q.x() + m_data[1][2] * q.y() + m_data[1][3] * q.z());
    res.setY(m_data[2][0] * q.scalar() + m_data[2][1] * q.x() + m_data[2][2] * q.y() + m_data[2][3] * q.z());
    res.setZ(m_data[3][0] * q.scalar() + m_data[3][1] * q.x() + m_data[3][2] * q.y() + m_data[3][3] * q.z());

    return res;
}

void RTMatrix4x4::setToIdentity()
{
    fill(0);
    m_data[0][0] = 1;
    m_data[1][1] = 1;
    m_data[2][2] = 1;
    m_data[3][3] = 1;
}

RTMatrix4x4 RTMatrix4x4::transposed()
{
    RTMatrix4x4 res;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            res.m_data[col][row] = m_data[row][col];
    return res;
}

//  Note:
//  The matrix inversion code here was strongly influenced by some old code I found
//  but I have no idea where it came from. Apologies to whoever wrote it originally!
//  If it's you, please let me know at info@richards-tech.com so I can credit it correctly.

RTMatrix4x4 RTMatrix4x4::inverted()
{
    RTMatrix4x4 res;

    RTFLOAT det = matDet();

    if (det == 0) {
        res.setToIdentity();
        return res;
    }

    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if ((row + col) & 1)
                res.m_data[col][row] = -matMinor(row, col) / det;
            else
                res.m_data[col][row] = matMinor(row, col) / det;
        }
    }

    return res;
}

RTFLOAT RTMatrix4x4::matDet()
{
    RTFLOAT det = 0;

    det += m_data[0][0] * matMinor(0, 0);
    det -= m_data[0][1] * matMinor(0, 1);
    det += m_data[0][2] * matMinor(0, 2);
    det -= m_data[0][3] * matMinor(0, 3);
    return det;
}

RTFLOAT RTMatrix4x4::matMinor(const int row, const int col)
{
    static int map[] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};

    int *rc;
    int *cc;
    RTFLOAT res = 0;

    rc = map + row * 3;
    cc = map + col * 3;

    res += m_data[rc[0]][cc[0]] * m_data[rc[1]][cc[1]] * m_data[rc[2]][cc[2]];
    res -= m_data[rc[0]][cc[0]] * m_data[rc[1]][cc[2]] * m_data[rc[2]][cc[1]];
    res -= m_data[rc[0]][cc[1]] * m_data[rc[1]][cc[0]] * m_data[rc[2]][cc[2]];
    res += m_data[rc[0]][cc[1]] * m_data[rc[1]][cc[2]] * m_data[rc[2]][cc[0]];
    res += m_data[rc[0]][cc[2]] * m_data[rc[1]][cc[0]] * m_data[rc[2]][cc[1]];
    res -= m_data[rc[0]][cc[2]] * m_data[rc[1]][cc[1]] * m_data[rc[2]][cc[0]];
    return res;
}

