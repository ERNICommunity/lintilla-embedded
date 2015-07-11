/*
 * ATractionAdapter.cpp
 *
 *  Created on: 30.07.2014
 *      Author: niklausd
 */

#include <ATractionAdapter.h>

#define KP_DEFAULT 10.0
#define KI_DEFAULT 10.0
#define KD_DEFAULT 0.0

ATractionAdapter::ATractionAdapter(UltrasonicSensor* ultrasonicSensor, FreeSixIMU* sixImuSensor)
: m_ultrasonicSensor(ultrasonicSensor)
, m_sixImuSensor(sixImuSensor)
, m_yawActualAngle(0.0)
, m_yawTargetAngle(0.0)
, m_calculatedSpeedDiff(0.0)
, m_Kp(KP_DEFAULT)
, m_Ki(KI_DEFAULT)
, m_Kd(KP_DEFAULT)

{
//  m_ultrasonicSensor->setIsObstacleDetectionActive(true);
  m_pid = new PID(&m_yawActualAngle, &m_calculatedSpeedDiff, &m_yawTargetAngle,
      m_Kp, m_Ki, m_Kd, DIRECT);
}

ATractionAdapter::~ATractionAdapter()
{ }

float ATractionAdapter::getYawAngle()
{
  float ypr[3];

  m_sixImuSensor->getYawPitchRoll(ypr);
  return ypr[0];
}

void ATractionAdapter::notifyDirectionChange(bool isForward)
{
  if (0 != m_ultrasonicSensor)
  {
    m_ultrasonicSensor->setIsObstacleDetectionActive(isForward);
  }
}

void ATractionAdapter::setTargetAngle(double targetAngle)
{
  m_yawTargetAngle = targetAngle;
}

float ATractionAdapter::computeSpeedDiff()
{
  m_yawActualAngle = getYawAngle();
  m_pid->Compute();
  Serial.print(m_calculatedSpeedDiff);
  return m_calculatedSpeedDiff;
}
