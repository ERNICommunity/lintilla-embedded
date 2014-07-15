/*
 * ALintillaMmiAdapter.cpp
 *
 *  Created on: 14.07.2014
 *      Author: niklausd
 */
#include "CmdSequence.h"
#include "LintillaIvm.h"
#include "UltrasonicSensor.h"
#include "Adafruit_CC3000.h"
#include "DistanceCount.h"

#include <ALintillaMmiAdapter.h>

ALintillaMmiAdapter::ALintillaMmiAdapter(CmdSequence* cmdSeq, LintillaIvm* ivm,
                                         UltrasonicSensor* ultrasonicSensorFront, Adafruit_CC3000* cc3000,
                                         DistanceCount* lDistCount, DistanceCount* rDistCount)
: LintillaMmiAdapter()
, m_cmdSeq(cmdSeq)
, m_ivm(ivm)
, m_ultrasonicSensorFront(ultrasonicSensorFront)
, m_cc3000(cc3000)
, m_lDistCount(lDistCount)
, m_rDistCount(rDistCount)
{ }

ALintillaMmiAdapter::~ALintillaMmiAdapter()
{ }

bool ALintillaMmiAdapter::isSeqRunning()
{
  bool isRunning = false;
  if (0 != m_cmdSeq)
  {
    isRunning = m_cmdSeq->isRunning();
  }
  return isRunning;
}

void ALintillaMmiAdapter::startSequence()
{
  if ((0 != m_cmdSeq) && (!isObstacleDetected()) && (!isBattVoltageBelowWarnThreshold()))
  {
    if ((0 != m_lDistCount) && (0 != m_rDistCount))
    {
      m_lDistCount->reset();
      m_rDistCount->reset();
    }
    m_cmdSeq->start();
  }
}

void ALintillaMmiAdapter::stopSequence()
{
  if (0 != m_cmdSeq)
  {
    m_cmdSeq->stop();
  }
}

unsigned char ALintillaMmiAdapter::getDeviceId()
{
  unsigned char deviceId = 0;
  if (0 != m_ivm)
  {
    deviceId = m_ivm->getDeviceId();
  }
  return deviceId;
}

void ALintillaMmiAdapter::setDeviceId(unsigned char deviceId)
{
  if (0 != m_ivm)
  {
    m_ivm->setDeviceId(deviceId);
  }
}

unsigned char ALintillaMmiAdapter::getIvmVersion()
{
  unsigned char ivmVersion = 0;
  if (0 != m_ivm)
  {
    ivmVersion = m_ivm->getIvmVersion();
  }
 return ivmVersion;
}

bool ALintillaMmiAdapter::isFrontDistSensLimitExceeded()
{
  bool isLimitExceeded = (UltrasonicSensor::DISTANCE_LIMIT_EXCEEDED == getFrontDistanceCM());
  return isLimitExceeded;
}

unsigned long ALintillaMmiAdapter::getFrontDistanceCM()
{
  unsigned long dist = UltrasonicSensor::DISTANCE_LIMIT_EXCEEDED;   // [cm]
  if (0 != m_ultrasonicSensorFront)
  {
    dist = m_ultrasonicSensorFront->getDistanceCM();
  }
  return dist;
}

bool ALintillaMmiAdapter::isBattVoltageBelowWarnThreshold()
{
  bool isBelowWarnThreshold = false;
  return isBelowWarnThreshold;
}

float ALintillaMmiAdapter::getBatteryVoltage()
{
  float battVoltage = 0.0;
  return battVoltage;
}

bool ALintillaMmiAdapter::isWlanConnected()
{
  bool isConnected = false;
  if (0 != m_cc3000)
  {
    isConnected = m_cc3000->checkConnected();
  }
  return isConnected;
}

unsigned int ALintillaMmiAdapter::getCurrentIpAddress()
{
  return 0;
}

long int ALintillaMmiAdapter::getLeftWheelSpeed()
{
  long int speed = 0;
//  noInterrupts();
//  speed = leftWheelSpeed;
//  interrupts();
  return speed;
}

long int ALintillaMmiAdapter::getRightWheelSpeed()
{
  long int speed = 0;
//  noInterrupts();
//  speed = rightWheelSpeed;
//  interrupts();
  return speed;
}

bool ALintillaMmiAdapter::isObstacleDetected()
{
  bool isObstacleDetected = false;
  return isObstacleDetected;
}
