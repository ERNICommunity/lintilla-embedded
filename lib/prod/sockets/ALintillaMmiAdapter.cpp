/*
 * ALintillaMmiAdapter.cpp
 *
 *  Created on: 14.07.2014
 *      Author: niklausd
 */

#include "Battery.h"
#include "CmdSequence.h"
#include "LintillaIvm.h"
#include "UltrasonicSensor.h"
#include "Adafruit_CC3000.h"
#include "SpeedSensors.h"
#include "DistanceCount.h"

#include <ALintillaMmiAdapter.h>

ALintillaMmiAdapter::ALintillaMmiAdapter(Battery* battery, CmdSequence* cmdSeq, LintillaIvm* ivm,
                                         UltrasonicSensor* ultrasonicSensorFront, Adafruit_CC3000* cc3000,
                                         SpeedSensors* speedSensors)
: LintillaMmiAdapter()
, m_battery(battery)
, m_cmdSeq(cmdSeq)
, m_ivm(ivm)
, m_ultrasonicSensorFront(ultrasonicSensorFront)
, m_cc3000(cc3000)
, m_speedSensors(speedSensors)
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
    if ((0 != m_speedSensors) && (0 != m_speedSensors->lDistCount()) && (0 != m_speedSensors->rDistCount()))
    {
      m_speedSensors->lDistCount()->reset();
      m_speedSensors->rDistCount()->reset();
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

void ALintillaMmiAdapter::setWlanSSID(const char* ssid, int length)
{
  if (0 != m_ivm)
  {
    m_ivm->setWlanSSID(ssid, length);
  }
}

int ALintillaMmiAdapter::getWlanSSID(char* out)
{
  int length = 0;
  if (0 != m_ivm)
  {
    length = m_ivm->getWlanSSID(out);
  }
  return length;
}

void ALintillaMmiAdapter::setWlanPASS(const char* pass, int length)
{
  if (0 != m_ivm)
  {
    m_ivm->setWlanPASS(pass, length);
  }
}

int ALintillaMmiAdapter::getWlanPASS(char* out)
{
  int length = 0;
  if (0 != m_ivm)
  {
    length = m_ivm->getWlanPASS(out);
  }
  return length;
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
  if (0 != m_battery)
  {
    isBelowWarnThreshold = m_battery->isBattVoltageBelowWarnThreshold();
  }
  return isBelowWarnThreshold;
}

bool ALintillaMmiAdapter::isBattVoltageBelowStopThreshold()
{
  bool isBelowStopThreshold = false;
  if (0 != m_battery)
  {
    isBelowStopThreshold = m_battery->isBattVoltageBelowStopThreshold();
  }
  return isBelowStopThreshold;
}

float ALintillaMmiAdapter::getBatteryVoltage()
{
  float battVoltage = 0.0;
  if (0 != m_battery)
  {
    battVoltage = m_battery->getBatteryVoltage();
  }
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

uint32_t ALintillaMmiAdapter::getCurrentIpAddress()
{
  uint32_t ipAddress = 0;
  uint32_t netmask   = 0;
  uint32_t gateway   = 0;
  uint32_t dhcpserv  = 0;
  uint32_t dnsserv   = 0;
  if (0 != m_cc3000)
  {
    m_cc3000->getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv);
  }
  return ipAddress;
}

long int ALintillaMmiAdapter::getLeftWheelSpeed()
{
  long int speed = 0;
  noInterrupts();
  speed = m_speedSensors->leftWheelSpeed();
  interrupts();
  return speed;
}

long int ALintillaMmiAdapter::getRightWheelSpeed()
{
  long int speed = 0;
  noInterrupts();
  speed = m_speedSensors->rightWheelSpeed();
  interrupts();
  return speed;
}

bool ALintillaMmiAdapter::isObstacleDetected()
{
  bool isObstacleDetected = false;
  return isObstacleDetected;
}
