/*
 * BatteryImpl.cpp
 *
 *  Created on: 30.04.2014
 *      Author: niklausd
 */

#include "Timer.h"
#include "../Battery.h"
#include "BatteryVoltageEvalFsm.h"
#include "BatteryImpl.h"

//-----------------------------------------------------------------------------

class BattStartupTimerAdapter : public TimerAdapter
{
private:
  BatteryImpl* m_battImpl;

public:
  BattStartupTimerAdapter(BatteryImpl* battImpl)
  : m_battImpl(battImpl)
  { }

  void timeExpired()
  {
    if (0 != m_battImpl)
    {
      m_battImpl->startup();
    }
  }
};

//-----------------------------------------------------------------------------

class BattPollTimerAdapter : public TimerAdapter
{
private:
  BatteryImpl* m_battImpl;

public:
  BattPollTimerAdapter(BatteryImpl* battImpl)
  : m_battImpl(battImpl)
  { }

  void timeExpired()
  {
    if (0 != m_battImpl)
    {
      m_battImpl->evaluateStatus();
    }
  }
};

//-----------------------------------------------------------------------------

const unsigned int BatteryImpl::s_DEFAULT_STARTUP_TIME = 500;
const unsigned int BatteryImpl::s_DEFAULT_POLL_TIME = 1000;

const float BatteryImpl::s_BATT_WARN_THRSHD = 6.5;
const float BatteryImpl::s_BATT_STOP_THRSHD = 6.3;
const float BatteryImpl::s_BATT_SHUT_THRSHD = 6.1;
const float BatteryImpl::s_BATT_HYST        = 0.2;

const unsigned int BatteryImpl::s_V_ADC_FULLRANGE = 5;
const unsigned int BatteryImpl::s_N_ADC_FULLRANGE = 1023;


BatteryImpl::BatteryImpl(BatteryAdapter* adapter)
: m_adapter(adapter)
, m_evalFsm(new BatteryVoltageEvalFsm(this))
, m_startupTimer(new Timer(new BattStartupTimerAdapter(this), Timer::IS_NON_RECURRING, s_DEFAULT_STARTUP_TIME))
, m_pollTimer(new Timer(new BattPollTimerAdapter(this), Timer::IS_RECURRING))
, m_batteryVoltage(0.0)
, m_battVoltageSenseFactor(2.0)
{ }

BatteryImpl::~BatteryImpl()
{
  delete m_pollTimer->adapter();
  delete m_pollTimer; m_pollTimer = 0;

  delete m_startupTimer->adapter();
  delete m_startupTimer; m_startupTimer = 0;

  delete m_evalFsm;

  m_adapter = 0;
}

void BatteryImpl::attachAdapter(BatteryAdapter* adapter)
{
  m_adapter = adapter;
}

BatteryAdapter* BatteryImpl::adapter()
{
  return m_adapter;
}

void BatteryImpl::startup()
{
  if (0 != m_adapter)
  {
    m_battVoltageSenseFactor = m_adapter->readBattVoltageSenseFactor();
  }
  m_pollTimer->startTimer(s_DEFAULT_POLL_TIME);
}

void BatteryImpl::evaluateStatus()
{
  if ((0 != m_adapter) && (0 != m_evalFsm))
  {
    float batteryVoltage = m_adapter->readRawBattSenseValue() * m_battVoltageSenseFactor * s_V_ADC_FULLRANGE / s_N_ADC_FULLRANGE;

    bool isVoltageFalling = false;
    bool isVoltageRising = false;

    if (batteryVoltage < m_batteryVoltage)
    {
      isVoltageFalling = true;
    }
    else if (batteryVoltage > m_batteryVoltage)
    {
      isVoltageRising = true;
    }

    if (isVoltageFalling || isVoltageRising)
    {
      m_batteryVoltage = batteryVoltage;
      m_evalFsm->voltageChanged();
    }
  }
}

void BatteryImpl::battVoltageSensFactorChanged()
{
  if (0 != m_adapter)
  {
    m_battVoltageSenseFactor = m_adapter->readBattVoltageSenseFactor();
  }
}

float BatteryImpl::getBatteryVoltage()
{
  return m_batteryVoltage;
}

bool BatteryImpl::isBattVoltageOk()
{
  bool isVoltageOk = true;
  if (0 != m_evalFsm)
  {
    m_evalFsm->isBattVoltageOk();
  }
  return isVoltageOk;
}

bool BatteryImpl::isBattVoltageBelowWarnThreshold()
{
  bool isVoltageBelowWarnThreshold = false;
  if (0 != m_evalFsm)
  {
    m_evalFsm->isBattVoltageBelowWarnThreshold();
  }
  return isVoltageBelowWarnThreshold;
}

bool BatteryImpl::isBattVoltageBelowStopThreshold()
{
  bool isVoltageBelowStopThreshold = false;
  if (0 != m_evalFsm)
  {
    m_evalFsm->isBattVoltageBelowStopThreshold();
  }
  return isVoltageBelowStopThreshold;
}

bool BatteryImpl::isBattVoltageBelowShutdownThreshold()
{
  bool isVoltageBelowShutdownThreshold = false;
  if (0 != m_evalFsm)
  {
    m_evalFsm->isBattVoltageBelowShutdownThreshold();
  }
  return isVoltageBelowShutdownThreshold;
}

const char* BatteryImpl::getCurrentStateName()
{
  if (0 == m_evalFsm)
  {
    return "BatteryImpl::m_evalFsm, null pointer exception";
  }
  return m_evalFsm->state()->toString();
}

const char* BatteryImpl::getPreviousStateName()
{
  if (0 == m_evalFsm)
  {
    return "BatteryImpl::m_evalFsm, null pointer exception";
  }
  return m_evalFsm->state()->toString();
}
