/*
 * SpeedSensors.cpp
 *
 *  Created on: 26.07.2014
 *      Author: niklausd
 */

#include "Arduino.h"
#include "Timer.h"
#include "DistanceCount.h"
#include "SpeedSensors.h"

//-----------------------------------------------------------------------------

const unsigned int SPEED_SENSORS_READ_TIMER_INTVL_MILLIS = 100;

const int IRQ_PIN_18 = 5;
const int IRQ_PIN_19 = 4;

const int L_SPEED_SENS_IRQ = IRQ_PIN_18;
const int R_SPEED_SENS_IRQ = IRQ_PIN_19;

volatile unsigned long int speedSensorCountLeft  = 0;
volatile unsigned long int speedSensorCountRight = 0;

// IRQ Service routines
void countLeftSpeedSensor()
{
  noInterrupts();
  speedSensorCountLeft++;
  interrupts();
}

void countRightSpeedSensor()
{
  noInterrupts();
  speedSensorCountRight++;
  interrupts();
}

//-----------------------------------------------------------------------------

class SpeedSensorReadTimerAdapter : public TimerAdapter
{
private:
  SpeedSensors* m_speedSensors;

public:
  SpeedSensorReadTimerAdapter(SpeedSensors* speedSensors)
  : m_speedSensors(speedSensors)
  { }

  void timeExpired()
  {
    noInterrupts();
    if (0 != m_speedSensors)
    {
      // read the speed sensor counters and reset them
      m_speedSensors->updateLeftWheelSpeed(speedSensorCountLeft);
      speedSensorCountLeft  = 0;
      m_speedSensors->updateRightWheelSpeed(speedSensorCountRight);
      speedSensorCountRight = 0;
    }
    interrupts();
  }
};

//-----------------------------------------------------------------------------

SpeedSensors::SpeedSensors()
: m_speedSensorReadTimer(new Timer(new SpeedSensorReadTimerAdapter(this), Timer::IS_RECURRING, SPEED_SENSORS_READ_TIMER_INTVL_MILLIS))
, m_lDistCount(new DistanceCount())
, m_rDistCount(new DistanceCount())
, m_leftWheelSpeed(0)
, m_rightWheelSpeed(0)
{
  attachInterrupt(L_SPEED_SENS_IRQ, countLeftSpeedSensor,  RISING);
  attachInterrupt(R_SPEED_SENS_IRQ, countRightSpeedSensor, RISING);
}

SpeedSensors::~SpeedSensors()
{
  detachInterrupt(L_SPEED_SENS_IRQ);
  detachInterrupt(R_SPEED_SENS_IRQ);

  delete m_speedSensorReadTimer->adapter();
  m_speedSensorReadTimer->attachAdapter(0);

  delete m_speedSensorReadTimer;
  m_speedSensorReadTimer = 0;

  delete m_lDistCount;
  m_lDistCount = 0;

  delete m_rDistCount;
  m_rDistCount = 0;
}

DistanceCount* SpeedSensors::lDistCount()
{
  return m_lDistCount;
}

DistanceCount* SpeedSensors::rDistCount()
{
  return m_rDistCount;
}

void SpeedSensors::updateLeftWheelSpeed(unsigned long int wheelSpeed)
{
  m_leftWheelSpeed = wheelSpeed;
  m_lDistCount->add(wheelSpeed);
}

void SpeedSensors::updateRightWheelSpeed(unsigned long int wheelSpeed)
{
  m_rightWheelSpeed = wheelSpeed;
  m_rDistCount->add(wheelSpeed);
}


unsigned long int SpeedSensors::leftWheelSpeed()
{
  return m_leftWheelSpeed;
}

unsigned long int SpeedSensors::rightWheelSpeed()
{
  return m_rightWheelSpeed;
}
