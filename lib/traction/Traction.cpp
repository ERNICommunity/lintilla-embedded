/*
 * Traction.cpp
 *
 *  Created on: 27.07.2014
 *      Author: niklausd
 */

#include <Arduino.h>
#include <DbgTraceContext.h>
#include <DbgTraceLevel.h>
#include <DbgTraceOut.h>
#include <DbgTracePort.h>
#include <SN754410Driver.h>
#include <Timer.h>
#include <Traction.h>

//-----------------------------------------------------------------------------

const int Traction::MOTOR_SPEED_STRAIGHT  = 200;
const int Traction::MOTOR_SPIN_SPEED      = 150;

const int Traction::SPEED_PIN1            = 44;
const int Traction::SPEED_PIN2            = 45;

const int Traction::MOTOR_1A_PIN          = 46;
const int Traction::MOTOR_3A_PIN          = 47;

const int Traction::MOTOR_2A_PIN          = 48;
const int Traction::MOTOR_4A_PIN          = 49;

const unsigned int Traction::PID_SAMPLING_RATE     = 200;

//-----------------------------------------------------------------------------


class ControllerTimerAdapter : public TimerAdapter
{
public:
  ControllerTimerAdapter(Traction* traction)
  : m_traction(traction)
  { }

  void timeExpired()
  {
    if (0 != m_traction)
    {
      m_traction->readjustAngle();
    }
  }

private:
  Traction* m_traction;
};


Traction::Traction(TractionAdapter* adapter)
: m_motorL(new SN754410_Driver(SPEED_PIN1, MOTOR_1A_PIN, MOTOR_2A_PIN))
, m_motorR(new SN754410_Driver(SPEED_PIN2, MOTOR_3A_PIN, MOTOR_4A_PIN))
, m_adapter(adapter)
, m_speed_value_motor_left(0)
, m_speed_value_motor_right(0)
, m_isLeftMotorFwd(false)
, m_isRightMotorFwd(false)
, m_targetAngle(0.0)
, m_pidTimer(new Timer(new ControllerTimerAdapter(this), Timer::IS_RECURRING))
, m_tracePort(new DbgTrace_Port("trac", "traceConsoleOut", DbgTrace_Level::debug))
{ }

Traction::~Traction()
{
  delete m_motorL;
  m_motorL = 0;

  delete m_motorR;
  m_motorR = 0;

  delete m_tracePort;
  m_tracePort = 0;

  delete m_pidTimer;
  m_pidTimer = 0;
}

void Traction::attachAdapter(TractionAdapter* adapter)
{
  m_adapter = adapter;
}

TractionAdapter* Traction::adapter()
{
  return m_adapter;
}

void Traction::motorStop()
{
  m_pidTimer->cancelTimer();
  m_speed_value_motor_left  = 0;
  m_speed_value_motor_right = 0;
  updateActors();
}

void Traction::moveBackward()
{
  moveStraight(false);
}

void Traction::moveForward()
{
  moveStraight(true);
}

void Traction::moveControlledForward()
{
  float speedDiff;

  m_pidTimer->startTimer(PID_SAMPLING_RATE);

  if (0 != m_adapter)
  {
    m_targetAngle = m_adapter->getYawAngle(); //angle should be constant during this movement
    m_adapter->setTargetAngle((double)m_targetAngle);
  }
  m_isLeftMotorFwd  = true;
  m_isRightMotorFwd = true;
  m_speed_value_motor_left  = MOTOR_SPEED_STRAIGHT;
  m_speed_value_motor_right = MOTOR_SPEED_STRAIGHT;

  updateActors();
}

void Traction::readjustAngle()
{
  float speedDiff;

  if (0 != m_adapter)
  {
    speedDiff = m_adapter->computeSpeedDiff()/2;
    // limit the PID output speed and prevent change of sign.
    if (speedDiff > MOTOR_SPEED_STRAIGHT/2)
    {
      speedDiff = MOTOR_SPEED_STRAIGHT/2;
    }
    if (speedDiff < MOTOR_SPEED_STRAIGHT/-2)
    {
      speedDiff = MOTOR_SPEED_STRAIGHT/-2;
    }
    m_speed_value_motor_left  = MOTOR_SPEED_STRAIGHT - speedDiff;
    m_speed_value_motor_right = MOTOR_SPEED_STRAIGHT + speedDiff;

    if (0 != m_tracePort)
    {
      //TR_PRINT_DBL(m_tracePort, DbgTrace_Level::debug, speedDiff);
      TR_PRINT_DBL(m_tracePort, DbgTrace_Level::debug, m_adapter->getYawAngle());
    }
    updateActors();
  }
}

void Traction::moveStraight(bool forward)
{
  m_isLeftMotorFwd  = forward;
  m_isRightMotorFwd = forward;
  m_speed_value_motor_left  = MOTOR_SPEED_STRAIGHT;
  m_speed_value_motor_right = MOTOR_SPEED_STRAIGHT;
  updateActors();
}

void Traction::spinOnPlace(bool right, float angle)
{
  m_isLeftMotorFwd = right;
  m_isRightMotorFwd = !right;
  m_speed_value_motor_left  = MOTOR_SPIN_SPEED;
  m_speed_value_motor_right = MOTOR_SPIN_SPEED;
  updateActors();
}

void Traction::updateActors()
{
  int speedAndDirectionLeft  = m_speed_value_motor_left  * (m_isLeftMotorFwd  ? 1 : -1);
  int speedAndDirectionRight = m_speed_value_motor_right * (m_isRightMotorFwd ? 1 : -1);

  if (0 != m_adapter)
  {
    m_adapter->notifyDirectionChange(m_isLeftMotorFwd || m_isRightMotorFwd);
  }

  if ((0 != m_motorL) && (0 != m_motorR))
  {
    m_motorL->setSpeed(speedAndDirectionLeft);
    m_motorR->setSpeed(speedAndDirectionRight);
  }
}
