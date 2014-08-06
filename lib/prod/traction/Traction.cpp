/*
 * Traction.cpp
 *
 *  Created on: 27.07.2014
 *      Author: niklausd
 */

#include "SN754410Driver.h"
#include "Traction.h"

//-----------------------------------------------------------------------------

const int Traction::cSpeed     = 200;
const int Traction::cSpinSpeed = 150;

const int Traction::speedPin1 = 44;
const int Traction::speedPin2 = 45;

const int Traction::motor1APin = 46;
const int Traction::motor3APin = 47;

int Traction::motor2APin = 48;
int Traction::motor4APin = 49;

//-----------------------------------------------------------------------------

Traction::Traction(TractionAdapter* adapter)
: m_motorL(new SN754410_Driver(speedPin1, motor1APin, motor2APin))
, m_motorR(new SN754410_Driver(speedPin2, motor3APin, motor4APin))
, m_adapter(adapter)
, m_speed_value_motor_left(0)
, m_speed_value_motor_right(0)
, m_isLeftMotorFwd(false)
, m_isRightMotorFwd(false)
{ }

Traction::~Traction()
{
  delete m_motorL;
  m_motorL = 0;

  delete m_motorR;
  m_motorR = 0;
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

void Traction::moveStraight(bool forward)
{
  m_isLeftMotorFwd  = forward;
  m_isRightMotorFwd = forward;
  m_speed_value_motor_left  = cSpeed;
  m_speed_value_motor_right = cSpeed;
  updateActors();
}

void Traction::spinOnPlace(bool right, float angle)
{
//  setPointAngle = startAngle + angle;
  m_isLeftMotorFwd = right;
  m_isRightMotorFwd = !right;
  m_speed_value_motor_left  = cSpinSpeed;
  m_speed_value_motor_right = cSpinSpeed;
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
