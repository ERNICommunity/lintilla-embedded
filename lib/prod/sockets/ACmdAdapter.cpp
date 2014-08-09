/*
 * ACmdAdapter.cpp
 *
 *  Created on: 29.07.2014
 *      Author: niklausd
 */

#include "Arduino.h"
#include "Traction.h"
#include <ACmdAdapter.h>

ACmdAdapter::ACmdAdapter(Traction* traction)
: m_traction(traction)
{ }

ACmdAdapter::~ACmdAdapter()
{ }

void ACmdAdapter::stopAction()
{
  if (0 != m_traction)
  {
    m_traction->motorStop();
    Serial.print("ACmdAdapter::stopAction()\n");
  }
}

void ACmdAdapter::moveForwardAction()
{
  if (0 != m_traction)
  {
    m_traction->moveForward();
    Serial.print("ACmdAdapter::moveForwardAction()\n");
  }
}

void ACmdAdapter::moveBackwardAction()
{
  if (0 != m_traction)
  {
    m_traction->moveBackward();
    Serial.print("ACmdAdapter::moveBackwardAction()\n");
  }
}

void ACmdAdapter::spinOnPlaceLeftAction(float angle)
{
  if (0 != m_traction)
  {
    m_traction->spinOnPlace(false, angle);
    Serial.print("ACmdAdapter::spinOnPlaceLeftAction()\n");
  }
}

void ACmdAdapter::spinOnPlaceRightAction(float angle)
{
  if (0 != m_traction)
  {
    m_traction->spinOnPlace(true, angle);
    Serial.print("ACmdAdapter::spinOnPlaceRightAction()\n");
  }
}
