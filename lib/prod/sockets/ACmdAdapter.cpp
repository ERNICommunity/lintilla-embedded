/*
 * ACmdAdapter.cpp
 *
 *  Created on: 29.07.2014
 *      Author: niklausd
 */

#include "Arduino.h"
#include "Traction.h"
#include "DbgTraceContext.h"
#include "DbgTracePort.h"
#include "ACmdAdapter.h"

ACmdAdapter::ACmdAdapter(Traction* traction)
: m_traction(traction)
, m_trPort(new DbgTrace_Port(DbgTrace_Context::getContext(), "cmdAd", DbgTrace_Context::getContext()->getTraceOut("trConOut"), DbgTrace_Level::debug))
{ }

ACmdAdapter::~ACmdAdapter()
{
  delete m_trPort; m_trPort = 0;
}

void ACmdAdapter::stopAction()
{
  if (0 != m_traction)
  {
    m_traction->motorStop();
    TR_PRINT_STR(m_trPort, DbgTrace_Level::info, "stopAction()");
//    Serial.print("ACmdAdapter::stopAction()\n");
  }
}

void ACmdAdapter::moveForwardAction()
{
  if (0 != m_traction)
  {
    m_traction->moveForward();
    TR_PRINT_STR(m_trPort, DbgTrace_Level::info, "moveForwardAction()");
//    Serial.print("ACmdAdapter::moveForwardAction()\n");
  }
}

void ACmdAdapter::moveBackwardAction()
{
  if (0 != m_traction)
  {
    m_traction->moveBackward();
    TR_PRINT_STR(m_trPort, DbgTrace_Level::info, "moveBackwardAction()");
//    Serial.print("ACmdAdapter::moveBackwardAction()\n");
  }
}

void ACmdAdapter::spinOnPlaceLeftAction(float angle)
{
  if (0 != m_traction)
  {
    m_traction->spinOnPlace(false, angle);
    TR_PRINT_STR(m_trPort, DbgTrace_Level::info, "spinOnPlaceLeftAction()");
//    Serial.print("ACmdAdapter::spinOnPlaceLeftAction()\n");
  }
}

void ACmdAdapter::spinOnPlaceRightAction(float angle)
{
  if (0 != m_traction)
  {
    m_traction->spinOnPlace(true, angle);
    TR_PRINT_STR(m_trPort, DbgTrace_Level::info, "spinOnPlaceRightAction()");
//    Serial.print("ACmdAdapter::spinOnPlaceRightAction()\n");
  }
}
