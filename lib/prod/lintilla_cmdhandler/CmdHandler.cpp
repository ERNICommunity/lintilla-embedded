/*
 * Cmd.cpp
 *
 *  Created on: 10.03.2014
 *      Author: niklausd
 */

#include "CmdAdapter.h"
#include "CmdSequence.h"
#include "CmdHandler.h"

CmdHandler::CmdHandler(CmdSequence* cmdSeq, unsigned int timeMillis, const char* name)
: m_cmdSeq(cmdSeq)
, m_timeMillis(timeMillis)
, m_name(name)
, m_next(0)
{
  if (0 != cmdSeq)
  {
    cmdSeq->attach(this);
  }
}

CmdHandler::~CmdHandler()
{
  if (0 != m_cmdSeq)
  {
    m_cmdSeq->detach(this);
  }
}

void CmdHandler::setTime(unsigned int timeMillis)
{
  m_timeMillis = timeMillis;
}

unsigned int CmdHandler::getTime()
{
  return m_timeMillis;
}

CmdHandler* CmdHandler::next()
{
  return m_next;
}

void CmdHandler::setNext(CmdHandler* next)
{
  m_next = next;
}

const char* CmdHandler::getName()
{
  return m_name;
}


CmdSequence* CmdHandler::cmdSequence()
{
  return m_cmdSeq;
}

//-----------------------------------------------------------------------------

CmdStop::CmdStop(CmdSequence* cmdSeq, unsigned int timeMillis)
: CmdHandler(cmdSeq, timeMillis, "CmdStop")
{ }

void CmdStop::execute()
{
  if ((0 != cmdSequence()) && (0 != cmdSequence()->adapter()))
  {
    cmdSequence()->adapter()->stopAction();
  }
}

//-----------------------------------------------------------------------------

CmdMoveForward::CmdMoveForward(CmdSequence* cmdSeq, unsigned int timeMillis)
: CmdHandler(cmdSeq, timeMillis, "CmdMoveForward")
{ }

void CmdMoveForward::execute()
{
  if ((0 != cmdSequence()) && (0 != cmdSequence()->adapter()))
  {
    cmdSequence()->adapter()->moveForwardAction();
  }
}

//-----------------------------------------------------------------------------

CmdMoveBackward::CmdMoveBackward(CmdSequence* cmdSeq, unsigned int timeMillis)
: CmdHandler(cmdSeq, timeMillis, "CmdMoveBackward")
{ }

void CmdMoveBackward::execute()
{
  if ((0 != cmdSequence()) && (0 != cmdSequence()->adapter()))
  {
    cmdSequence()->adapter()->moveBackwardAction();
  }
}

//-----------------------------------------------------------------------------

CmdSpinOnPlaceLeft::CmdSpinOnPlaceLeft(CmdSequence* cmdSeq, unsigned int timeMillis, float angle)
: CmdHandler(cmdSeq, timeMillis, "CmdSpinOnPlaceLeft")
, m_angle(angle)
{ }

void CmdSpinOnPlaceLeft::execute()
{
  if ((0 != cmdSequence()) && (0 != cmdSequence()->adapter()))
  {
    cmdSequence()->adapter()->spinOnPlaceLeftAction(m_angle);
  }
}

//-----------------------------------------------------------------------------

CmdSpinOnPlaceRight::CmdSpinOnPlaceRight(CmdSequence* cmdSeq, unsigned int timeMillis, float angle)
: CmdHandler(cmdSeq, timeMillis, "CmdSpinOnPlaceRight")
, m_angle(angle)
{ }

void CmdSpinOnPlaceRight::execute()
{
  if ((0 != cmdSequence()) && (0 != cmdSequence()->adapter()))
  {
    cmdSequence()->adapter()->spinOnPlaceRightAction(m_angle);
  }
}

//-----------------------------------------------------------------------------
