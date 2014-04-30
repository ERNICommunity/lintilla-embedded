/*
 * CmdSequence.cpp
 *
 *  Created on: 10.03.2014
 *      Author: niklausd
 */

#include "Timer.h"
#include "TimerAdapter.h"
#include "Cmd.h"
#include "CmdAdapter.h"
#include "CmdSequence.h"

class CmdSeqTimerAdapter : public TimerAdapter
{
public:
  CmdSeqTimerAdapter(CmdSequence* cmdSeq)
  : m_cmdSeq(cmdSeq)
  { }

  void timeExpired()
  {
    if (0 != m_cmdSeq)
    {
      m_cmdSeq->execNextCmd();
    }
  }

private:
  CmdSequence* m_cmdSeq;
};

CmdSequence::CmdSequence(CmdAdapter* adapter)
: m_isRunning(false)
, m_firstCmd(0)
, m_currentCmd(0)
, m_adapter(adapter)
, m_timer(new Timer(new CmdSeqTimerAdapter(this), Timer::IS_NON_RECURRING))
{ }

CmdSequence::~CmdSequence()
{ }

void CmdSequence::start()
{
  m_currentCmd = m_firstCmd;
  execCmd();
}

void CmdSequence::stop()
{
  m_timer->cancelTimer();
  if (0 != adapter())
  {
    adapter()->stopAction();
  }
  m_isRunning = false;
}

bool CmdSequence::isRunning()
{
  return m_isRunning;
}


void CmdSequence::printCmdNameList()
{
  Cmd* next = m_firstCmd;
  while (0 != next)
  {
    next->printName();
    next = next->next();
  }
}

void CmdSequence::execNextCmd()
{
  m_currentCmd = m_currentCmd->next();
  execCmd();
}

void CmdSequence::execCmd()
{
  if ((0 != m_currentCmd) && (0 != m_timer))
  {
    m_isRunning = true;
    m_timer->startTimer(m_currentCmd->getTime());
    m_currentCmd->execute();
  }
  else
  {
    m_isRunning = false;
    if (0 != m_timer)
    {
      m_timer->cancelTimer();
    }
    m_currentCmd = m_firstCmd;
  }
}

void CmdSequence::attach(Cmd* cmd)
{
  if (0 == m_firstCmd)
  {
    m_firstCmd = cmd;
  }
  else
  {
    Cmd* next = m_firstCmd;
    while (next->next() != 0)
    {
      next = next->next();
    }
    next->setNext(cmd);
  }
}

void CmdSequence::detach(Cmd* cmd)
{
  if (m_firstCmd == cmd)
  {
    m_firstCmd = cmd->next();
  }
  else
  {
    Cmd* next = m_firstCmd;
    while ((next != 0) && next->next() != cmd)
    {
      next = next->next();
    }
    if (next != 0)
    {
      next->setNext(cmd->next());
    }
  }
}
