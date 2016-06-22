/*
 * LintillaMmiScreenFsm.cpp
 *
 *  Created on: 15.09.2014
 *      Author: niklausd
 */
#include "Arduino.h"
#include "DbgTracePort.h"
#include <LintillaMmi.h>
#include <LintillaMmiScreen.h>
#include <LintillaMmiScreenFsm.h>
#include <LintillaMmiScreenState.h>

LintillaMmiScreenFsm::LintillaMmiScreenFsm(LintillaMmi* mmi)
: m_state(0)
, m_trPort(new DbgTrace_Port("mmifsm", DbgTrace_Level::info))
{
  m_state = LintillaMmiWlanScreenState::Instance();
  m_state->attachScreen(new LintillaMmiWLANScreen(mmi));
  TR_PRINT_STR(m_trPort , DbgTrace_Level::info, "LintillaMmiScreenFsm ctor: created state");
  TR_PRINT_STR(m_trPort , DbgTrace_Level::info, m_state->toString());

  m_state = LintillaMmiIdScreenState::Instance();
  m_state->attachScreen(new LintillaMmiIdScreen(mmi));
  TR_PRINT_STR(m_trPort , DbgTrace_Level::info, "LintillaMmiScreenFsm ctor: created state");
  TR_PRINT_STR(m_trPort , DbgTrace_Level::info, m_state->toString());

  m_state = LintillaMmiHomeScreenState::Instance();
  m_state->attachScreen(new LintillaMmiHomeScreen(mmi));
  TR_PRINT_STR(m_trPort , DbgTrace_Level::info, "LintillaMmiScreenFsm ctor: created state");
  TR_PRINT_STR(m_trPort , DbgTrace_Level::info, m_state->toString());
}

LintillaMmiScreenFsm::~LintillaMmiScreenFsm()
{
  delete LintillaMmiIdScreenState::Instance()->screen();
  LintillaMmiIdScreenState::Instance()->attachScreen(0);
  delete LintillaMmiIdScreenState::Instance();

  delete LintillaMmiHomeScreenState::Instance()->screen();
  LintillaMmiHomeScreenState::Instance()->attachScreen(0);
  delete LintillaMmiHomeScreenState::Instance();

  delete LintillaMmiWlanScreenState::Instance()->screen();
  LintillaMmiWlanScreenState::Instance()->attachScreen(0);
  delete LintillaMmiWlanScreenState::Instance();

  m_state = 0;
}

void LintillaMmiScreenFsm::select()
{
  if (0 != m_state)
  {
    TR_PRINT_STR(m_trPort , DbgTrace_Level::debug, "LintillaMmiScreenFsm::select() on state:");
    TR_PRINT_STR(m_trPort , DbgTrace_Level::debug, m_state->toString());
    m_state->select(this);
  }
}

void LintillaMmiScreenFsm::left()
{
  if (0 != m_state)
  {
    m_state->left(this);
  }
}

void LintillaMmiScreenFsm::right()
{
  if (0 != m_state)
  {
    m_state->right(this);
  }
}

void LintillaMmiScreenFsm::up()
{
  if (0 != m_state)
  {
    m_state->up(this);
  }
}

void LintillaMmiScreenFsm::down()
{
  if (0 != m_state)
  {
    m_state->down(this);
  }
}

void LintillaMmiScreenFsm::changeState(LintillaMmiScreenState* state)
{
  m_state = state;
  if (0 != m_state)
  {
    m_state->entry(this);
  }
}

void LintillaMmiScreenFsm::updateDisplay()
{
  if (0 != m_state)
  {
    m_state->updateDisplay();
  }
}

DbgTrace_Port* LintillaMmiScreenFsm::trPort()
{
  return m_trPort;
}

