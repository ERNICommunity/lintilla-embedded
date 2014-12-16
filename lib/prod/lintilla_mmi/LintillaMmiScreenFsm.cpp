/*
 * LintillaMmiScreenFsm.cpp
 *
 *  Created on: 15.09.2014
 *      Author: niklausd
 */
#include "Arduino.h"

#include <LintillaMmi.h>
#include <LintillaMmiScreen.h>
#include <LintillaMmiScreenFsm.h>
#include <LintillaMmiScreenState.h>

LintillaMmiScreenFsm::LintillaMmiScreenFsm(LintillaMmi* mmi)
: m_state(0)
{
  m_state = LintillaMmiWlanScreenState::Instance();
  m_state->attachScreen(new LintillaMmiWLANScreen(mmi));
  Serial.print("LintillaMmiScreenFsm ctor: created state ");
  Serial.println(m_state->toString());

  m_state = LintillaMmiIdScreenState::Instance();
  m_state->attachScreen(new LintillaMmiIdScreen(mmi));
  Serial.print("LintillaMmiScreenFsm ctor: created state ");
  Serial.println(m_state->toString());

  m_state = LintillaMmiHomeScreenState::Instance();
  m_state->attachScreen(new LintillaMmiHomeScreen(mmi));
  Serial.print("LintillaMmiScreenFsm ctor: created state ");
  Serial.println(m_state->toString());
}

LintillaMmiScreenFsm::~LintillaMmiScreenFsm()
{
  delete LintillaMmiIdScreenState::Instance()->screen();
  LintillaMmiIdScreenState::Instance()->attachScreen(0);

  delete LintillaMmiHomeScreenState::Instance()->screen();
  LintillaMmiHomeScreenState::Instance()->attachScreen(0);
}

void LintillaMmiScreenFsm::select()
{
  if (0 != m_state)
  {
    Serial.print("LintillaMmiScreenFsm::select() on state: ");
    Serial.println(m_state->toString());
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
