/*
 * LintillaMmiScreenState.cpp
 *
 *  Created on: 15.09.2014
 *      Author: niklausd
 */

#include "Arduino.h"

#include <LintillaMmi.h>
#include <LintillaMmiScreenFsm.h>
#include <LintillaMmiScreenState.h>
#include <LintillaMmiScreen.h>

LintillaMmiScreenState::LintillaMmiScreenState()
: m_screen(0)
{ }

LintillaMmiScreenState::~LintillaMmiScreenState()
{ }

void LintillaMmiScreenState::updateDisplay()
{
  screen()->updateDisplay();
}

//-----------------------------------------------------------------------------

LintillaMmiScreenState* LintillaMmiHomeScreenState::s_instance = 0;

LintillaMmiScreenState* LintillaMmiHomeScreenState::Instance()
{
  if (0 == s_instance)
  {
    s_instance = new LintillaMmiHomeScreenState();
  }
  return s_instance;
}

const char* LintillaMmiHomeScreenState::toString()
{
  return "HomeScreenState";
}

void LintillaMmiHomeScreenState::select(LintillaMmiScreenFsm* fsm)
{
  if ((0 != fsm) && (0 != screen()) && (0 != screen()->mmi()) && (0 != screen()->mmi()->adapter()))
  {
    if (!screen()->mmi()->adapter()->isSeqRunning())
    {
      fsm->changeState(LintillaMmiIdScreenState::Instance());
    }
  }
}

void LintillaMmiHomeScreenState::left(LintillaMmiScreenFsm* fsm)
{
  if ((0 != screen()) && (0 != screen()->mmi()) && (0 != screen()->mmi()->adapter()))
  {
    if (!screen()->mmi()->adapter()->isSeqRunning())
    {
      screen()->mmi()->adapter()->startSequence();
    }
  }
}

void LintillaMmiHomeScreenState::right(LintillaMmiScreenFsm* fsm)
{
  if ((0 != screen()) && (0 != screen()->mmi()) && (0 != screen()->mmi()->adapter()))
  {
    screen()->mmi()->adapter()->stopSequence();
  }
}

void LintillaMmiHomeScreenState::up(LintillaMmiScreenFsm* fsm)
{
  if (0 != screen())
  {
    screen()->setCursorUp();
  }
}

void LintillaMmiHomeScreenState::down(LintillaMmiScreenFsm* fsm)
{
  if (0 != screen())
  {
    screen()->setCursorDown();
  }
}

void LintillaMmiHomeScreenState::entry(LintillaMmiScreenFsm* fsm)
{
  Serial.println(toString());
  updateDisplay();
}

//-----------------------------------------------------------------------------

LintillaMmiScreenState* LintillaMmiIdScreenState::s_instance = 0;

LintillaMmiScreenState* LintillaMmiIdScreenState::Instance()
{
  if (0 == s_instance)
  {
    s_instance = new LintillaMmiIdScreenState();
  }
  return s_instance;
}

const char* LintillaMmiIdScreenState::toString()
{
  return "IdScreenState";
}

void LintillaMmiIdScreenState::select(LintillaMmiScreenFsm* fsm)
{
  if (screen()->isEditMode())
  {
    screen()->setEditMode(false);
  }
}

void LintillaMmiIdScreenState::left(LintillaMmiScreenFsm* fsm)
{
  if (0 != screen())
  {
    if (!screen()->isEditMode())
    {
      screen()->setEditMode(true);
    }
  }
}

void LintillaMmiIdScreenState::right(LintillaMmiScreenFsm* fsm)
{
  if ((0 != fsm) && (0 != screen()))
  {
    if (!screen()->isEditMode())
    {
      fsm->changeState(LintillaMmiHomeScreenState::Instance());
    }
  }
}

void LintillaMmiIdScreenState::up(LintillaMmiScreenFsm* fsm)
{
  if (0 != screen())
  {
    screen()->setCursorUp();
  }
}

void LintillaMmiIdScreenState::down(LintillaMmiScreenFsm* fsm)
{
  if (0 != screen())
  {
    screen()->setCursorDown();
  }
}

void LintillaMmiIdScreenState::entry(LintillaMmiScreenFsm* fsm)
{
  Serial.println(toString());
  updateDisplay();
}
