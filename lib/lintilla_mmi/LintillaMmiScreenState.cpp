/*
 * LintillaMmiScreenState.cpp
 *
 *  Created on: 15.09.2014
 *      Author: niklausd
 */

#include "Arduino.h"
#include "DbgTracePort.h"
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
  if (0 != fsm)
  {
    TR_PRINT_STR(fsm->trPort(), DbgTrace_Level::debug, "LintillaMmiHomeScreenState::entry() on state:");
    TR_PRINT_STR(fsm->trPort(), DbgTrace_Level::debug, toString());
  }
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
  if ((0 != screen()) && (0 != fsm))
  {
    if (screen()->isEditMode())
    {
      screen()->setEditMode(false);
    }
    else
    {
      fsm->changeState(LintillaMmiWlanScreenState::Instance());
    }
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
  if (0 != fsm)
  {
    TR_PRINT_STR(fsm->trPort(), DbgTrace_Level::debug, "LintillaMmiIdScreenState::entry() on state:");
    TR_PRINT_STR(fsm->trPort(), DbgTrace_Level::debug, toString());
  }
  updateDisplay();
}

//-----------------------------------------------------------------------------

LintillaMmiScreenState* LintillaMmiWlanScreenState::s_instance = 0;

LintillaMmiScreenState* LintillaMmiWlanScreenState::Instance()
{
  if (0 == s_instance)
  {
    s_instance = new LintillaMmiWlanScreenState();
  }
  return s_instance;
}

const char* LintillaMmiWlanScreenState::toString()
{
  return "WlanScreenState";
}

void LintillaMmiWlanScreenState::select(LintillaMmiScreenFsm* fsm)
{
  if ((0 != fsm) && (0 != screen()))
  {
    if (!screen()->isEditMode())
    {
      fsm->changeState(LintillaMmiHomeScreenState::Instance());
    }
  }
  else
  {
    fsm->changeState(LintillaMmiHomeScreenState::Instance());
  }
}

void LintillaMmiWlanScreenState::left(LintillaMmiScreenFsm* fsm)
{
  if (0 != screen())
  {
    if (!screen()->isEditMode())
    {
      screen()->setEditMode(true);
    }
    else
    {
      screen()->setEditMode(false);
    }
  }
}

void LintillaMmiWlanScreenState::right(LintillaMmiScreenFsm* fsm)
{
  if ((0 != fsm) && (0 != screen()))
  {
    if (!screen()->isEditMode())
    {
      fsm->changeState(LintillaMmiHomeScreenState::Instance());
    }
  }
}

void LintillaMmiWlanScreenState::up(LintillaMmiScreenFsm* fsm)
{
  if (0 != screen())
  {
    screen()->setCursorUp();
  }
}

void LintillaMmiWlanScreenState::down(LintillaMmiScreenFsm* fsm)
{
  if (0 != screen())
  {
    screen()->setCursorDown();
  }
}

void LintillaMmiWlanScreenState::entry(LintillaMmiScreenFsm* fsm)
{
  if (0 != fsm)
  {
    TR_PRINT_STR(fsm->trPort(), DbgTrace_Level::debug, "LintillaMmiWlanScreenState::entry() on state:");
    TR_PRINT_STR(fsm->trPort(), DbgTrace_Level::debug, toString());
  }
  updateDisplay();
}
