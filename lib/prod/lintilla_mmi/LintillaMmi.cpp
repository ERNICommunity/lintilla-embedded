/*
 * LintillaMMI.cpp
 *
 *  Created on: 12.07.2014
 *      Author: niklausd
 */

#include "Arduino.h"

#include "LcdKeypad.h"
#include "Blanking.h"
#include "Timer.h"
#include "LintillaMmi.h"
#include "LintillaMmiScreenFsm.h"

//-----------------------------------------------------------------------------

const unsigned int cUpdateDisplayInterval = 200; // Display update interval [ms]
class DisplayTimerAdapter : public TimerAdapter
{
private:
  LintillaMmi* m_mmi;

public:
  DisplayTimerAdapter(LintillaMmi* mmi)
  : m_mmi(mmi)
  { }

  void timeExpired()
  {
    if (0 != m_mmi)
    {
      m_mmi->updateDisplay();
    }
  }
};

//-----------------------------------------------------------------------------

class MyLcdKeypadAdapter : public LcdKeypadAdapter
{
public:
  MyLcdKeypadAdapter(LintillaMmi* mmi)
  : m_mmi(mmi)
  { }

private:
  LintillaMmi* m_mmi;

public:
  void handleKeyChanged(LcdKeypad::Key newKey)
  {
    Serial.print("MyLcdKeypadAdapter::handleKeyChanged(), newKey: ");
    Serial.println((LcdKeypad::NO_KEY == newKey)     ? "NO_KEY"     :
                   (LcdKeypad::SELECT_KEY == newKey) ? "SELECT_KEY" :
                   (LcdKeypad::LEFT_KEY == newKey)   ? "LEFT_KEY"   :
                   (LcdKeypad::UP_KEY == newKey)     ? "UP_KEY"     :
                   (LcdKeypad::DOWN_KEY == newKey)   ? "DOWN_KEY"   :
                   (LcdKeypad::RIGHT_KEY == newKey)  ? "RIGHT_KEY"  : "OOPS!! Invalid value!!");

    if ((0 != m_mmi) && (0 != m_mmi->screenFsm()))
    {
      switch (newKey)
      {
        case LcdKeypad::SELECT_KEY: m_mmi->screenFsm()->select(); break;
        case LcdKeypad::LEFT_KEY: m_mmi->screenFsm()->left();     break;
        case LcdKeypad::RIGHT_KEY: m_mmi->screenFsm()->right();   break;
        case LcdKeypad::UP_KEY: m_mmi->screenFsm()->up();         break;
        case LcdKeypad::DOWN_KEY: m_mmi->screenFsm()->down();     break;
        default:
          break;
      }
    }
  }
};

//-----------------------------------------------------------------------------

LintillaMmi::LintillaMmi(LintillaMmiAdapter* adapter)
: m_lcdKeypad(new LcdKeypad())
, m_displayBlanking(new Blanking())
, m_adapter(adapter)
, m_displayTimer(new Timer(new DisplayTimerAdapter(this), Timer::IS_RECURRING, cUpdateDisplayInterval))
, m_screenFsm(new LintillaMmiScreenFsm(this))
, m_isBacklightOn(true)
{
  if (0 != m_lcdKeypad)
  {
    m_lcdKeypad->attachAdapter(new MyLcdKeypadAdapter(this));
  }
}

LintillaMmi::~LintillaMmi()
{
  delete m_displayBlanking;
  m_displayBlanking = 0;

  delete m_displayTimer->adapter();
  m_displayTimer->attachAdapter(0);

  delete m_displayTimer;
  m_displayTimer = 0;

  delete m_lcdKeypad->adapter();
  m_lcdKeypad->attachAdapter(0);

  delete m_lcdKeypad;
  m_lcdKeypad = 0;
}

LcdKeypad* LintillaMmi::lcdKeypad()
{
  return m_lcdKeypad;
}

Blanking* LintillaMmi::displayBlanking()
{
  return m_displayBlanking;
}

LintillaMmiScreenFsm* LintillaMmi::screenFsm()
{
  return m_screenFsm;
}

void LintillaMmi::attachAdapter(LintillaMmiAdapter* adapter)
{
  m_adapter = adapter;
}

LintillaMmiAdapter* LintillaMmi::adapter()
{
  return m_adapter;
}

bool LintillaMmi::isBacklightOn()
{
  return m_isBacklightOn;
}

void LintillaMmi::setBackLightOn(bool isBacklightOn)
{
  m_isBacklightOn = isBacklightOn;
}

void LintillaMmi::lcdBackLightControl()
{
  if (adapter()->isBattVoltageBelowStopThreshold())
  {
    m_isBacklightOn = false;
  }

  if (0 != m_lcdKeypad)
  {
    m_lcdKeypad->setBackLightOn(m_isBacklightOn);
  }
}

void LintillaMmi::updateDisplay()
{
  lcdBackLightControl();
  if (0 != screenFsm())
  {
    screenFsm()->updateDisplay();
  }
}

//-----------------------------------------------------------------------------
