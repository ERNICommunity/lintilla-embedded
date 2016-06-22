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
#include "DbgCliTopic.h"
#include "LintillaMmiDbgCmd.h"
#include "DbgTraceContext.h"
#include "DbgTracePort.h"

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

class MmiLcdKeypadAdapter : public LcdKeypadAdapter
{
public:
  MmiLcdKeypadAdapter(LintillaMmi* mmi)
  : m_mmi(mmi)
  , m_trPort(new DbgTrace_Port("mmiLcdKey", DbgTrace_Level::info))
  { }

private:
  LintillaMmi* m_mmi;
  DbgTrace_Port* m_trPort;

public:
  void handleKeyChanged(LcdKeypad::Key newKey)
  {
    TR_PRINT_STR(m_trPort, DbgTrace_Level::debug, "handleKeyChanged(), newKey: ");
    TR_PRINT_STR(m_trPort, DbgTrace_Level::debug, (LcdKeypad::NO_KEY == newKey)     ? "NO_KEY"     :
                                                  (LcdKeypad::SELECT_KEY == newKey) ? "SELECT_KEY" :
                                                  (LcdKeypad::LEFT_KEY == newKey)   ? "LEFT_KEY"   :
                                                  (LcdKeypad::UP_KEY == newKey)     ? "UP_KEY"     :
                                                  (LcdKeypad::DOWN_KEY == newKey)   ? "DOWN_KEY"   :
                                                  (LcdKeypad::RIGHT_KEY == newKey)  ? "RIGHT_KEY"  : "OOPS!! Invalid value!!");

    if ((0 != m_mmi) && (0 != m_mmi->screenFsm()))
    {
      switch (newKey)
      {
        case LcdKeypad::SELECT_KEY : m_mmi->screenFsm()->select(); break;
        case LcdKeypad::LEFT_KEY   : m_mmi->screenFsm()->left();   break;
        case LcdKeypad::RIGHT_KEY  : m_mmi->screenFsm()->right();  break;
        case LcdKeypad::UP_KEY     : m_mmi->screenFsm()->up();     break;
        case LcdKeypad::DOWN_KEY   : m_mmi->screenFsm()->down();   break;
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
, m_isBacklightOn(false)
, m_newIsBacklightOn(true)
, m_dbgCliTopic(new DbgCli_Topic(DbgCli_Node::RootNode(), "mmi", "MMI Node."))
, m_dbgCliCmd_Key(new LintillaMmiDbgCmd_Key(this))
{
  if (0 != m_lcdKeypad)
  {
    m_lcdKeypad->attachAdapter(new MmiLcdKeypadAdapter(this));
  }
}

LintillaMmi::~LintillaMmi()
{
  delete m_dbgCliCmd_Key;
  m_dbgCliCmd_Key = 0;

  delete m_dbgCliTopic;
  m_dbgCliTopic = 0;

  delete m_lcdKeypad->adapter();
  m_lcdKeypad->attachAdapter(0);

  delete m_screenFsm;
  m_screenFsm = 0;
  
  delete m_displayTimer->adapter();
  m_displayTimer->attachAdapter(0);

  delete m_displayTimer;
  m_displayTimer = 0;

  delete m_displayBlanking;
  m_displayBlanking = 0;

  delete m_lcdKeypad;
  m_lcdKeypad = 0;
}

void LintillaMmi::attachAdapter(LintillaMmiAdapter* adapter)
{
  m_adapter = adapter;
}

LintillaMmiAdapter* LintillaMmi::adapter()
{
  return m_adapter;
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

bool LintillaMmi::isBacklightOn()
{
  return m_isBacklightOn;
}

void LintillaMmi::setBackLightOn(bool isBacklightOn)
{
  m_newIsBacklightOn = isBacklightOn;
  if (m_newIsBacklightOn != m_isBacklightOn)
  {
    lcdBackLightControl();
  }
}

void LintillaMmi::lcdBackLightControl()
{
  if (adapter()->isBattVoltageBelowStopThreshold())
  {
    m_newIsBacklightOn = false;
  }

  if (0 != m_lcdKeypad)
  {
    if (m_newIsBacklightOn != m_isBacklightOn)
    {
      m_isBacklightOn = m_newIsBacklightOn;
      m_lcdKeypad->setBackLightOn(m_isBacklightOn);
    }
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
