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

    if ((0 != m_mmi) && (0 != m_mmi->adapter()) && (0 != m_mmi->lcdKeypad()))
    {
      if (!m_mmi->isIvmRobotIdEditMode())
      {
        if ((LcdKeypad::UP_KEY == newKey))
        {
          m_mmi->setBackLightOn(true);
        }
        else if (LcdKeypad::DOWN_KEY == newKey)
        {
          m_mmi->setBackLightOn(false);
        }
      }

      if (!m_mmi->adapter()->isSeqRunning())
      {
        if (!m_mmi->isIvmAccessMode())
        {
          if (LcdKeypad::SELECT_KEY == newKey)
          {
            m_mmi->setIvmAccessMode(true);
          }
          else if (LcdKeypad::LEFT_KEY == newKey)
          {
            m_mmi->adapter()->startSequence();
          }
        }
        else
        {
          if (!m_mmi->isIvmRobotIdEditMode())
          {
            if (LcdKeypad::RIGHT_KEY == newKey)
            {
              m_mmi->setIvmAccessMode(false);
            }
            else if (LcdKeypad::LEFT_KEY == newKey)
            {
              m_mmi->setIvmRobotIdEditMode(true);
            }
          }
          else
          {
            unsigned char robotId = m_mmi->adapter()->getDeviceId();

            if (LcdKeypad::SELECT_KEY == newKey)
            {
              m_mmi->setIvmRobotIdEditMode(false);
            }
            if (LcdKeypad::UP_KEY == newKey)
            {
              robotId++;
              m_mmi->adapter()->setDeviceId(robotId);
            }
            if (LcdKeypad::DOWN_KEY == newKey)
            {
              robotId--;
              m_mmi->adapter()->setDeviceId(robotId);
            }
          }
        }
      }
      else
      {
        if (LcdKeypad::RIGHT_KEY == newKey)
        {
          m_mmi->adapter()->stopSequence();
        }
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
, m_isIvmAccessMode(false)
, m_isIvmRobotIdEditMode(false)
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

void LintillaMmi::attachAdapter(LintillaMmiAdapter* adapter)
{
  m_adapter = adapter;
}

LintillaMmiAdapter* LintillaMmi::adapter()
{
  return m_adapter;
}

bool LintillaMmi::isIvmAccessMode()
{
  return m_isIvmAccessMode;
}

bool LintillaMmi::isIvmRobotIdEditMode()
{
  return m_isIvmRobotIdEditMode;
}

void LintillaMmi::setIvmAccessMode(bool isIvmAccessMode)
{
  m_isIvmAccessMode = isIvmAccessMode;
}

void LintillaMmi::setIvmRobotIdEditMode(bool isIvmRobotIdEditMode)
{
  m_isIvmRobotIdEditMode = isIvmRobotIdEditMode;
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

  if (0 != m_lcdKeypad)
  {
    if (isIvmAccessMode())
    {
      m_lcdKeypad->setCursor(0, 0);

      m_lcdKeypad->print("IVM Data (V.");
      m_lcdKeypad->print(adapter()->getIvmVersion());
      m_lcdKeypad->print(")     ");


      m_lcdKeypad->setCursor(0, 1);

      m_lcdKeypad->print("Robot ID: ");

      if (isIvmRobotIdEditMode() && m_displayBlanking->isSignalBlanked())
      {
        m_lcdKeypad->print("      ");
      }
      else
      {
        m_lcdKeypad->print(adapter()->getDeviceId());
      }
      m_lcdKeypad->print("     ");
    }
    else
    {
      //-------------------------------------------
      // LCD Display Line 1
      //-------------------------------------------
      m_lcdKeypad->setCursor(0, 0);

      m_lcdKeypad->print("Dst:");
      if (m_adapter->isFrontDistSensLimitExceeded())
      {
        m_lcdKeypad->print("infin ");
      }
      else
      {
        unsigned long frontDistanceCM = m_adapter->getFrontDistanceCM();
        m_lcdKeypad->print(frontDistanceCM > 99 ? "" : frontDistanceCM > 9 ? " " : "  ");
        m_lcdKeypad->print(frontDistanceCM);
        m_lcdKeypad->print("cm ");
      }

      if (m_displayBlanking->isSignalBlanked() && (m_adapter->isBattVoltageBelowWarnThreshold()))
      {
        m_lcdKeypad->print("      ");
      }
      else
      {
        m_lcdKeypad->print("B:");
        m_lcdKeypad->print(m_adapter->getBatteryVoltage());
        m_lcdKeypad->print("[V]");
      }

      //-------------------------------------------
      // LCD Display Line 2
      //-------------------------------------------
      m_lcdKeypad->setCursor(0, 1);

      if (!m_adapter->isWlanConnected())
      {
        m_lcdKeypad->print("Connecting WiFi ");
      }
      else if (m_lcdKeypad->isUpKey() || (4 != m_adapter->getDeviceId()))
      {
        uint32_t currentIpAddress = m_adapter->getCurrentIpAddress();
        // IP Address presentation: either on up key pressed or always on robots not having ID = 4
        m_lcdKeypad->print((uint8_t)(currentIpAddress >> 24));
        m_lcdKeypad->print('.');
        m_lcdKeypad->print((uint8_t)(currentIpAddress >> 16));
        m_lcdKeypad->print('.');
        m_lcdKeypad->print((uint8_t)(currentIpAddress >>  8));
        m_lcdKeypad->print('.');
        m_lcdKeypad->print((uint8_t)(currentIpAddress));
        m_lcdKeypad->print("                 ");
      }
      else
      {
        // Speed value presentation (only useful on robot having ID = 4, since only this one has wheel speed sensors)
        int lWSpd = m_adapter->getLeftWheelSpeed();
        int rWspd = m_adapter->getRightWheelSpeed();

        m_lcdKeypad->setCursor(0, 1);
        m_lcdKeypad->print("v ");
        m_lcdKeypad->print("l:");
        m_lcdKeypad->print(lWSpd > 99 ? "" : lWSpd > 9 ? " " : "  ");
        m_lcdKeypad->print(lWSpd);
        m_lcdKeypad->print(" r:");
        m_lcdKeypad->print(rWspd > 99 ? "" : rWspd > 9 ? " " : "  ");
        m_lcdKeypad->print(rWspd);
        m_lcdKeypad->print("   ");
      }
    }
  }
}

//-----------------------------------------------------------------------------
