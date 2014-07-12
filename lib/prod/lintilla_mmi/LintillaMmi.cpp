/*
 * LintillaMMI.cpp
 *
 *  Created on: 12.07.2014
 *      Author: niklausd
 */

#include "LcdKeypad.h"

#include "LintillaMmi.h"

class MyLcdKeypadAdapter : public LcdKeypadAdapter
{
public:
  MyLcdKeypadAdapter(LintillaMmi* mmi)
  : m_mmi(mmi)
  { }

private:
  void handleKeyChanged(LcdKeypad::Key newKey)
  {
    if (0 != m_mmi)
    {
      LcdKeypad* lcdKeypad = m_mmi->lcdKeypad();
      if (0 != lcdKeypad)
      {
        if (LcdKeypad::UP_KEY == newKey)
        {
          if (!m_mmi->isIvmAccessMode())
          {
            lcdKeypad->setBackLightOn(true);
          }
        }
        if (LcdKeypad::DOWN_KEY == newKey)
        {
          if (!m_mmi->isIvmAccessMode())
          {
            lcdKeypad->setBackLightOn(false);
          }
        }
      }
      if (LcdKeypad::SELECT_KEY == newKey)
      {
        if (m_mmi->isIvmRobotIdEditMode())
        {
          m_mmi->setIvmRobotIdEditMode(false);
        }
        else if (!m_mmi->isIvmAccessMode())
        {
          m_mmi->setIvmAccessMode(true);
        }
      }
      if (LcdKeypad::LEFT_KEY == newKey)
      {
        if (m_mmi->isIvmAccessMode())
        {
          m_mmi->setIvmRobotIdEditMode(true);
        }
      }
      if (LcdKeypad::RIGHT_KEY == newKey)
      {
        if (m_mmi->isIvmAccessMode())
        {
          m_mmi->setIvmAccessMode(false);
        }
      }
    }
  }

private:
  LintillaMmi* m_mmi;
};

LintillaMmi::LintillaMmi()
: m_lcdKeypad(new LcdKeypad())
, m_isIvmAccessMode(false)
, m_isIvmRobotIdEditMode(false)
{
  m_lcdKeypad->attachAdapter(new MyLcdKeypadAdapter(this));
}

LintillaMmi::~LintillaMmi()
{ }

LcdKeypad* LintillaMmi::lcdKeypad()
{
  return m_lcdKeypad;
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
