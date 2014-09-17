/*
 * LintillaMmiScreen.cpp
 *
 *  Created on: 15.09.2014
 *      Author: niklausd
 */

#include <LintillaMmiScreen.h>
#include <LintillaMmi.h>
#include <LcdKeypad.h>
#include <Blanking.h>

LintillaMmiScreen::LintillaMmiScreen(LintillaMmi* mmi)
: m_mmi(mmi)
, m_isEditMode(false)
{ }

LintillaMmiScreen::~LintillaMmiScreen()
{ }

//-----------------------------------------------------------------------------

LintillaMmiHomeScreen::LintillaMmiHomeScreen(LintillaMmi* mmi)
: LintillaMmiScreen(mmi)
{ }

LintillaMmiHomeScreen::~LintillaMmiHomeScreen()
{ }

void LintillaMmiHomeScreen::setCursorUp()
{
  if (0 != mmi())
  {
    mmi()->setBackLightOn(true);
  }
}

void LintillaMmiHomeScreen::setCursorDown()
{
  if (0 != mmi())
  {
    mmi()->setBackLightOn(false);
  }
}

void LintillaMmiHomeScreen::updateDisplay()
{
  if ((0 != mmi()) && (0 != mmi()->lcdKeypad()) && (0 != mmi()->adapter()) && (0 != mmi()->displayBlanking()))
  {
    //-------------------------------------------
    // LCD Display Line 1
    //-------------------------------------------
    mmi()->lcdKeypad()->setCursor(0, 0);

    mmi()->lcdKeypad()->print("Dst:");
    if (mmi()->adapter()->isFrontDistSensLimitExceeded())
    {
      mmi()->lcdKeypad()->print("infin ");
    }
    else
    {
      unsigned long frontDistanceCM = mmi()->adapter()->getFrontDistanceCM();
      mmi()->lcdKeypad()->print(frontDistanceCM > 99 ? "" : frontDistanceCM > 9 ? " " : "  ");
      mmi()->lcdKeypad()->print(frontDistanceCM);
      mmi()->lcdKeypad()->print("cm ");
    }

    if (mmi()->displayBlanking()->isSignalBlanked() && (mmi()->adapter()->isBattVoltageBelowWarnThreshold()))
    {
      mmi()->lcdKeypad()->print("      ");
    }
    else
    {
      mmi()->lcdKeypad()->print("B:");
      mmi()->lcdKeypad()->print(mmi()->adapter()->getBatteryVoltage());
      mmi()->lcdKeypad()->print("[V]");
    }

    //-------------------------------------------
    // LCD Display Line 2
    //-------------------------------------------
    mmi()->lcdKeypad()->setCursor(0, 1);

    if (!mmi()->adapter()->isWlanConnected())
    {
      mmi()->lcdKeypad()->print("Connecting WiFi ");
    }
    else if (mmi()->lcdKeypad()->isUpKey() || (4 != mmi()->adapter()->getDeviceId()))
    {
      uint32_t currentIpAddress = mmi()->adapter()->getCurrentIpAddress();
      // IP Address presentation: either on up key pressed or always on robots not having ID = 4
      mmi()->lcdKeypad()->print((uint8_t)(currentIpAddress >> 24));
      mmi()->lcdKeypad()->print('.');
      mmi()->lcdKeypad()->print((uint8_t)(currentIpAddress >> 16));
      mmi()->lcdKeypad()->print('.');
      mmi()->lcdKeypad()->print((uint8_t)(currentIpAddress >>  8));
      mmi()->lcdKeypad()->print('.');
      mmi()->lcdKeypad()->print((uint8_t)(currentIpAddress));
      mmi()->lcdKeypad()->print("                 ");
    }
    else
    {
      // Speed value presentation (only useful on robot having ID = 4, since only this one has wheel speed sensors)
      int lWSpd = mmi()->adapter()->getLeftWheelSpeed();
      int rWspd = mmi()->adapter()->getRightWheelSpeed();

      mmi()->lcdKeypad()->setCursor(0, 1);
      mmi()->lcdKeypad()->print("v ");
      mmi()->lcdKeypad()->print("l:");
      mmi()->lcdKeypad()->print(lWSpd > 99 ? "" : lWSpd > 9 ? " " : "  ");
      mmi()->lcdKeypad()->print(lWSpd);
      mmi()->lcdKeypad()->print(" r:");
      mmi()->lcdKeypad()->print(rWspd > 99 ? "" : rWspd > 9 ? " " : "  ");
      mmi()->lcdKeypad()->print(rWspd);
      mmi()->lcdKeypad()->print("   ");
    }
  }
}

//-----------------------------------------------------------------------------

LintillaMmiIdScreen::LintillaMmiIdScreen(LintillaMmi* mmi)
: LintillaMmiScreen(mmi)
{ }

LintillaMmiIdScreen::~LintillaMmiIdScreen()
{ }

void LintillaMmiIdScreen::updateDisplay()
{
  if ((0 != mmi()) && (0 != mmi()->lcdKeypad()) && (0 != mmi()->adapter()) && (0 != mmi()->displayBlanking()))
  {
    mmi()->lcdKeypad()->setCursor(0, 0);

    mmi()->lcdKeypad()->print("IVM Data (V.");
    mmi()->lcdKeypad()->print(mmi()->adapter()->getIvmVersion());
    mmi()->lcdKeypad()->print(")     ");


    mmi()->lcdKeypad()->setCursor(0, 1);

    mmi()->lcdKeypad()->print("Robot ID: ");

    if (isEditMode() && mmi()->displayBlanking()->isSignalBlanked())
    {
      mmi()->lcdKeypad()->print("      ");
    }
    else
    {
      mmi()->lcdKeypad()->print(mmi()->adapter()->getDeviceId());
    }
    mmi()->lcdKeypad()->print("     ");
  }
}

void LintillaMmiIdScreen::setCursorUp()
{
  if (0 != mmi())
  {
    if (isEditMode() && (0 != mmi()->adapter()))
    {
      unsigned char id = mmi()->adapter()->getDeviceId();
      mmi()->adapter()->setDeviceId(id + 1);
    }
    else
    {
      mmi()->setBackLightOn(true);
    }
  }
}

void LintillaMmiIdScreen::setCursorDown()
{
  if (0 != mmi())
  {
    if (isEditMode() && (0 != mmi()->adapter()))
    {
      unsigned char id = mmi()->adapter()->getDeviceId();
      mmi()->adapter()->setDeviceId(id - 1);
    }
    else
    {
      mmi()->setBackLightOn(false);
    }
  }
}
