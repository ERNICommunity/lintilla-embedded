/*
 * LintillaBatteryAdapter.cpp
 *
 *  Created on: 22.05.2014
 *      Author: niklausd
 */

#include <avr/power.h>
#include <avr/sleep.h>

#include "Arduino.h"

#include "Battery.h"
#include "LintillaMmi.h"
#include "LintillaIvm.h"
#include "CmdSequence.h"

#include "LintillaBatteryAdapter.h"

const int LintillaBatteryAdapter::s_BATT_SENSE_PIN = A9;

LintillaBatteryAdapter::LintillaBatteryAdapter()
: BatteryAdapter()
, m_battery(0)
, m_lintillaMmi(0)
, m_lintillaIvm(0)
, m_cmdSeq(0)
, m_rawBattSenseValue(600)
{
  pinMode(s_BATT_SENSE_PIN, INPUT);         //ensure Battery Sense pin is an input
  digitalWrite(s_BATT_SENSE_PIN, LOW);      //ensure pullup is off on Battery Sense pin
}

LintillaBatteryAdapter::~LintillaBatteryAdapter()
{ }

void LintillaBatteryAdapter::attachBattery(Battery* battery)
{
  m_battery = battery;
}

void LintillaBatteryAdapter::attachLintillaMmi(LintillaMmi* lintillaMmi)
{
  m_lintillaMmi = lintillaMmi;
}

void LintillaBatteryAdapter::attachLintillaIvm(LintillaIvm* lintillaIvm)
{
  m_lintillaIvm = lintillaIvm;
}

void LintillaBatteryAdapter::attachCmdSequence(CmdSequence* cmdSeq)
{
  m_cmdSeq = cmdSeq;
}

void LintillaBatteryAdapter::incrRawBattSenseValue()
{
  m_rawBattSenseValue++;
  Serial.print("LintillaBatteryAdapter::incrRawBattSenseValue(), m_rawBattSenseValue = ");
  Serial.println(m_rawBattSenseValue);
}

void LintillaBatteryAdapter::decrRawBattSenseValue()
{
  m_rawBattSenseValue--;
  Serial.print("LintillaBatteryAdapter::decrRawBattSenseValue(), m_rawBattSenseValue = ");
  Serial.println(m_rawBattSenseValue);
}

void LintillaBatteryAdapter::debugPrintStateChange()
{
  if (0 != m_battery)
  {
    Serial.print("Battery: state chg: ");
    Serial.print(m_battery->getPreviousStateName());
    Serial.print(" -> ");
    Serial.println(m_battery->getCurrentStateName());
  }
}

void LintillaBatteryAdapter::notifyBattVoltageOk()
{
  Serial.print("notifyBattVoltageOk() - ");
  debugPrintStateChange();
  if (0 != m_lintillaMmi)
  {
    m_lintillaMmi->setBackLightOn(true);
  }
}

void LintillaBatteryAdapter::notifyBattVoltageBelowWarnThreshold()
{
  Serial.print("notifyBattVoltageBelowWarnThreshold() - ");
  debugPrintStateChange();
  if (0 != m_lintillaMmi)
  {
    m_lintillaMmi->setBackLightOn(true);
  }
}

void LintillaBatteryAdapter::notifyBattVoltageBelowStopThreshold()
{
  Serial.print("notifyBattVoltageBelowStopThreshold() - ");
  debugPrintStateChange();
  if (0 != m_cmdSeq)
  {
    m_cmdSeq->stop();
  }
  if (0 != m_lintillaMmi)
  {
    m_lintillaMmi->setBackLightOn(false);
  }
}

void LintillaBatteryAdapter::notifyBattVoltageBelowShutdownThreshold()
{
  Serial.print("notifyBattVoltageBelowShutdownThreshold() - ");
  debugPrintStateChange();
  unsigned char deviceId = 0;
  if (0 != m_lintillaIvm)
  {
    deviceId = m_lintillaIvm->getDeviceId();
  }
  if (0 != m_lintillaMmi)
  {
    m_lintillaMmi->setBackLightOn(false);
  }
  if (0 != deviceId)
  {
    delay(1000);
    sleepNow();
  }
}

float LintillaBatteryAdapter::readBattVoltageSenseFactor()
{
  float battVoltageSenseFactor = 2.0;
  if (0 != m_lintillaIvm)
  {
    battVoltageSenseFactor = m_lintillaIvm->getBattVoltageSenseFactor();
  }
  return battVoltageSenseFactor;
}

unsigned int LintillaBatteryAdapter::readRawBattSenseValue()
{
  m_rawBattSenseValue = analogRead(s_BATT_SENSE_PIN);
  return m_rawBattSenseValue;
}

void LintillaBatteryAdapter::sleepNow()
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and
   * wake up sources are available in which sleep modus.
   *
   * In the avr/sleep.h file, the call names of these sleep modus are to be found:
   *
   * The 5 different modes are:
   * SLEEP_MODE_IDLE -the least power savings
   * SLEEP_MODE_ADC
   * SLEEP_MODE_PWR_SAVE
   * SLEEP_MODE_STANDBY
   * SLEEP_MODE_PWR_DOWN -the most power savings
   *
   * the power reduction management <avr/power.h> is described in
   * http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
   */

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here

  sleep_enable(); // enables the sleep bit in the mcucr register
                  // so sleep is possible. just a safety pin
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  sleep_mode(); // here the device is actually put to sleep!!

  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();  // first thing after waking from sleep:
                    // disable sleep...

  power_all_enable();
}
