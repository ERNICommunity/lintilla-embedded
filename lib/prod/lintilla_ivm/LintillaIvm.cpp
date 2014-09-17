/*
 * LintillaIvm.cpp
 *
 *  Created on: 23.01.2014
 *      Author: niklausd
 */

#include "Ivm.h"
#include "IvmSerialEeprom.h"
#include "LintillaIvm.h"

#include "string.h"


const char * wlan_ssid= "LintillaNet";
const char * wlan_pass= "AnswerIs42";

const float LintillaIvm::s_battSensFactorDefault = 2.000;
const float LintillaIvm::s_battSensFactor1       = 2.430;
const float LintillaIvm::s_battSensFactor2       = 2.450;
const float LintillaIvm::s_battSensFactor3       = 2.530;
const float LintillaIvm::s_battSensFactor4       = 2.000;
const float LintillaIvm::s_battSensFactor5       = 2.285;


const unsigned char LintillaIvm::s_currentVersion = 3;

const unsigned int  LintillaIvm::s_ivmBattVoltSensFactAddrHigh = 2;
const unsigned int  LintillaIvm::s_ivmBattVoltSensFactAddrLow  = 3;
const unsigned int  LintillaIvm::s_ivmWlanSsidAddr = 4;
const unsigned int  LintillaIvm::s_ivmWlanPassAddr = 36;

LintillaIvm::LintillaIvm()
: Ivm(new IvmSerialEeprom())
{
  maintainVersionChange();
}

LintillaIvm::~LintillaIvm()
{
  delete getIvmMemory();
  setIvmMemory(0);
}

void LintillaIvm::setBattVoltageSenseFactor(float battVoltageSenseFactor)
{
  unsigned int battSensFact = 0;
  if (0.0 > battVoltageSenseFactor)
  {
    battSensFact = 0;
  }
  else if (65.535 <= battVoltageSenseFactor)
  {
    battSensFact = 0xFFFF;
  }
  else
  {
    battSensFact = static_cast<unsigned int>(1000.0 * battVoltageSenseFactor);
  }

  IF_IvmMemory* ivmMemory = getIvmMemory();
  if (0 != ivmMemory)
  {
    ivmMemory->write(s_ivmBattVoltSensFactAddrHigh, (battSensFact & 0xFF00) >> 8);
    ivmMemory->write(s_ivmBattVoltSensFactAddrLow,  (battSensFact & 0x00FF)     );
  }
}

float LintillaIvm::getBattVoltageSenseFactor()
{
  float battVoltageSenseFactor = s_battSensFactorDefault;
  unsigned int battSensFact = 0;
  IF_IvmMemory* ivmMemory = getIvmMemory();
  if (0 != ivmMemory)
  {
    battSensFact  = ivmMemory->read(s_ivmBattVoltSensFactAddrHigh) << 8;
    battSensFact += ivmMemory->read(s_ivmBattVoltSensFactAddrLow);
    battVoltageSenseFactor = static_cast<float>(battSensFact) / 1000.0;
  }
  return battVoltageSenseFactor;
}

void LintillaIvm::setWlanSSID(const char* ssid, int length)
{
  writeToIvm(s_ivmWlanSsidAddr, ssid, (length > wlan_max_length) ? wlan_max_length : length);
}

int LintillaIvm::getWlanSSID(char* out)
{
  readFromIvm(s_ivmWlanSsidAddr, out, wlan_max_length);
}

void LintillaIvm::setWlanPASS(const char* pass, int length)
{
  writeToIvm(s_ivmWlanPassAddr, pass, (length > wlan_max_length) ? wlan_max_length : length);
}

int LintillaIvm::getWlanPASS(char* out)
{
  readFromIvm(s_ivmWlanPassAddr, out, wlan_max_length);
}

void LintillaIvm::maintainVersionChange()
{
  unsigned char ivmVersion = getIvmVersion();

  unsigned char deviceId   = getDeviceId();
  if (255 == ivmVersion || 0 == ivmVersion)
  {
    // Assume uninitialized IVM memory, bring up to current version
    setIvmVersion(s_currentVersion);

    // Initialize Device ID
    if (255 == deviceId)
    {
      // Assume version uninitialized, initialize to zero.
      setDeviceId(0);
      deviceId = 0;
    }
  }

  if ((s_currentVersion - 1) >= ivmVersion)
  {
    // IVMVersion not present, write current version
    setIvmVersion(s_currentVersion);

    if (2 > ivmVersion)
    {
      // initialize Version 2 feature: BatteryVoltageSenseFactor, set default value
      float initialBattSensFactor = s_battSensFactorDefault;
      switch (deviceId)
      {
        case 1: initialBattSensFactor = s_battSensFactor1; break;
        case 2: initialBattSensFactor = s_battSensFactor2; break;
        case 3: initialBattSensFactor = s_battSensFactor3; break;
        case 4: initialBattSensFactor = s_battSensFactor4; break;
        case 5: initialBattSensFactor = s_battSensFactor5; break;
        default:
          break;
      }
      setBattVoltageSenseFactor(initialBattSensFactor);
    }

    if (3 > ivmVersion)
    {
      setWlanSSID(wlan_ssid, strlen(wlan_ssid) + 1);
      setWlanPASS(wlan_pass, strlen(wlan_pass) + 1);
    }
  }
}

void LintillaIvm::writeToIvm(const unsigned int addr, const char* in,
    int length)
{
  IF_IvmMemory* ivmMemory = getIvmMemory();
  if (0 != ivmMemory)
  {
    for(unsigned int i = 0; i < length; ++i)
    {
      ivmMemory->write(addr+i, in[i]);
    }
  }
}

int LintillaIvm::readFromIvm(const unsigned int addr, char* out,
    int length)
{
  unsigned int i = 0;
  IF_IvmMemory* ivmMemory = getIvmMemory();
  if (0 != ivmMemory)
  {
    for(i = 0; i < length; ++i)
    {
      out[i] = ivmMemory->read(addr+i);
    }
  }

  return i;
}
