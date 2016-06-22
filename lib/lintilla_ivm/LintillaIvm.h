/*
 * LintillaIvm.h
 *
 *  Created on: 23.01.2014
 *      Author: niklausd
 */

#ifndef LINTILLAIVM_H_
#define LINTILLAIVM_H_

#include <Ivm.h>

const int wlan_max_length = 32;

/**
 * Lintilla Inventory Management Capabilities (cumulative)
 * - Version 0: DeviceId (addr 0)
 * - Version 1: IVMVersion (addr 1)
 * - Version 2: BatteryVoltageSenseFactor (addr 2,3; float*1000 as unsigned short int, high byte: addr 2, low byte: addr 3)
 * - Version 3: WLAN Access Parameters (addr 4, WLAN_SSID unsigned char [32]; addr 36, WLAN_PASS unsigned char [32])
 */
class LintillaIvm: public Ivm
{
public:
  LintillaIvm();
  virtual ~LintillaIvm();

  void setBattVoltageSenseFactor(float battVoltageSenseFactor);
  float getBattVoltageSenseFactor();

  void setWlanSSID(const char* ssid, int length);
  int getWlanSSID(char* out);

  void setWlanPASS(const char* pass, int length);
  int getWlanPASS(char* out);

private:
  void maintainVersionChange();
  void writeToIvm(const unsigned int addr, const char* in, int length);
  int readFromIvm(const unsigned int addr, char* out, int length);

private:
  /**
   * Current Version of the Lintilla Inventory Management Capabilities.
   */
  const static unsigned char s_currentVersion;

  /**
   *
   */
  const static unsigned int s_ivmBattVoltSensFactAddrHigh;

  const static unsigned int s_ivmWlanSsidAddr;

  const static unsigned int s_ivmWlanPassAddr;

  /**
   *
   */
  const static unsigned int s_ivmBattVoltSensFactAddrLow;

  const static float s_battSensFactorDefault;
  const static float s_battSensFactor1;
  const static float s_battSensFactor2;
  const static float s_battSensFactor3;
  const static float s_battSensFactor4;
  const static float s_battSensFactor5;

private: // forbidden default functions
  LintillaIvm& operator = (const LintillaIvm& );  // assignment operator
  LintillaIvm(const LintillaIvm& src);            // copy constructor
};

#endif /* LINTILLAIVM_H_ */
