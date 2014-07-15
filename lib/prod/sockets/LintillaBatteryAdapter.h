/*
 * LintillaBatteryAdapter.h
 *
 *  Created on: 22.05.2014
 *      Author: niklausd
 */

#ifndef LINTILLABATTERYADAPTER_H_
#define LINTILLABATTERYADAPTER_H_

#include <Battery.h>

class Battery;
class LintillaMmi;
class LintillaIvm;

class LintillaBatteryAdapter: public BatteryAdapter
{
public:
  LintillaBatteryAdapter();
  virtual ~LintillaBatteryAdapter();

  void attachBattery(Battery* battery);
  void attachLintillaMmi(LintillaMmi* lintillaMmi);
  void attachLintillaIvm(LintillaIvm* lintillaIvm);
  void attachCmdSequence(CmdSequence* cmdSeq);

public:
  void incrRawBattSenseValue();
  void decrRawBattSenseValue();

private:
  void debugPrintStateChange();

public:
  void notifyBattVoltageOk();
  void notifyBattVoltageBelowWarnThreshold();
  void notifyBattVoltageBelowStopThreshold();
  void notifyBattVoltageBelowShutdownThreshold();
  float readBattVoltageSenseFactor();
  unsigned int readRawBattSenseValue();

private:
  void sleepNow();

private:
  Battery* m_battery;
  LintillaMmi* m_lintillaMmi;
  LintillaIvm* m_lintillaIvm;
  CmdSequence* m_cmdSeq;
  unsigned int m_rawBattSenseValue;

  static const int s_BATT_SENSE_PIN;


private: // forbidden default functions
  LintillaBatteryAdapter& operator= (const LintillaBatteryAdapter& src);  // assignment operator
  LintillaBatteryAdapter(const LintillaBatteryAdapter& src);              // copy constructor
};

#endif /* LINTILLABATTERYADAPTER_H_ */
