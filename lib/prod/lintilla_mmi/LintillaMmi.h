/*
 * LintillaMMI.h
 *
 *  Created on: 12.07.2014
 *      Author: niklausd
 */

#ifndef LINTILLAMMI_H_
#define LINTILLAMMI_H_

#include <stdint.h>

class LcdKeypad;
class Blanking;
class Timer;
class LintillaMmiScreenFsm;

//-----------------------------------------------------------------------------

class LintillaMmiAdapter
{
public:
  virtual ~LintillaMmiAdapter() { }
  
  virtual bool isSeqRunning() = 0;
  virtual void startSequence() = 0;
  virtual void stopSequence() = 0;

  virtual void setWlanSSID(const char* ssid, int length) = 0;
  virtual int getWlanSSID(char* out) = 0;

  virtual void setWlanPASS(const char* pass, int length) = 0;
  virtual int getWlanPASS(char* out) = 0;

  virtual unsigned char getDeviceId() = 0;
  virtual void setDeviceId(unsigned char deviceId) = 0;

  virtual unsigned char getIvmVersion() = 0;

  virtual bool isFrontDistSensLimitExceeded() = 0;
  virtual unsigned long getFrontDistanceCM() = 0;

  virtual bool isBattVoltageBelowWarnThreshold() = 0;
  virtual bool isBattVoltageBelowStopThreshold() = 0;
  virtual float getBatteryVoltage() = 0;

  virtual bool isWlanConnected() = 0;
  virtual uint32_t getCurrentIpAddress() = 0;

  virtual long int getLeftWheelSpeed() = 0;
  virtual long int getRightWheelSpeed() = 0;

protected:
  LintillaMmiAdapter() { }

private: // forbidden default functions
  LintillaMmiAdapter& operator = (const LintillaMmiAdapter& src);  // assignment operator
  LintillaMmiAdapter(const LintillaMmiAdapter& src);               // copy constructor
};

//-----------------------------------------------------------------------------

class LintillaMmi
{
public:
  LintillaMmi(LintillaMmiAdapter* adapter);
  virtual ~LintillaMmi();

  void attachAdapter(LintillaMmiAdapter* adapter);
  LintillaMmiAdapter* adapter();

  LcdKeypad* lcdKeypad();
  Blanking* displayBlanking();
  LintillaMmiScreenFsm* screenFsm();

  bool isBacklightOn();
  void setBackLightOn(bool isBacklightOn);
  void lcdBackLightControl();

  void updateDisplay();

private:
  LcdKeypad* m_lcdKeypad;
  Blanking* m_displayBlanking;
  LintillaMmiAdapter* m_adapter;
  Timer* m_displayTimer;
  LintillaMmiScreenFsm* m_screenFsm;

  bool m_isBacklightOn;

private: // forbidden default functions
  LintillaMmi& operator = (const LintillaMmi& src);  // assignment operator
  LintillaMmi(const LintillaMmi& src);               // copy constructor
};

#endif /* LINTILLAMMI_H_ */
