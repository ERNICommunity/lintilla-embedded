/*
 * LintillaMMI.h
 *
 *  Created on: 12.07.2014
 *      Author: niklausd
 */

#ifndef LINTILLAMMI_H_
#define LINTILLAMMI_H_

class LcdKeypad;
class Blanking;
class Timer;

//-----------------------------------------------------------------------------

class LintillaMmiAdapter
{
public:
  virtual bool isSeqRunning() = 0;
  virtual void startSequence() = 0;
  virtual void stopSequence() = 0;

  virtual unsigned char getDeviceId() = 0;
  virtual void setDeviceId(unsigned char deviceId) = 0;

  virtual unsigned char getIvmVersion() = 0;

  virtual bool isFrontDistSensLimitExceeded() = 0;
  virtual unsigned long getFrontDistanceCM() = 0;

  virtual bool isBattVoltageBelowWarnThreshold() = 0;
  virtual float getBatteryVoltage() = 0;

  virtual bool isWlanConnected() = 0;
  virtual uint32_t getCurrentIpAddress() = 0;

  virtual long int getLeftWheelSpeed() = 0;
  virtual long int getRightWheelSpeed() = 0;

  virtual ~LintillaMmiAdapter() { }

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

  bool isIvmAccessMode();
  bool isIvmRobotIdEditMode();

  void setIvmAccessMode(bool isIvmAccessMode);
  void setIvmRobotIdEditMode(bool isIvmRobotIdEditMode);

  void updateDisplay();

private:
  LcdKeypad* m_lcdKeypad;
  Blanking* m_displayBlanking;
  LintillaMmiAdapter* m_adapter;
  Timer* m_displayTimer;


  // Display Menu states
  bool m_isIvmAccessMode;
  bool m_isIvmRobotIdEditMode;

private: // forbidden default functions
  LintillaMmi& operator = (const LintillaMmi& src);  // assignment operator
  LintillaMmi(const LintillaMmi& src);               // copy constructor
};

#endif /* LINTILLAMMI_H_ */
