/*
 * ALintillaMmiAdapter.h
 *
 *  Created on: 14.07.2014
 *      Author: niklausd
 */

#ifndef ALINTILLAMMIADAPTER_H_
#define ALINTILLAMMIADAPTER_H_

#include <LintillaMmi.h>

class Battery;
class CmdSequence;
class LintillaIvm;
class UltrasonicSensor;
class Adafruit_CC3000;
class SpeedSensors;

//-----------------------------------------------------------------------------

class ALintillaMmiAdapter: public LintillaMmiAdapter
{
public:
  ALintillaMmiAdapter(Battery* battery, CmdSequence* cmdSeq, LintillaIvm* ivm,
                      UltrasonicSensor* ultrasonicSensorFront, Adafruit_CC3000* cc3000,
                      SpeedSensors* speedSensors);
  virtual ~ALintillaMmiAdapter();

  bool isSeqRunning();
  void startSequence();
  void stopSequence();

  unsigned char getDeviceId();
  void setDeviceId(unsigned char deviceId);

  unsigned char getIvmVersion();

  bool isFrontDistSensLimitExceeded();
  unsigned long getFrontDistanceCM();

  bool isBattVoltageBelowWarnThreshold();
  bool isBattVoltageBelowStopThreshold();
  float getBatteryVoltage();

  bool isWlanConnected();
  uint32_t getCurrentIpAddress();

  long int getLeftWheelSpeed();
  long int getRightWheelSpeed();

  bool isObstacleDetected();

private:
  Battery* m_battery;
  CmdSequence* m_cmdSeq;
  LintillaIvm* m_ivm;
  UltrasonicSensor* m_ultrasonicSensorFront;
  Adafruit_CC3000* m_cc3000;
  SpeedSensors* m_speedSensors;

private: // forbidden default functions
  ALintillaMmiAdapter& operator = (const ALintillaMmiAdapter& );  // assignment operator
  ALintillaMmiAdapter(const ALintillaMmiAdapter& src);            // copy constructor
};

#endif /* ALINTILLAMMIADAPTER_H_ */
