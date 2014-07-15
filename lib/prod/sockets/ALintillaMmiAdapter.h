/*
 * ALintillaMmiAdapter.h
 *
 *  Created on: 14.07.2014
 *      Author: niklausd
 */

#ifndef ALINTILLAMMIADAPTER_H_
#define ALINTILLAMMIADAPTER_H_

#include <LintillaMmi.h>

class CmdSequence;
class LintillaIvm;
class UltrasonicSensor;
class Adafruit_CC3000;
class DistanceCount;

//-----------------------------------------------------------------------------

class ALintillaMmiAdapter: public LintillaMmiAdapter
{
public:
  ALintillaMmiAdapter(CmdSequence* cmdSeq, LintillaIvm* ivm,
                      UltrasonicSensor* ultrasonicSensorFront, Adafruit_CC3000* cc3000,
                      DistanceCount* lDistCount, DistanceCount* rDistCount);
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
  float getBatteryVoltage();

  bool isWlanConnected();
  unsigned int getCurrentIpAddress();

  long int getLeftWheelSpeed();
  long int getRightWheelSpeed();

  bool isObstacleDetected();

private:
  CmdSequence* m_cmdSeq;
  LintillaIvm* m_ivm;
  UltrasonicSensor* m_ultrasonicSensorFront;
  Adafruit_CC3000* m_cc3000;
  DistanceCount* m_lDistCount;
  DistanceCount* m_rDistCount;

private: // forbidden default functions
  ALintillaMmiAdapter& operator = (const ALintillaMmiAdapter& );  // assignment operator
  ALintillaMmiAdapter(const ALintillaMmiAdapter& src);            // copy constructor
};

#endif /* ALINTILLAMMIADAPTER_H_ */
