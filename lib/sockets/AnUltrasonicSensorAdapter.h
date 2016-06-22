/*
 * AnUltrasonicSensorAdapter.h
 *
 *  Created on: 30.07.2014
 *      Author: niklausd
 */

#ifndef ANULTRASONICSENSORADAPTER_H_
#define ANULTRASONICSENSORADAPTER_H_

#include <UltrasonicSensor.h>

class CmdSequence;

class AnUltrasonicSensorAdapter: public UltrasonicSensorAdapter
{
public:
  AnUltrasonicSensorAdapter(CmdSequence* cmdSequence);
  virtual ~AnUltrasonicSensorAdapter();
  virtual void notifyObstacleDetectionChange(bool isObstacleDetected);
  virtual void startPing();
  virtual unsigned long getEchoTimeMicros();

private:
  CmdSequence* m_cmdSequence;

public:
  static const unsigned int s_triggerPin;
  static const unsigned int s_echoPin;
  static const unsigned int s_echoIrq;

private: // forbidden default functions
  AnUltrasonicSensorAdapter& operator = (const AnUltrasonicSensorAdapter& );  // assignment operator
  AnUltrasonicSensorAdapter(const AnUltrasonicSensorAdapter& src);            // copy constructor
};

#endif /* ANULTRASONICSENSORADAPTER_H_ */
