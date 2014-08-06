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

  void notifyObstacleDetectionChange(bool isObstacleDetected);

private:
  CmdSequence* m_cmdSequence;

private: // forbidden default functions
  AnUltrasonicSensorAdapter& operator = (const AnUltrasonicSensorAdapter& );  // assignment operator
  AnUltrasonicSensorAdapter(const AnUltrasonicSensorAdapter& src);            // copy constructor
};

#endif /* ANULTRASONICSENSORADAPTER_H_ */
