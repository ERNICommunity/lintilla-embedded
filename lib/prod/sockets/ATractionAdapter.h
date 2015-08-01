/*
 * ATractionAdapter.h
 *
 *  Created on: 30.07.2014
 *      Author: niklausd
 */

#ifndef ATRACTIONADAPTER_H_
#define ATRACTIONADAPTER_H_

#include "UltrasonicSensor.h"
#include "FreeSixIMU.h"
#include "PID_v1.h"
#include <Traction.h>

class ATractionAdapter: public TractionAdapter
{
public:
  ATractionAdapter(UltrasonicSensor* ultrasonicSensor, FreeSixIMU* sixImuSensor);
  virtual ~ATractionAdapter();

  float getYawAngle();
  void notifyDirectionChange(bool isForward);
  void setTargetAngle(double targetAngle);
  float computeSpeedDiff();

private:
  UltrasonicSensor* m_ultrasonicSensor;
  FreeSixIMU* m_sixImuSensor;
  PID* m_pid;

private:
  double m_yawActualAngle;
  double m_yawTargetAngle;
  double m_calculatedSpeedDiff;
  double m_Kp;
  double m_Ki;
  double m_Kd;

private: // forbidden default functions
  ATractionAdapter& operator= (const ATractionAdapter& src);  // assignment operator
  ATractionAdapter(const ATractionAdapter& src);              // copy constructor
};

#endif /* ATRACTIONADAPTER_H_ */
