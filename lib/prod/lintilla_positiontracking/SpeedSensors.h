/*
 * SpeedSensors.h
 *
 *  Created on: 26.07.2014
 *      Author: niklausd
 */

#ifndef SPEEDSENSORS_H_
#define SPEEDSENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif
void countLeftSpeedSensor();
void countRightSpeedSensor();
#ifdef __cplusplus
}
#endif

class Timer;
class DistanceCount;

class SpeedSensors
{
public:
  SpeedSensors();
  virtual ~SpeedSensors();

  DistanceCount* lDistCount();
  DistanceCount* rDistCount();

private:
  Timer* m_speedSensorReadTimer;
  DistanceCount* m_lDistCount;
  DistanceCount* m_rDistCount;

private: // forbidden default functions
  SpeedSensors& operator = (const SpeedSensors& src); // assignment operator
  SpeedSensors(const SpeedSensors& src);              // copy constructor
};

#endif /* SPEEDSENSORS_H_ */
