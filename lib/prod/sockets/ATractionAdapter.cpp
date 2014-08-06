/*
 * ATractionAdapter.cpp
 *
 *  Created on: 30.07.2014
 *      Author: niklausd
 */

#include <ATractionAdapter.h>

ATractionAdapter::ATractionAdapter(UltrasonicSensor* ultrasonicSensor)
: m_ultrasonicSensor(ultrasonicSensor)
{
//  m_ultrasonicSensor->setIsObstacleDetectionActive(true);
}

ATractionAdapter::~ATractionAdapter()
{ }

void ATractionAdapter::notifyDirectionChange(bool isForward)
{
  if (0 != m_ultrasonicSensor)
  {
    m_ultrasonicSensor->setIsObstacleDetectionActive(isForward);
  }
}
