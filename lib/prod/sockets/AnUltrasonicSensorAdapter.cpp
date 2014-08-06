/*
 * AnUltrasonicSensorAdapter.cpp
 *
 *  Created on: 30.07.2014
 *      Author: niklausd
 */

#include "Arduino.h"
#include "CmdSequence.h"
#include <AnUltrasonicSensorAdapter.h>

AnUltrasonicSensorAdapter::AnUltrasonicSensorAdapter(CmdSequence* cmdSequence)
: m_cmdSequence(cmdSequence)
{ }

AnUltrasonicSensorAdapter::~AnUltrasonicSensorAdapter()
{ }

void AnUltrasonicSensorAdapter::notifyObstacleDetectionChange(bool isObstacleDetected)
{
  Serial.print("notifyObstacleDetectionChange(), isObstacleDetected: ");
  Serial.println(isObstacleDetected ? "true" : "false");
  if ((0 != m_cmdSequence) && isObstacleDetected)
  {
    m_cmdSequence->stop();
    Serial.println("notifyObstacleDetectionChange(): m_cmdSequence->stop()");
  }
}

