/*
 * AnUltrasonicSensorAdapter.cpp
 *
 *  Created on: 30.07.2014
 *      Author: niklausd
 */

#include "Arduino.h"
#include "CmdSequence.h"
#include <AnUltrasonicSensorAdapter.h>

const unsigned int AnUltrasonicSensorAdapter::s_triggerPin = 34;
const unsigned int AnUltrasonicSensorAdapter::s_echoPin    = 19;
const unsigned int AnUltrasonicSensorAdapter::s_echoIrq    =  4;

volatile unsigned long echoStartTimeMicros       = 0;
volatile unsigned long echoEndTimeMicros         = 0;
volatile unsigned long echoTimeMicros = 0;

void ultrasonicSensorFrontEchoIsr()
{
  noInterrupts();
  if (digitalRead(AnUltrasonicSensorAdapter::s_echoPin))
  {
    // on rising edge
    echoStartTimeMicros = micros();
    echoEndTimeMicros   = 0;
  }
  else
  {
    // on falling edge
    echoEndTimeMicros = micros();
    echoTimeMicros = echoEndTimeMicros - echoStartTimeMicros;
  }
  interrupts();
}

AnUltrasonicSensorAdapter::AnUltrasonicSensorAdapter(CmdSequence* cmdSequence)
: m_cmdSequence(cmdSequence)
{
  pinMode(s_echoPin, INPUT);
  digitalWrite(s_echoPin, HIGH);
  pinMode(s_triggerPin, OUTPUT);
  attachInterrupt(s_echoIrq, ultrasonicSensorFrontEchoIsr, CHANGE);
}

AnUltrasonicSensorAdapter::~AnUltrasonicSensorAdapter()
{
  detachInterrupt(s_echoIrq);
  pinMode(s_triggerPin, INPUT);
  digitalWrite(s_echoPin, LOW);
}

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

void AnUltrasonicSensorAdapter::startPing()
{
  digitalWrite(s_triggerPin, LOW);  // Send low pulse
  delayMicroseconds(2);             // Wait for 2 microseconds
  digitalWrite(s_triggerPin, HIGH); // Send high pulse
  delayMicroseconds(20);            // Wait for 20 microseconds
  digitalWrite(s_triggerPin, LOW);  // Holdoff
}

unsigned long AnUltrasonicSensorAdapter::getEchoTimeMicros()
{
  return echoTimeMicros;
}
