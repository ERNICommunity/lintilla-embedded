/*
 * DistanceCount.cpp
 *
 *  Created on: 15.07.2014
 *      Author: niklausd
 */

#include "DistanceCount.h"

DistanceCount::DistanceCount()
: m_cumulativeDistanceCount(0)
{ }

DistanceCount::~DistanceCount()
{ }

void DistanceCount::reset()
{
  m_cumulativeDistanceCount = 0;
}

void DistanceCount::add(unsigned long int delta)
{
  m_cumulativeDistanceCount += delta;
}

unsigned long int DistanceCount::cumulativeDistanceCount()
{
  return m_cumulativeDistanceCount;
}
