/*
 * Battery.cpp
 *
 *  Created on: 30.04.2014
 *      Author: niklausd
 */

#include "../Battery.h"
#include "BatteryImpl.h"

Battery::Battery(BatteryAdapter* adapter)
: m_impl(new BatteryImpl(adapter))
{ }

Battery::~Battery()
{
  delete m_impl; m_impl = 0;
}

void Battery::attachAdapter(BatteryAdapter* adapter)
{
  if (0 != m_impl)
  {
    m_impl->attachAdapter(adapter);
  }
}

BatteryAdapter* Battery::adapter()
{
  BatteryAdapter* adapter = 0;
  if (0 != m_impl)
  {
    adapter = m_impl->adapter();
  }
  return adapter;
}
