/*
 * BatteryImpl.cpp
 *
 *  Created on: 30.04.2014
 *      Author: niklausd
 */

#include "BatteryImpl.h"
#include "../BatteryAdapter.h"

BatteryImpl::BatteryImpl(BatteryAdapter* adapter)
: m_adapter(adapter)
{ }

BatteryImpl::~BatteryImpl()
{ }


void BatteryImpl::attachAdapter(BatteryAdapter* adapter)
{
  m_adapter = adapter;
}

BatteryAdapter* BatteryImpl::adapter()
{
  return m_adapter;
}
