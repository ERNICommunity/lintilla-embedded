/*
 * BatteryImpl.h
 *
 *  Created on: 30.04.2014
 *      Author: niklausd
 */

#ifndef BATTERYIMPL_H_
#define BATTERYIMPL_H_

class BatteryAdapter;

class BatteryImpl
{
public:
  BatteryImpl(BatteryAdapter* adapter);
  virtual ~BatteryImpl();

  void attachAdapter(BatteryAdapter* adapter);

  BatteryAdapter* adapter();

private:
  BatteryAdapter* m_adapter;

private: // forbidden default functions
  BatteryImpl& operator = (const BatteryImpl& src); // assignment operator
  BatteryImpl(const BatteryImpl& src);              // copy constructor
};

#endif /* BATTERYIMPL_H_ */
