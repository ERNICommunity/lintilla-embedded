/*
 * Battery.h
 *
 *  Created on: 30.04.2014
 *      Author: niklausd
 */

#ifndef BATTERY_H_
#define BATTERY_H_

class BatteryAdapter;
class BatteryImpl;

class Battery
{
public:
  Battery(BatteryAdapter* adapter = 0);
  virtual ~Battery();

  void attachAdapter(BatteryAdapter* adapter);

  BatteryAdapter* adapter();

private:
  BatteryImpl* m_impl;

private: // forbidden default functions
  Battery& operator = (const Battery& src); // assignment operator
  Battery(const Battery& src);              // copy constructor
};

#endif /* BATTERY_H_ */
