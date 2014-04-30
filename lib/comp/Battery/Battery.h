/*
 * Battery.h
 *
 *  Created on: 30.04.2014
 *      Author: niklausd
 */

#ifndef BATTERY_H_
#define BATTERY_H_

class Battery
{
public:
  Battery();
  virtual ~Battery();

private: // forbidden default functions
  Battery& operator = (const Battery& src); // assignment operator
  Battery(const Battery& src);              // copy constructor
};

#endif /* BATTERY_H_ */
