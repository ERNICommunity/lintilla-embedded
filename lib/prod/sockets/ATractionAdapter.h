/*
 * ATractionAdapter.h
 *
 *  Created on: 30.07.2014
 *      Author: niklausd
 */

#ifndef ATRACTIONADAPTER_H_
#define ATRACTIONADAPTER_H_

#include "UltrasonicSensor.h"
#include <Traction.h>

class ATractionAdapter: public TractionAdapter
{
public:
  ATractionAdapter(UltrasonicSensor* ultrasonicSensor);
  virtual ~ATractionAdapter();

  void notifyDirectionChange(bool isForward);

private:
  UltrasonicSensor* m_ultrasonicSensor;

private: // forbidden default functions
  ATractionAdapter& operator= (const ATractionAdapter& src);  // assignment operator
  ATractionAdapter(const ATractionAdapter& src);              // copy constructor
};

#endif /* ATRACTIONADAPTER_H_ */
