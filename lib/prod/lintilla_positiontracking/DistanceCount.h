/*
 * DistanceCount.h
 *
 *  Created on: 15.07.2014
 *      Author: niklausd
 */

#ifndef DISTANCECOUNT_H_
#define DISTANCECOUNT_H_

class DistanceCount
{
public:
  DistanceCount();
  virtual ~DistanceCount();

  void reset();
  void add(unsigned long int delta);
  unsigned long int cumulativeDistanceCount();

private:
  unsigned long int m_cumulativeDistanceCount;
};

#endif /* DISTANCECOUNT_H_ */
