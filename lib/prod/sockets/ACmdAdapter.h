/*
 * ACmdAdapter.h
 *
 *  Created on: 29.07.2014
 *      Author: niklausd
 */

#ifndef ACMDADAPTER_H_
#define ACMDADAPTER_H_

#include <CmdAdapter.h>

class Traction;
class DbgTrace_Port;

class ACmdAdapter: public CmdAdapter
{
public:
  ACmdAdapter(Traction* traction);
  virtual ~ACmdAdapter();

  void stopAction();
  void moveForwardAction();
  void moveBackwardAction();
  void spinOnPlaceLeftAction(float angle);
  void spinOnPlaceRightAction(float angle);

private:
  Traction* m_traction;
  DbgTrace_Port* m_trPort;

private: // forbidden default functions
  ACmdAdapter& operator = (const ACmdAdapter& );  // assignment operator
  ACmdAdapter(const ACmdAdapter& src);            // copy constructor
};

#endif /* ACMDADAPTER_H_ */
