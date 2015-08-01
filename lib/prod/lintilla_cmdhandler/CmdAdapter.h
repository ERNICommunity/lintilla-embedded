/*
 * CmdAdapter.h
 *
 *  Created on: 11.03.2014
 *      Author: niklausd
 */

#ifndef CMDADAPTER_H_
#define CMDADAPTER_H_

class CmdAdapter
{
public:
  CmdAdapter();
  virtual ~CmdAdapter();

  virtual void stopAction() { }

  virtual void moveForwardAction() { }

  virtual void moveControlledForwardAction() {}

  virtual void moveBackwardAction() { }

  virtual void spinOnPlaceLeftAction(float angle) { }

  virtual void spinOnPlaceRightAction(float angle) { }

private: // forbidden default functions
  CmdAdapter& operator= (const CmdAdapter& src);  // assignment operator
  CmdAdapter(const CmdAdapter& src);              // copy constructor
};

#endif /* CMDADAPTER_H_ */
