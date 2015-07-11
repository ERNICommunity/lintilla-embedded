/*
 * CmdHandler.h
 *
 *  Created on: 10.03.2014
 *      Author: niklausd
 */

#ifndef CMDHANDLER_H_
#define CMDHANDLER_H_

class CmdSequence;

class CmdHandler
{
protected:
  CmdHandler(CmdSequence* cmdSeq, unsigned int timeMillis, const char* name);

public:
  virtual ~CmdHandler();

  void setTime(unsigned int millis);
  unsigned int getTime();

  virtual void execute() = 0;

  CmdHandler* next();
  void setNext(CmdHandler* next);

  const char* getName();

  CmdSequence* cmdSequence();

private:
  CmdSequence*  m_cmdSeq;
  unsigned int  m_timeMillis;
  const char*   m_name;
  CmdHandler*   m_next;

private: // forbidden default functions
  CmdHandler& operator = (const CmdHandler& src); // assignment operator
  CmdHandler(const CmdHandler& src);              // copy constructor
};

//-----------------------------------------------------------------------------

class CmdStop : public CmdHandler
{
public:
  CmdStop(CmdSequence* cmdSeq, unsigned int timeMillis);
  virtual ~CmdStop() { }
  virtual void execute();

private: // forbidden default functions
  CmdStop& operator = (const CmdStop& src); // assignment operator
  CmdStop(const CmdStop& src);              // copy constructor
};

//-----------------------------------------------------------------------------

class CmdMoveForward : public CmdHandler
{
public:
  CmdMoveForward(CmdSequence* cmdSeq, unsigned int timeMillis);
  virtual ~CmdMoveForward() { }
  virtual void execute();

private: // forbidden default functions
  CmdMoveForward& operator = (const CmdMoveForward& src); // assignment operator
  CmdMoveForward(const CmdMoveForward& src);              // copy constructor
};

//-----------------------------------------------------------------------------

class CmdMoveControlledForward : public CmdHandler
{
public:
  CmdMoveControlledForward(CmdSequence* cmdSeq, unsigned int timeMillis);
  virtual ~CmdMoveControlledForward() { }
  virtual void execute();

private: // forbidden default functions
  CmdMoveControlledForward& operator = (const CmdMoveControlledForward& src); // assignment operator
  CmdMoveControlledForward(const CmdMoveControlledForward& src);              // copy constructor
};

//-----------------------------------------------------------------------------

class CmdMoveBackward : public CmdHandler
{
public:
  CmdMoveBackward(CmdSequence* cmdSeq, unsigned int timeMillis);
  virtual ~CmdMoveBackward() { }
  virtual void execute();

private: // forbidden default functions
  CmdMoveBackward& operator = (const CmdMoveBackward& src); // assignment operator
  CmdMoveBackward(const CmdMoveBackward& src);              // copy constructor
};

//-----------------------------------------------------------------------------

class CmdSpinOnPlaceLeft : public CmdHandler
{
public:
  CmdSpinOnPlaceLeft(CmdSequence* cmdSeq, unsigned int timeMillis, float angle = 0.0);
  virtual ~CmdSpinOnPlaceLeft() { }
  virtual void execute();

private:
  float m_angle;

private: // forbidden default functions
  CmdSpinOnPlaceLeft& operator = (const CmdSpinOnPlaceLeft& src); // assignment operator
  CmdSpinOnPlaceLeft(const CmdSpinOnPlaceLeft& src);              // copy constructor
};

//-----------------------------------------------------------------------------

class CmdSpinOnPlaceRight : public CmdHandler
{
public:
  CmdSpinOnPlaceRight(CmdSequence* cmdSeq, unsigned int timeMillis, float angle = 0.0);
  virtual ~CmdSpinOnPlaceRight() { }
  virtual void execute();

private:
  float m_angle;

private: // forbidden default functions
  CmdSpinOnPlaceRight& operator = (const CmdSpinOnPlaceRight& src); // assignment operator
  CmdSpinOnPlaceRight(const CmdSpinOnPlaceRight& src);              // copy constructor
};

//-----------------------------------------------------------------------------

#endif /* CMDHANDLER_H_ */
