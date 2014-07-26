/*
 * Cmd.h
 *
 *  Created on: 10.03.2014
 *      Author: niklausd
 */

#ifndef CMD_H_
#define CMD_H_

class CmdSequence;

class Cmd
{
protected:
  Cmd(CmdSequence* cmdSeq, unsigned int timeMillis, const char* name);

public:
  virtual ~Cmd();

  void setTime(unsigned int millis);
  unsigned int getTime();

  virtual void execute() = 0;

  Cmd* next();
  void setNext(Cmd* next);

  const char* getName();

  CmdSequence* cmdSequence();

private:
  CmdSequence*  m_cmdSeq;
  unsigned int  m_timeMillis;
  const char*   m_name;
  Cmd*          m_next;

private: // forbidden default functions
  Cmd& operator = (const Cmd& src); // assignment operator
  Cmd(const Cmd& src);              // copy constructor
};

//-----------------------------------------------------------------------------

class CmdStop : public Cmd
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

class CmdMoveForward : public Cmd
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

class CmdMoveBackward : public Cmd
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

class CmdSpinOnPlaceLeft : public Cmd
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

class CmdSpinOnPlaceRight : public Cmd
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

#endif /* CMD_H_ */
