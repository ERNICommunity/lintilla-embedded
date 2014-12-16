/*
 * LintillaMmiScreenState.h
 *
 *  Created on: 15.09.2014
 *      Author: niklausd
 */

#ifndef LINTILLAMMISCREENSTATE_H_
#define LINTILLAMMISCREENSTATE_H_

class LintillaMmiScreenFsm;
class LintillaMmiScreen;

class LintillaMmiScreenState
{
protected:
  LintillaMmiScreenState();
public:
  virtual ~LintillaMmiScreenState();

public:
  virtual void attachScreen(LintillaMmiScreen* screen) { m_screen = screen; }

  virtual void select(LintillaMmiScreenFsm* fsm) { };
  virtual void left(LintillaMmiScreenFsm* fsm)   { };
  virtual void right(LintillaMmiScreenFsm* fsm)  { };
  virtual void up(LintillaMmiScreenFsm* fsm)     { };
  virtual void down(LintillaMmiScreenFsm* fsm)   { };
  virtual void entry(LintillaMmiScreenFsm* fsm)  { };

  void updateDisplay();

  virtual const char* toString() = 0;

  LintillaMmiScreen* screen() { return m_screen; }

private:
  LintillaMmiScreen* m_screen;

private: // forbidden default functions
  LintillaMmiScreenState& operator = (const LintillaMmiScreenState& src);  // assignment operator
  LintillaMmiScreenState(const LintillaMmiScreenState& src);               // copy constructor
};

//-----------------------------------------------------------------------------

class LintillaMmiHomeScreenState : public LintillaMmiScreenState
{
private:
  LintillaMmiHomeScreenState() { }

public:
  static LintillaMmiScreenState* Instance();

  virtual ~LintillaMmiHomeScreenState() { }

  virtual void select(LintillaMmiScreenFsm* fsm);
  virtual void left(LintillaMmiScreenFsm* fsm)  ;
  virtual void right(LintillaMmiScreenFsm* fsm) ;
  virtual void up(LintillaMmiScreenFsm* fsm)    ;
  virtual void down(LintillaMmiScreenFsm* fsm)  ;
  virtual void entry(LintillaMmiScreenFsm* fsm) ;

  virtual const char* toString();

private:
  static LintillaMmiScreenState* s_instance;

private: // forbidden default functions
  LintillaMmiHomeScreenState& operator = (const LintillaMmiHomeScreenState& src); // assignment operator
  LintillaMmiHomeScreenState(const LintillaMmiHomeScreenState& src);              // copy constructor

};

//-----------------------------------------------------------------------------

class LintillaMmiIdScreenState : public LintillaMmiScreenState
{
private:
  LintillaMmiIdScreenState() { }

public:
  static LintillaMmiScreenState* Instance();

  virtual ~LintillaMmiIdScreenState() { }

  virtual void select(LintillaMmiScreenFsm* fsm);
  virtual void left(LintillaMmiScreenFsm* fsm)  ;
  virtual void right(LintillaMmiScreenFsm* fsm) ;
  virtual void up(LintillaMmiScreenFsm* fsm)    ;
  virtual void down(LintillaMmiScreenFsm* fsm)  ;
  virtual void entry(LintillaMmiScreenFsm* fsm) ;

  virtual const char* toString();

private:
  static LintillaMmiScreenState* s_instance;

private: // forbidden default functions
  LintillaMmiIdScreenState& operator = (const LintillaMmiIdScreenState& src); // assignment operator
  LintillaMmiIdScreenState(const LintillaMmiIdScreenState& src);              // copy constructor

};

//-----------------------------------------------------------------------------

class LintillaMmiWlanScreenState : public LintillaMmiScreenState
{
private:
  LintillaMmiWlanScreenState() { }

public:
  static LintillaMmiScreenState* Instance();

  virtual ~LintillaMmiWlanScreenState() { }

  virtual void select(LintillaMmiScreenFsm* fsm);
  virtual void left(LintillaMmiScreenFsm* fsm)  ;
  virtual void right(LintillaMmiScreenFsm* fsm) ;
  virtual void up(LintillaMmiScreenFsm* fsm)    ;
  virtual void down(LintillaMmiScreenFsm* fsm)  ;
  virtual void entry(LintillaMmiScreenFsm* fsm) ;

  virtual const char* toString();

private:
  static LintillaMmiScreenState* s_instance;

private: // forbidden default functions
  LintillaMmiWlanScreenState& operator = (const LintillaMmiWlanScreenState& src); // assignment operator
  LintillaMmiWlanScreenState(const LintillaMmiWlanScreenState& src);              // copy constructor

};
#endif /* LINTILLAMMISCREENSTATE_H_ */
