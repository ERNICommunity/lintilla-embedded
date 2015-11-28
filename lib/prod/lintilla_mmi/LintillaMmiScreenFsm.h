/*
 * LintillaMmiScreenFsm.h
 *
 *  Created on: 15.09.2014
 *      Author: niklausd
 */

#ifndef LINTILLAMMISCREENFSM_H_
#define LINTILLAMMISCREENFSM_H_

class LintillaMmi;
class LintillaMmiScreenState;
class DbgTrace_Port;

class LintillaMmiScreenFsm
{
public:
  LintillaMmiScreenFsm(LintillaMmi* mmi);
  virtual ~LintillaMmiScreenFsm();

  void select();
  void left();
  void right();
  void up();
  void down();

  void changeState(LintillaMmiScreenState* state);

  void updateDisplay();

  DbgTrace_Port* trPort();

private:
  LintillaMmiScreenState* m_state;
  DbgTrace_Port* m_trPort;

private: // forbidden default functions
  LintillaMmiScreenFsm& operator = (const LintillaMmiScreenFsm& src);  // assignment operator
  LintillaMmiScreenFsm(const LintillaMmiScreenFsm& src);               // copy constructor
};

#endif /* LINTILLAMMISCREENFSM_H_ */
