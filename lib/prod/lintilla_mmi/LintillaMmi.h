/*
 * LintillaMMI.h
 *
 *  Created on: 12.07.2014
 *      Author: niklausd
 */

#ifndef LINTILLAMMI_H_
#define LINTILLAMMI_H_

class LintillaMmi
{
public:
  LintillaMmi();
  virtual ~LintillaMmi();

  LcdKeypad* lcdKeypad();

  bool isIvmAccessMode();
  bool isIvmRobotIdEditMode();

  void setIvmAccessMode(bool isIvmAccessMode);
  void setIvmRobotIdEditMode(bool isIvmRobotIdEditMode);

private:
  LcdKeypad* m_lcdKeypad;

  // Display Menu states
  bool m_isIvmAccessMode;
  bool m_isIvmRobotIdEditMode;

private: // forbidden default functions
  LintillaMmi& operator = (const LintillaMmi& src);  // assignment operator
  LintillaMmi(const LintillaMmi& src);               // copy constructor
};

#endif /* LINTILLAMMI_H_ */
