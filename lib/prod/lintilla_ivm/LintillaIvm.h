/*
 * LintillaIvm.h
 *
 *  Created on: 23.01.2014
 *      Author: niklausd
 */

#ifndef LINTILLAIVM_H_
#define LINTILLAIVM_H_

#include <Ivm.h>

class LintillaIvm: public Ivm
{
public:
  LintillaIvm();
  virtual ~LintillaIvm();

private:
  void maintainVersionChange();

private:
  /**
   * Current Version of the Lintilla Inventory Management Capabilities.
   */
  const static unsigned char s_currentVersion;

private: // forbidden default functions
  LintillaIvm& operator = (const LintillaIvm& );  // assignment operator
  LintillaIvm(const LintillaIvm& src);            // copy constructor
};

#endif /* LINTILLAIVM_H_ */
