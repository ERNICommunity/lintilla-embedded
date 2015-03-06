/*
 * LintillaMmiDbgCmd.h
 *
 *  Created on: 06.03.2015
 *      Author: niklausd
 */

#ifndef PROD_LINTILLA_MMI_LINTILLAMMIDBGCMD_H_
#define PROD_LINTILLA_MMI_LINTILLAMMIDBGCMD_H_

#include "DbgCliCommand.h"

class LintillaMmi;

class LintillaMmiDbgCmd_Key: DbgCli_Command
{
public:
  LintillaMmiDbgCmd_Key(LintillaMmi* mmi);
  virtual ~LintillaMmiDbgCmd_Key();
  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle);

private:
  void printUsage();

private:
  LintillaMmi* m_mmi;

private: // forbidden default functions
  LintillaMmiDbgCmd_Key& operator = (const LintillaMmiDbgCmd_Key& src);  // assignment operator
  LintillaMmiDbgCmd_Key(const LintillaMmiDbgCmd_Key& src);               // copy constructor
};

#endif /* PROD_LINTILLA_MMI_LINTILLAMMIDBGCMD_H_ */
