/*
 * LintillaMmiDbgCmd.cpp
 *
 *  Created on: 06.03.2015
 *      Author: niklausd
 */

#include "Arduino.h"
#include "LintillaMmi.h"
#include "LintillaMmiScreenFsm.h"
#include "LintillaMmiDbgCmd.h"

LintillaMmiDbgCmd_Key::LintillaMmiDbgCmd_Key(LintillaMmi* mmi)
: DbgCli_Command(DbgCli_Node::RootNode()->getNode("dbg", "mmi"), "key", "Emulate LcdKeypad key entry.")
, m_mmi(mmi)
{ }

LintillaMmiDbgCmd_Key::~LintillaMmiDbgCmd_Key()
{ }

void LintillaMmiDbgCmd_Key::printUsage()
{
  Serial.println("dbg mmi key - usage: { s | l | r | u | d }");
}

void LintillaMmiDbgCmd_Key::execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
{
  if (0 != m_mmi)
  {
    if (0 == strcmp(args[idxToFirstArgToHandle], "s"))
    {
      m_mmi->screenFsm()->select();
    }
    else if (0 == strcmp(args[idxToFirstArgToHandle], "l"))
    {
      m_mmi->screenFsm()->left();
    }
    else if (0 == strcmp(args[idxToFirstArgToHandle], "r"))
    {
      m_mmi->screenFsm()->right();
    }
    else if (0 == strcmp(args[idxToFirstArgToHandle], "u"))
    {
      m_mmi->screenFsm()->up();
    }
    else if (0 == strcmp(args[idxToFirstArgToHandle], "d"))
    {
      m_mmi->screenFsm()->down();
    }
    else
    {
      printUsage();
    }
  }
}
