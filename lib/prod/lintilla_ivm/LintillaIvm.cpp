/*
 * LintillaIvm.cpp
 *
 *  Created on: 23.01.2014
 *      Author: niklausd
 */

#include "Ivm.h"
#include "IvmSerialEeprom.h"
#include "LintillaIvm.h"

const unsigned char LintillaIvm::s_currentVersion = 1;

LintillaIvm::LintillaIvm()
: Ivm(new IvmSerialEeprom())
{
  maintainVersionChange();
}

LintillaIvm::~LintillaIvm()
{
  delete getIvmMemory();
  setIvmMemory(0);
}

void LintillaIvm::maintainVersionChange()
{
  unsigned char ivmVersion = getIvmVersion();
  unsigned char deviceId   = getDeviceId();
  if (255 == ivmVersion || 0 == ivmVersion)
  {
    // Assume uninitialized IVM memory, bring up to current version
    setIvmVersion(s_currentVersion);

    // Initialize Device ID
    if (255 == deviceId)
    {
      // Assume version uninitialized, initialize to zero.
      setDeviceId(0);
    }
  }
}
