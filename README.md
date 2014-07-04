lintilla-embedded
=================

Arduino Robot SW.

Clone this repository recursively into:

C:\git\arduino-projects\lintilla-embedded

using this command:

git clone --recursive git@github.com:ERNICommunity/lintilla-embedded.git C:\git\arduino-projects\lintilla-embedded

After cloning, run the following batch:

 C:\git\arduino-projects\lintilla-embedded\prepare-workspace.bat

This downloads and installs all needed tools to build this and work on this project using Eclipse CDT together with the Eclipse Arduino Plugin, unless the tools are already installed at the following locations:

    Arduino IDE Version 1.5.2: C:\git\arduino-projects\Tools\arduino_revs\arduino-1.5.2
    Eclipse Arduino Workbench Version 2014-07-02_17-56-02: C:\git\arduino-projects\Tools\eclipseArduino_revs\win64.2014-07-02_17-56-02\eclipseArduino

Don’t get discouraged by an error message popping up potentially, just click OK and ignore it. This is not sorted out yet, but hopefully will be fixed soon.

Run the Eclipse Arduino Workbench with this batch:

  C:\git\arduino-projects\lintilla-embedded\eclipseArduino.bat
