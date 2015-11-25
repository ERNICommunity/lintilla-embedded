[![Stories in Ready](https://badge.waffle.io/ERNICommunity/lintilla-embedded.png?label=ready&title=Ready)](https://waffle.io/ERNICommunity/lintilla-embedded)
lintilla-embedded
=================

Arduino Robot SW.

See also the [wiki](https://github.com/ERNICommunity/lintilla-embedded/wiki) for more information.

Clone this repository recursively into:

    C:\git\arduino-projects\lintilla-embedded

using this command:

    git clone --recursive git@github.com:ERNICommunity/lintilla-embedded.git C:\git\arduino-projects\lintilla-embedded

After cloning, run the following batch:

    C:\git\arduino-projects\lintilla-embedded\prepare-workspace.bat

This downloads and installs all needed tools to build this and work on this project using Eclipse CDT together with the Eclipse Arduino Plugin, unless the tools are already installed at the following locations:

* Arduino IDE Version 1.5.6-r2:
        
        C:\git\arduino-projects\Tools\arduino_revs\arduino-1.5.6-r2

* Eclipse Arduino Workbench Version 2015-03-25_02-06-02:
        
        C:\git\arduino-projects\Tools\eclipseArduino_revs\win64.2015-03-25_02-06-02\eclipseArduino

The workspace is now prepared and the project is built now already.

Run the Eclipse Arduino Workbench with this batch:

    C:\git\arduino-projects\lintilla-embedded\eclipseArduino.bat

In the Eclipse Arduino Workbench select the lintilla_embedded entry in the Project Explorer (on the left-hand side) and press the Verify button, this compiles the project (and the red error icons will disappear). 

![](https://github.com/ERNICommunity/lintilla-embedded/wiki/pictures/select_proj_and_press_verify.png)
