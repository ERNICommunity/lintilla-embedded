[![Stories in Ready](https://badge.waffle.io/ERNICommunity/lintilla-embedded.png?label=ready&title=Ready)](https://waffle.io/ERNICommunity/lintilla-embedded)
lintilla-embedded
=================

:warning: **REPOSITORY NOT ACTIVE SINCE 2019 Dependabot alerts in git security settings disabled**

Arduino Robot SW.

See also the [wiki](https://github.com/ERNICommunity/lintilla-embedded/wiki) for more information.

## How to build (for Eclipse)
  1. Create a directory where your Eclipse Workspace will be stored and where this project shall be cloned into. E.g. `C:\git\pio-prj`
  2. Clone this repository recursively into the folder you created before, `git clone --recursive git@github.com:ERNICommunity/lintilla-embedded.git`
  3. Open a command shell in the just cloned project folder, i.e in `C:\git\pio-prj\lintilla-embedded`
  4. Run the command `pio init -b megaatmega2560 --ide eclipse`
  5. Run the command `pio run`

## Open project in Eclipse CDT
  6. Open Eclipse CDT, as workspace choose the folder you created before, i.e `C:\git\pio-prj`
  7. Import the project with File->Import->General->Existing Projects into Workspace, choose the `lintilla-embedded` (i.e `C:\git\pio-prj\lintilla-embedded`)
