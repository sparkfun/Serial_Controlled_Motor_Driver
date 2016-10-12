SparkFun Serial Controlled Motor Driver (SCMD) Software
=======================================

This software is intended to be run on a 328p microcontroller.  It contains a library for processing SPI and I2C communications to the SCMD.
Repository Contents
-------------------
* **/SparkFun_SCMD_Arduino_Library** -- Library folder.  Copy to your Arduino library directory
    * /src - SCMD.cpp, SCMD.h, SCMD_config.h - files required to operate SCMD.  You may find register names in SCMD_config.h useful at the sketch level, when useing readRegister() and writeRegister().
    * /examples/RegisterRWTool - Reads and writes registers in the SCMD by serial input.  Use to explore the nature of the SCMD.
* **/testProgram** - Program that uses the library and drives motors based on slider and switch input (not documented)

License Information
-------------------
The hardware is released under [Creative Commons ShareAlike 4.0 International](https://creativecommons.org/licenses/by-sa/4.0/).
The code is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.