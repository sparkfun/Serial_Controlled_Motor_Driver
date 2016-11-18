SparkFun Serial Controlled Motor Driver Arduino Library
========================================

![Serial Controlled Motor Driver](https://cdn.sparkfun.com/assets/learn_tutorials/5/7/4/SCMD_Main_Photo.jpg)

[*Serial Controlled Motor Driver (SCMD) (ROB-13911)*](https://www.sparkfun.com/products/13911)

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/extras** - Additional documentation for the user. These files are ignored by the IDE. 
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE. 
* **library.properties** - General library properties for the Arduino package manager. 

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[Product Repository](https://github.com/sparkfun/Serial_Controlled_Motor_Driver)** - Main repository (including hardware files) for the SCMD.
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/serial-controlled-motor-driver-hookup-guide)** - Basic hookup guide for the SCMD.

### Usage

The library is made such that new motor driver object is constructed without parameters, the user populates the public settings structure, then calls .begin() to start the wire library and apply the communication settings.

Example:

	SCMD myMotorDriver; //This creates an instance of SCMD which will be bound to a single master.
	
	void setup()
	{
		myMotorDriver.settings.commInterface = I2C_MODE; //or SPI_MODE
		myMotorDriver.settings.I2CAddress = 0x5A;
		myMotorDriver.settings.chipSelectPin = 10;
		myMotorDriver.begin();
	}

#### Settings
The main SCMD class has a public member which is named settings.  To configure settings, use the format `myMotorDriver.settings.I2CAddress = (...);` then call .begin() to apply.

settings contains the following members:

* uint8_t commInterface -- Set equal to I2C_MODE or SPI_MODE
* uint8_t I2CAddress -- Set to address that master is configured to in case of I2C usage
* uint8_t chipSelectPin -- Set to chip select pin used on Arduino in case of SPI

### Classes and Structures

There are a few classes used in the library.  The main class is called `SCMD`, which is the object that talks to the motor drivers.  There are also a couple structs in use -- `SCMDSettings` and `SCMDDiagnostics`.  A `SCMDSettings` object named settings is present within the SCMD class for configuration.

#### SCMD
SCMD is used to declare a single chain of motor drivers at specified port and modes in settings. The contained functions are described in a later section.

	class SCMD
	{
	public:
		//settings
		SCMDSettings settings;
		SCMD( void );
		
		uint8_t begin( void );
		... (Other functions...)
		
		uint16_t i2cFaults; //Location to hold i2c faults for alternate
		driver
	};

#### SCMDSettings
SCMDSettings is an type for the settings member of SCMD. It is declared public to be configured by the user.

	struct SCMDSettings
	{
	public:
		//Main Interface and mode settings
		uint8_t commInterface;
		uint8_t I2CAddress;
		uint8_t chipSelectPin;
	};

#### SCMDDiagnostics
SCMDDiagnostics contains a bunch of 8 bit values of data for use with getDiagnostics and getRemoteDiagnostics. Declared objects are passed as a reference to the diagnostic function and written by the collected data.

	struct SCMDDiagnostics
	{
	public:
		//Attainable metrics from SCMD
		uint8_t numberOfSlaves = 0;
		uint8_t U_I2C_RD_ERR = 0;
		uint8_t U_I2C_WR_ERR = 0;
		uint8_t U_BUF_DUMPED = 0;
		uint8_t E_I2C_RD_ERR = 0;
		uint8_t E_I2C_WR_ERR = 0;
		uint8_t LOOP_TIME = 0;
		uint8_t SLV_POLL_CNT = 0;
		uint8_t MST_E_ERR = 0;
		uint8_t MST_E_STATUS = 0;
		uint8_t FSAFE_FAULTS = 0;
		uint8_t REG_OOR_CNT = 0;
		uint8_t REG_RO_WRITE_CNT = 0;
	};

### Functions

#### uint8_t begin( void );

Call after providing settings to start the wire library, apply the settings, and get the ID word (return value should be 0xA9).  Don't progress unless this returns 0xA9!

#### bool ready( void );

This function checks to see if the SCMD is done booting and is ready to receive commands.  Use this after .begin(), and don't progress to your main program until this returns true.

#### bool busy( void );

This function checks to see if the SCMD busy with an operation.  Wait for busy to be clear before sending each configuration commands (not needed for motor drive levels).

#### void enable( void );

Call after .begin(); to allow PWM signals into the H-bridges.  If any outputs are connected as bridged, configure the driver to be bridged before calling .enable();.  This prevents the bridges from shorting out each other before configuration.

#### void disable( void );

Call to remove drive from the H-bridges.  All outputs will go low.

#### void reset( void );

This resets the I2C hardware for Teensy 3 devices using the alternate library, and nothing otherwise.

#### void setDrive( uint8_t channel, uint8_t direction, uint8_t level );

This sets an output to drive at a level and direction.

* channel:  Motor number, 0 through 33.
* direction:  1 or 0 for forward or backwards.
* level: 0 to 255 for drive strength.

#### void inversionMode( uint8_t motorNum, uint8_t polarity );

This switches the perceived direction of a particular motor.

* motorNum:  Motor number, 0 through 33.
* polarity: 0 for normal and 1 for inverted direction.

#### void bridgingMode( uint8_t driverNum, uint8_t bridged );

This connects any board's outputs together controlling both from what was the 'A' position.

* driverNum:  Number of connected SCMD, 0 (master) to 16.
* bridged: 0 for normal and 1 for bridged.

#### void getDiagnostics( SCMDDiagnostics &diagObjectReference );

This returns a diagnostic report from the master.

* &diagObjectReference:  Pass a local SCMDDiagnostics object that will be written into.

#### void getRemoteDiagnostics( uint8_t address, SCMDDiagnostics &diagObjectReference );

This returns a diagnostic report from a slave.

* address: address of intended slave.  This starts at 0x50 for the first slave and goes up from there.
* &diagObjectReference:  Pass a local SCMDDiagnostics object that will be written into.

#### void resetDiagnosticCounts( void );

Clears the diagnostic counts of a master.

#### void resetRemoteDiagnosticCounts( uint8_t address );

Clears the diagnostic counts of a slave.

* address: address of intended slave.  This starts at 0x50 for the first slave and goes up from there.

#### uint8_t readRegister(uint8_t offset);

Returns the contents of a memory location of the master.

* offset:  Memory address to read.

#### void writeRegister(uint8_t offset, uint8_t dataToWrite);

Writes data to a memory location of the master.

* offset:  Memory address to write.
* dataToWrite:  Data to write to that address.

#### uint8_t readRemoteRegister(uint8_t address, uint8_t offset);

Returns the contents of a memory location of a slave.

* address: address of intended slave.  This starts at 0x50 for the first slave and goes up from there.
* offset:  Memory address to read.

#### void writeRemoteRegister(uint8_t address, uint8_t offset, uint8_t dataToWrite);

Writes data to a memory location of a slave.

* address: address of intended slave.  This starts at 0x50 for the first slave and goes up from there.
* offset:  Memory address to write.
* dataToWrite:  Data to write to that address.


Products that use this Library 
---------------------------------

* [ROB-13911](https://www.sparkfun.com/products/13911)- A serial controlled bridgable driver for small DC motors.

Version History
---------------

* [V_1.0.0](https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library/tree/V_1.0.0) - Public Release

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

_<COLLABORATION CREDIT>_
