/******************************************************************************
SCMD_config.h
Serial Controlled Motor Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/Serial_Controlled_Motor_Driver

This file contains names for register locations described in the documentation.
Use in conjunction with readRegister() and writeRegister().

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE _______
Teensy loader ________

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
#if !defined(SCMD_CONFIG_H)
#define SCMD_CONFIG_H

//defaults    
#define ID_WORD            0xA9//Device ID to be programmed into memory for reads
#define START_SLAVE_ADDR   0x50//Start address of slaves
#define MAX_SLAVE_ADDR     0x5F//Max address of slaves
#define MASTER_LOCK_KEY    0x9B
#define USER_LOCK_KEY      0x5C
#define FIRMWARE_VERSION   0x03
    
//Address map
#define SCMD_FID                   0x00
#define SCMD_ID                    0x01
#define SCMD_SLAVE_ADDR            0x02
#define SCMD_CONFIG_BITS           0x03
#define SCMD_U_I2C_RD_ERR          0x04
#define SCMD_U_I2C_WR_ERR          0x05
#define SCMD_U_BUF_DUMPED          0x06
#define SCMD_E_I2C_RD_ERR          0x07
#define SCMD_E_I2C_WR_ERR          0x08
#define SCMD_UPORT_TIME	           0x09
#define SCMD_SLV_POLL_CNT          0x0A
#define SCMD_SLV_TOP_ADDR          0x0B
#define SCMD_MST_E_ERR             0x0C
#define SCMD_MST_E_STATUS          0x0D
#define SCMD_FSAFE_FAULTS          0x0E
#define SCMD_REG_OOR_CNT           0x0F
#define SCMD_REG_RO_WRITE_CNT      0x10

#define SCMD_MOTOR_A_INVERT        0x12
#define SCMD_MOTOR_B_INVERT        0x13
#define SCMD_BRIDGE                0x14
#define SCMD_LOCAL_MASTER_LOCK     0x15
#define SCMD_LOCAL_USER_LOCK       0x16
#define SCMD_MST_E_IN_FN           0x17
#define SCMD_U_PORT_CLKDIV_U       0x18
#define SCMD_U_PORT_CLKDIV_L       0x19
#define SCMD_U_PORT_CLKDIV_CTRL    0x1A
#define SCMD_E_PORT_CLKDIV_U       0x1B
#define SCMD_E_PORT_CLKDIV_L       0x1C
#define SCMD_E_PORT_CLKDIV_CTRL    0x1D
#define SCMD_U_BUS_UART_BAUD       0x1E
//#define SCMD_PAGE_SELECT           0x1F
#define SCMD_MA_DRIVE              0x20
#define SCMD_MB_DRIVE              0x21
#define SCMD_S1A_DRIVE             0x22
#define SCMD_S1B_DRIVE             0x23
#define SCMD_S2A_DRIVE             0x24
#define SCMD_S2B_DRIVE             0x25
#define SCMD_S3A_DRIVE             0x26
#define SCMD_S3B_DRIVE             0x27
#define SCMD_S4A_DRIVE             0x28
#define SCMD_S4B_DRIVE             0x29
#define SCMD_S5A_DRIVE             0x2A
#define SCMD_S5B_DRIVE             0x2B
#define SCMD_S6A_DRIVE             0x2C
#define SCMD_S6B_DRIVE             0x2D
#define SCMD_S7A_DRIVE             0x2E
#define SCMD_S7B_DRIVE             0x2F
#define SCMD_S8A_DRIVE             0x30
#define SCMD_S8B_DRIVE             0x31
#define SCMD_S9A_DRIVE             0x32
#define SCMD_S9B_DRIVE             0x33
#define SCMD_S10A_DRIVE            0x34
#define SCMD_S10B_DRIVE            0x35
#define SCMD_S11A_DRIVE            0x36
#define SCMD_S11B_DRIVE            0x37
#define SCMD_S12A_DRIVE            0x38
#define SCMD_S12B_DRIVE            0x39
#define SCMD_S13A_DRIVE            0x3A
#define SCMD_S13B_DRIVE            0x3B
#define SCMD_S14A_DRIVE            0x3C
#define SCMD_S14B_DRIVE            0x3D
#define SCMD_S15A_DRIVE            0x3E
#define SCMD_S15B_DRIVE            0x3F
#define SCMD_S16A_DRIVE            0x40
#define SCMD_S16B_DRIVE            0x41

#define SCMD_INV_2_9               0x50
#define SCMD_INV_10_17             0x51
#define SCMD_INV_18_25             0x52
#define SCMD_INV_26_33             0x53
#define SCMD_BRIDGE_SLV_L          0x54
#define SCMD_BRIDGE_SLV_H          0x55

#define SCMD_DRIVER_ENABLE         0x70
#define SCMD_UPDATE_RATE           0x71
#define SCMD_FORCE_UPDATE          0x72
#define SCMD_E_BUS_SPEED           0x73
#define SCMD_MASTER_LOCK           0x74
#define SCMD_USER_LOCK             0x75
#define SCMD_FSAFE_TIME            0x76

#define SCMD_REM_ADDR              0x79
#define SCMD_REM_OFFSET            0x7A
#define SCMD_REM_DATA_WR           0x7B
#define SCMD_REM_DATA_RD           0x7C
#define SCMD_REM_WRITE             0x7D
#define SCMD_REM_READ              0x7E

#endif