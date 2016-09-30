#if !defined(DEV_REGISTERS_H)
#define DEV_REGISTERS_H
#include "stdint.h" 

#define SCMD_STATUS                0x00
#define SCMD_ID                    0x01
#define SCMD_SLAVE_ADDR            0x02
#define SCMD_CONFIG_BITS           0x03
#define SCMD_I2C_FAULTS            0x04
#define SCMD_I2C_RD_ERR            0x05
#define SCMD_I2C_WR_ERR            0x06
#define SCMD_SPI_FAULTS            0x07
#define SCMD_UART_FAULTS           0x08
#define SCMD_UPORT_TIME	           0x09
#define SCMD_SLV_POLL_CNT          0x0A
#define SCMD_SLV_TOP_ADDR          0x0B
#define SCMD_FSAFE_TIME            0x0C
#define SCMD_FSAFE_FAULTS          0x0D
                                  
#define SCMD_SLAVE_ID              0x10
#define SCMD_REM_ADDR              0x11
#define SCMD_REM_OFFSET            0x12
#define SCMD_REM_DATA_WR           0x13
#define SCMD_REM_DATA_RD           0x14
#define SCMD_REM_WRITE             0x15
#define SCMD_REM_READ              0x16

#define SCMD_MOTOR_A_INVERT        0x17
#define SCMD_MOTOR_B_INVERT        0x18
#define SCMD_BRIDGE                0x19

#define SCMD_BAUD_RATE             0x1E
                                
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

#define SCMD_INV_2_9               0x60
#define SCMD_INV_10_17             0x61
#define SCMD_INV_18_25             0x62
#define SCMD_INV_26_33             0x63
#define SCMD_BRIDGE_SLV_L          0x64
#define SCMD_BRIDGE_SLV_H          0x65

#define SCMD_DRIVER_ENABLE         0x70
    
void initDevRegisters( void );
uint8_t readDevRegister( uint8_t );
void writeDevRegister( uint8_t regNumberIn, uint8_t dataToWrite );
void incrementDevRegister( uint8_t );
uint8_t getChangedStatus( uint8_t regNumberIn );
void clearChangedStatus( uint8_t regNumberIn );

#endif