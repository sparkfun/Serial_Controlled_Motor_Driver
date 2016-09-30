#if !defined(DIAGLEDS_H)
#define DIAGLEDS_H

#include <stdint.h>
    
void setDiagLeds(uint8_t input);

void sendDiagMessage( void ); //send error level

void setDiagMessage( uint8_t errorLevel, uint8_t message ); //send error level

void clearDiagMessage( uint8_t errorLevel ); //send error level
    
    
#endif