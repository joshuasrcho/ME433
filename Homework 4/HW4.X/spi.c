#include <xc.h>
#include "spi.h"

void initSPI1(){
    RPA1Rbits.RPA1R = 0b0011; // SDO1 assigned to pin A1.
    RPB14Rbits.RPB14R = 0b0011; // SS1 assigned to pin B14
    
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    
    SPI1CONbits.ON = 1; // turn on SPI1 
    
  
  
}

char SPI1_IO(char write);
