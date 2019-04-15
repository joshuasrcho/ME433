#include <xc.h>
#include "spi.h"

void initSPI1(){
    RPA1Rbits.RPA1R = 0b0011;  // SDO1 assigned to pin A1.
    
    TRISBbits.TRISB14 = 0;  // Pin B14 set as digital output which will act as manual SS1
    CS = 1; 
    
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    
    SPI1CONbits.ON = 1; // turn on SPI1 
}

// send a byte via spi and return the response
char SPI1_IO(char write){
  SPI1BUF = write;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage(char channel, int voltage) {
	unsigned short t = 0;
	t= channel << 15; //a is at the very end of the data transfer
	t = t | 0b01110000000000000;
	t = t | ((voltage&0b1111111111) <<2); //rejecting excessive bits (above 10)
	
	CS = 0;
	spi_io(t>>8);
	spi_
	
}
