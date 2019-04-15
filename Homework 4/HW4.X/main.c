#include <xc.h>           // processor SFR definitions
#include <math.h> 	//for sine wave plotting
#include "spi.h"  // enables SPI communication 

int main(void) {
    char data[] = "Help, I'm stuck in the RAM!";    // the test message
    char read[] = "***************************";    // buffer for reading from ram
    char buf[100];                                  // buffer for comm. with the user
    unsigned char status;                           // used to verify we set the status 
    initSPI1();
    
    int i = 0;

  while(1) {
	_CP0_SET_COUNT(0);
	float f = 512 +512*sin(i*2*3.1415/1000*10);  //should make a 10Hz sin wave)
	i++;



	setVoltage(0,512);		//test
	setVoltage(1,256);		//test

	//while(_CP0_GET_COUNT() < 2400000000/1000) {;}  //check this is 24Million
  }
  return 0;
}