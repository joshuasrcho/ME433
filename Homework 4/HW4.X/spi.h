#ifndef SPI_H
#define	SPI_H

#define CS LATBbits.LATB3 // chip select pin

void initSPI1();
char SPI1_IO(char write);
void setVoltage(char channel, int voltage);

#endif	/* SPI_H */

