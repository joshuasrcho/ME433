#ifndef SPI_H
#define	SPI_H

#define CS LATBbits.LATB14 // chip select pin

void initSPI1();
char SPI1_IO(char write);

#endif	/* SPI_H */

