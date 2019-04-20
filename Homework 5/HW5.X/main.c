#include "i2c_expander.h"
#include "i2c_master_noint.h"
// Demonstrate I2C by having the I2C1 talk to I2C5 on the same PIC32 (PIC32MX795F512H)
// Master will use SDA2 (B2) and SCL2 (B3).  Connect these through resistors to
// Vcc (3.3 V) (2.4k resistors recommended, but around that should be good enough)

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module




#define SLAVE_ADDR 0x32

int main() {
    i2c_master_setup();                       // init I2C2, which we use as a master

    while(1) {
      i2c_master_start();                     // Begin the start sequence
      i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1, 
                                              // which clears bit 0, indicating a write
      i2c_master_send(master_write0);         // send a byte to the slave       
      i2c_master_send(master_write1);         // send another byte to the slave
      i2c_master_restart();                   // send a RESTART so we can begin reading 
      i2c_master_send((SLAVE_ADDR << 1) | 1); // send slave address, left shifted by 1,
                                              // and then a 1 in lsb, indicating read
      master_read0 = i2c_master_recv();       // receive a byte from the bus
      i2c_master_ack(0);                      // send ACK (0): master wants another byte!
      master_read1 = i2c_master_recv();       // receive another byte from the bus
      i2c_master_ack(1);                      // send NACK (1):  master needs no more bytes
      i2c_master_stop();                      // send STOP:  end transmission, give up bus
    }
    return 0;
}
