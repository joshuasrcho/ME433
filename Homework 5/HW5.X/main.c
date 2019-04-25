#include "i2c_master_noint.h"
#include <xc.h>
#include <sys/attribs.h>  // __ISR macro

// Demonstrate I2C by having the I2C1 talk to I2C5 on the same PIC32 (PIC32MX250F128H)
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


#define EXPANDER_ADDR 0x20 // device op code

// Function prototypes
void initExpander(void);
void setExpander(unsigned char pin, unsigned char level);
char getExpander();

int main() {
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    __builtin_enable_interrupts();
    
    i2c_master_setup();                       // init I2C2, which we use as a master
    initExpander();                           // initialize MCP23008 I/O expander
    while(1){
        if (getExpander() >> 7){
            setExpander(0,0);
        }
        else{
            setExpander(0,1);
        }
    }
    return 0;
}

void initExpander(void){
    i2c_master_start();
    i2c_master_send(EXPANDER_ADDR << 1); // send the slave address, left shifted by 1, 
                                         // which clears bit 0, indicating a write
    i2c_master_send(0x00);                // send I/O direction register address
    i2c_master_send(0xF0);                // set GP0-3 as outputs and GP4-7 as inputs
    i2c_master_stop();
}

void setExpander(unsigned char pin, unsigned char level){
    i2c_master_start();
    i2c_master_send(EXPANDER_ADDR << 1); // send the slave address, left shifted by 1, 
                                         // which clears bit 0, indicating a write
    i2c_master_send(0xA);               // send output latch register address
    i2c_master_send(level << pin);       // set pin to high or low
    i2c_master_stop();
}

char getExpander(){
    i2c_master_start();
    i2c_master_send(EXPANDER_ADDR << 1);       // send slave address left shifted by 1,
                                               // which clears bit 0, indicating a write
    i2c_master_send(0x9);                      // send GPIO register address
    i2c_master_restart();
    i2c_master_send((EXPANDER_ADDR << 1) | 1); // send slave address left shifted by 1,
                                               // set least significant bit to 1, indicating a read
    char r = i2c_master_recv();
    i2c_master_ack(1);                    // send NACK = 1 (no more bytes requested from slave)
    i2c_master_stop();
    return r;
}
