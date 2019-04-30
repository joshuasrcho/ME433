#include <xc.h>
#include <sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include "ili9341.h"
#include <stdio.h>


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


#define LSM6DS33_ADDR 0x6B // device op code

// Function prototypes
void initIMU(void);
void I2C_read_multiple(unsigned char addr, unsigned char rgstr, unsigned char * data, int lnt);
void LCD_writeCharacter(unsigned short x, unsigned short y, char c);
void LCD_writeString(unsigned short x, unsigned short y, char* m);
void LCD_IMUBar(unsigned short x, unsigned short y, int length, int accelX, int accelY);

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
    
    TRISBbits.TRISB4 = 1; // B4 (pushbutton) is an input pin
    TRISAbits.TRISA4 = 0; // A4 (green LED) is an output pin
    LATAbits.LATA4 = 0; // A4 is LOW (green LED off)
    
    __builtin_enable_interrupts();
    
    // setup and initialization
    i2c_master_setup();                       // initialize I2C2, which we use as a master
    initIMU();                           // initialize LSM6DS33 IMU chip
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_YELLOW);
    
    // actual code
    char message[100];
    unsigned char d[15];
    char whoami;
    
    // test read from WHO_AM_I register
    i2c_master_start();
    i2c_master_send(LSM6DS33_ADDR << 1); // write
    i2c_master_send(0x0F); // send address of the first register to read from
    i2c_master_restart();
    i2c_master_send((LSM6DS33_ADDR << 1) | 1); // read
    whoami = i2c_master_recv();
    sprintf(message,"WHOAMI: %d",whoami);
    LCD_writeString(150,20,message);
    
    while (1){
        I2C_read_multiple(LSM6DS33_ADDR, 0x20, d, 14);
        short temperature = (d[1] << 8) | d[0]; // shift high byte and OR with low byte 
        short gyroX = (d[3] << 8) | d[2];
        short gyroY = (d[5] << 8) | d[4];
        short gyroZ = (d[7] << 8) | d[6];
        short accelX = (d[9] << 8) | d[8];
        short accelY = (d[11] << 8) | d[10];
        short accelZ = (d[13] << 8) | d[12];
        sprintf(message,"accelX:%hi",accelX);
        LCD_writeString(20,20,message);
        sprintf(message,"accelY:%hi",accelY);
        LCD_writeString(20,40,message);
        
        LCD_drawPixel(120+accelX/2000,160,ILI9341_RED);
        
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT() < 1200000){
            ;
        }
        LATAbits.LATA4 = !LATAbits.LATA4;
    }
    
    return 0;
}

void initIMU(){
    i2c_master_start();
    i2c_master_send(LSM6DS33_ADDR << 1);
    i2c_master_send(0x10); // write to CTRL1_XL register
    i2c_master_send(0x82); // 1.66kHz sample rate, 2g sensitivity, 100Hz filter
    i2c_master_restart();
    i2c_master_send(LSM6DS33_ADDR << 1);
    i2c_master_send(0x11); // write to CTRL2_G register
    i2c_master_send(0x88); // 1.66kHz sample rate, 1000 dps sensitivity
    i2c_master_send(LSM6DS33_ADDR << 1);
    i2c_master_send(0x12); // write to CTRL3_C register
    i2c_master_send(0x4); // turn on IF_INC
    i2c_master_stop();
    
}

void I2C_read_multiple(unsigned char addr, unsigned char rgstr, unsigned char * data, int lnt){
    int i;
    i2c_master_start();
    i2c_master_send(addr << 1); // write
    i2c_master_send(rgstr); // send address of the first register to read from
    i2c_master_restart();
    i2c_master_send((addr << 1) | 1); // read
    
    for (i = 0; i<lnt; i++){
        data[i] = i2c_master_recv();
        if (i == (lnt-1)){
            i2c_master_ack(1);
        }
        else{
            i2c_master_ack(0);
        }
    }
    i2c_master_stop(); 
}

void LCD_writeCharacter(unsigned short x, unsigned short y, char c){
    int i, j;
    if (x <= 235 && y <= 315){
        for (i=0; i<5; i++){
            for (j=0; j<8; j++){
                char t = ASCII[c-32][i] >> (7-j);
                t = t & 0x1;
                if (t){
                    LCD_drawPixel(x+i,y-j,ILI9341_PURPLE);
                }
                else{
                    LCD_drawPixel(x+i,y-j,ILI9341_YELLOW );
                }
            }
        }    
    }
    else{;}
}

void LCD_writeString(unsigned short x, unsigned short y, char* m){
    int i = 0; 
    while(m[i]){ 
        LCD_writeCharacter(x+6*i, y, m[i]); 
        i++;
    }
}

void LCD_IMUBar(unsigned short x, unsigned short y, int accelX, int accelY){
    int i;
    for (i=0; i<3; i++){
        LCD_drawPixel(120,160,ILI9341_RED);
    }
}