#include <xc.h>
#include <sys/attribs.h>  // __ISR macro
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
void XPT2046_read(unsigned short *x, unsigned short *y, unsigned int *z);
void LCD_writeCharacter(unsigned short x, unsigned short y, char c);
void LCD_writeString(unsigned short x, unsigned short y, char* m);

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
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_YELLOW);
    
    unsigned short x,y;
    unsigned int z;
    unsigned int xpos,ypos;
    char xraw[10];
    char yraw[10];
    char zraw[10];
    
    char xcoord[10];
    char ycoord[10];
    
    int counter = 0;
    char countermessage[10];
    
    int i,j;
    // ++ button
    for (i=0;i<30;i++){
        for (j=0;j<30;j++){
            LCD_drawPixel(150+i,100-j,ILI9341_OLIVE);
        }
    }
    LCD_writeString(160,110,"++");
    
    // -- button
    for (i=0;i<30;i++){
        for (j=0;j<30;j++){
            LCD_drawPixel(150+i,200-j,ILI9341_OLIVE);
        }
    }
    LCD_writeString(160,210,"--");
    
    
    // actual code
    while(1){
        XPT2046_read(&x,&y,&z);
        sprintf(xraw,"x: %4d ",x);
        LCD_writeString(10,10,xraw);
        sprintf(yraw,"y: %4d ",y);
        LCD_writeString(10,30,yraw);
        sprintf(zraw,"z: %4d ",z);
        LCD_writeString(10,50,zraw);
        
        if (z > 400){
            xpos = 240*(x-300)/3500;
            ypos = 330 - (320*(y-300)/3500);
        }
        else{
            xpos = 0;
            ypos = 0;
        }
  
        sprintf(xcoord,"x-coord: %3d ", xpos);
        LCD_writeString(10,100,xcoord);
        sprintf(ycoord,"y-coord: %3d ", ypos);
        LCD_writeString(10,120,ycoord);
        
        if ((xpos > 150) & (xpos < 180)){
            if ((ypos > 70) & (ypos < 100)){
                counter++;
            }
            else if ((ypos > 170) & (ypos < 200)){
                counter--;
            }
        }
        sprintf(countermessage,"Counter:%3d", counter);
        LCD_writeString(130,140,countermessage);
    }
    return 0;
}

void XPT2046_read(unsigned short *x, unsigned short *y, unsigned int *z){
    // Read x
    CS_TOUCH = 0;
    unsigned short t1, t2;
    spi_io(0b11010001); // send command byte. Read x-position. Returns garbage
    t1 = spi_io(0x00); // read the first byte
    t2 = spi_io(0x00); // read the second byte
    CS_TOUCH = 1;
    *x = (t1<<5) | (t2 >> 3); // combine the first and second bytes to create a 12 bit data
    
    // Read y
    CS_TOUCH = 0;
    spi_io(0b10010001); // send command byte. Read y-position. Returns garbage
    t1 = spi_io(0x00); // read the first byte
    t2 = spi_io(0x00); // read the second byte
    CS_TOUCH = 1;
    *y = (t1<<5) | (t2 >> 3);// combine the first and second bytes to create a 12 bit data
    
    // Read z1
    CS_TOUCH = 0;
    spi_io(0b10110001); // send command byte. Read x-position. Returns garbage
    t1 = spi_io(0x00); // read the first byte
    t2 = spi_io(0x00); // read the second byte
    CS_TOUCH = 1;
    unsigned short z1 = (t1<<5) | (t2 >> 3);// combine the first and second bytes to create a 12 bit data
    
    // Read z2
    CS_TOUCH = 0;
    spi_io(0b11000001); // send command byte. Read x-position. Returns garbage
    t1 = spi_io(0x00); // read the first byte
    t2 = spi_io(0x00); // read the second byte
    CS_TOUCH = 1;
    unsigned short z2 = (t1<<5) | (t2 >> 3);// combine the first and second bytes to create a 12 bit data
    
    *z = z1 - z2 + 4095;
 
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

