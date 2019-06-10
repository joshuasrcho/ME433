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


// Function prototypes
void LCD_plotCameraData(unsigned char* data,unsigned short size,unsigned short y, unsigned short c);
void LCD_writeCharacter(unsigned short x, unsigned short y, char c);
void LCD_writeString(unsigned short x, unsigned short y, char* m);

volatile int interruptCounter = 0;

void __ISR(_TIMER_3_VECTOR, IPL5SOFT) Timer3ISR(void) {
    interruptCounter++;
    if (interruptCounter == 100){
        interruptCounter = 0;
        OC1RS = 0;
        LATAbits.LATA10 = !LATAbits.LATA10;
    }
    //OC1RS = OC1RS + ((PR2+1)/100);
    OC1RS = 2399;
    IFS0bits.T3IF = 0;

}

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
    TRISAbits.TRISA10 = 0; // A4 (green LED) is an output pin
    LATAbits.LATA10 = 0; // A4 is LOW (green LED off)
    
    RPB15Rbits.RPB15R = 0b0101; // A0 is OC1 pin
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    PR2 = 2399; // PR = PBCLK / N / desiredF - 1 (20kHz)
    TMR2 = 0; // initial TMR2 count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC1RS = 1200; // duty cycle
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1
    
    T3CONbits.TCKPS = 6; // Timer3 prescaler = 64
    PR3 = 7499; // PR = PBCLK / N / desiredF - 1 (100Hz)
    TMR3 = 0; // initial TMR3 count is 0
    T3CONbits.ON = 1; // turn on Timer3
    IPC3bits.T3IP = 5; // interrupt priority 5
    IPC3bits.T3IS = 0; // interrupt subpriority 0
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
    
    
    // setup and initialization
    i2c_master_setup();                       // initialize I2C2, which we use as a master
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_YELLOW);
    LATAbits.LATA10 = !LATAbits.LATA10;

    __builtin_enable_interrupts();
    
    char m[100];

    // make dummy data 
    unsigned char r[240];
    unsigned char g[240];
    unsigned char b[240];
    int i;
    for (i=0;i<240;i++){
        r[i] = (unsigned char)(rand() % 255);
        g[i] = (unsigned char)(rand() % 255);
        b[i] = (unsigned char)(rand() % 255);
    }
    
    sprintf(m,"Red");
    LCD_writeString(10,75,m);
    LCD_plotCameraData(r,240,80,ILI9341_RED);
    sprintf(m,"Green");
    LCD_writeString(10,115,m);
    LCD_plotCameraData(g,240,120,ILI9341_OLIVE);
    sprintf(m,"Blue");
    LCD_writeString(10,155,m);
    LCD_plotCameraData(b,240,160,ILI9341_BLUE);
 
    while(1){;}

   

    
    return 0;
}


void LCD_plotCameraData(unsigned char* data, unsigned short size, unsigned short y, unsigned short c){
    unsigned short i;
    unsigned short j;
    for (i=0;i<size;i++){
        for (j=0;j<8;j++){
            LCD_drawPixel(i,y+8-j,ILI9341_WHITE);
        }
    }
    for (i=0;i<size;i++){
            LCD_drawPixel(i,y+8-(data[i]>>5),c);
        
    }
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

