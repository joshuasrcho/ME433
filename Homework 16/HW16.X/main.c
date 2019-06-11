#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>
#include "pragma.h"
#include "ili9341.h"
#include "i2c_master_noint.h"
#include "ov7670.h"

#define DIR1 LATAbits.LATA10
#define DIR2 LATAbits.LATA7
#define USER PORTBbits.RB4

void startup() {
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // pin definitions
    ANSELA = 0;
    ANSELB = 0;
    TRISAbits.TRISA7 = 0; // DIR2
    DIR2 = 1;
    TRISAbits.TRISA10 = 0; // DIR1
    DIR1 = 1;
    TRISBbits.TRISB4 = 1; // USER
    
    
    // LEFT MOTOR OC1 SETUP
    RPB15Rbits.RPB15R = 0b0101; // OC1 is B15, goes with DIR1
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    PR2 = 2399; // PR = PBCLK / N / desiredF - 1 (20kHz)
    TMR2 = 0; // initial TMR2 count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC1RS = 1200; // duty cycle
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1
    
    
    // RIGHT MOTOR OC4 SETUP
    RPA4Rbits.RPA4R = 0b0101; // OC4 is A4, goes with DIR2
    OC4CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC4RS = 1200; // duty cycle
    OC4R = 0; // initialize before turning OC1 on; afterward it is read-only
    OC4CONbits.ON = 1; // turn on OC1
    
    
    // LCD uses SPI1: A0 is SDO, A1 is SDI, B5 is CST, B14 is SCK1, A9 is DC, B7 is CS
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_DARKGREEN);
    
    
    // Camera uses C0-7, C8 as OC2, A8 as INT3/PCLK, B13 as INT2/HREF, C9 as INT1/VSYNC, and I2C1
    i2c_master_setup();
    ov7670_setup();
    
    // B3 is available as SCL2, B2 is available as SDA2
    RPB2Rbits.RPB2R = 0b0110; // I will use B2 for OC5 to drive the servo
    T3CONbits.TCKPS = 0b110; // Timer3 prescaler N=64 (1:1)
    PR5 = 15000; // PR = PBCLK / N / desiredF - 1 (50Hz)
    TMR3 = 0; // initial TMR3 count is 0
    OC5CONbits.OCTSEL = 1; // OC5 uses Timer3
    OC5CONbits.OCM = 0b110; // PWM mode without fault pin;
    OC5RS = 1500; // duty cycle
    OC5R = 1500; // initialize before turning OC1 on; afterward it is read-only
    T3CONbits.ON = 1; // turn on Timer3
    OC5CONbits.ON = 1; // turn on OC5
    

}

int main() {

     __builtin_disable_interrupts();

    startup();

    __builtin_enable_interrupts();
    
    int I = 0;
    char message[100];
    
       
    while(1) {

        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<48000000/2/2){
            while(USER == 0){}
        }
        DIR1 = 0;
        DIR2 = 1;
        
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<48000000/2/2){
            while(USER == 0){}
        }
        DIR1 = 1;
        DIR2 = 0;
        
        //I++;
        //sprintf(message,"I = %d   ", I);
        //drawString(140,92,message);
        
        unsigned char d[2000];
        
        int c = ov7670_count(d);
        sprintf(message, "c = %d   ",c);
        drawString(140,92,message); // there are 290 rows
        /*
        int i = 0;
        int t = 0;
        for (i=0;i<30;i++){
            sprintf(message, "%d    %d    %d    %d  ",d[t],d[t+1],d[t+2],d[t+3]);
            t=t+4;
            drawString(1,1+i*10,message);
        }
         * */
        
        int x = 0, x2 = 1;
        int y = 0;
        int dim = 0;
        for(x = 0; x < 310; x++, x2=x2+2){
            dim = d[x2]>>3;
            for(y=0;y<32;y++){
                if (y == dim){
                    LCD_drawPixel(y+30,x,ILI9341_BLACK);
                }
                else {
                    LCD_drawPixel(y+30,x,ILI9341_WHITE);
                }
            }
        }
    }
}