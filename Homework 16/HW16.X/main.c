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
#define SERVO LATBbits.LATB2
#define LEFT 2;
#define RIGHT 6;

volatile int interruptCounter = 0;
volatile int headTurn = 0;

void __ISR(_TIMER_5_VECTOR, IPL5SOFT) Timer5ISR(void) {
    if (interruptCounter < headTurn){ // 2 to 6
        SERVO = 1;
    }
    else{
        SERVO = 0;
        if (interruptCounter == 40){
            interruptCounter = 0;
        }
    }
    interruptCounter++;
    IFS0bits.T5IF = 0; // clear interrupt flag
}

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
    
    
    // RIGHT MOTOR OC1 SETUP
    RPB15Rbits.RPB15R = 0b0101; // OC1 is B15, goes with DIR1
    T3CONbits.TCKPS = 0; // Timer3 prescaler N=1 (1:1)
    PR3 = 2399; // PR = PBCLK / N / desiredF - 1 (20kHz)
    TMR3 = 0; // initial TMR3 count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin
    OC1CONbits.OCTSEL = 1; // use timer3
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    T3CONbits.ON = 1; // turn on Timer3
    OC1CONbits.ON = 1; // turn on OC1
    
    // LEFT MOTOR OC4 SETUP
    RPA4Rbits.RPA4R = 0b0101; // OC4 is A4, goes with DIR2
    OC4CONbits.OCM = 0b110; // PWM mode without fault pin
    OC4CONbits.OCTSEL = 1; // Use timer3
    OC4R = 0; // initialize before turning OC4 on; afterward it is read-only
    OC4CONbits.ON = 1; // turn on OC4
   
    
    // LCD uses SPI1: A0 is SDO, A1 is SDI, B5 is CST, B14 is SCK1, A9 is DC, B7 is CS
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_DARKGREEN);
    
    
    // Camera uses C0-7, C8 as OC2, A8 as INT3/PCLK, B13 as INT2/HREF, C9 as INT1/VSYNC, and I2C1
    i2c_master_setup();
    ov7670_setup();
    
    
    // B3 is available as SCL2, B2 is available as SDA2
    TRISBbits.TRISB2 = 0; // B2 is a digital output
    SERVO = 1;
    T5CONbits.TCKPS = 5; // Timer5 prescaler = 32
    PR5 = 749; // PR = PBCLK / N / desiredF - 1 (2000Hz)
    TMR5 = 0; // initial TMR5 count is 0
    T5CONbits.ON = 1; // turn on Timer5
    IPC5bits.T5IP = 5; // interrupt priority 5
    IPC5bits.T5IS = 0; // interrupt subpriority 0
    IFS0bits.T5IF = 0; // clear interrupt flag
    //IEC0bits.T5IE = 1; // enable interrupt
    
}

int main() {

     __builtin_disable_interrupts();

    startup();

    __builtin_enable_interrupts();
    
    int I = 0;
    char message[100];
    
    while(1) {
        /*
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<48000000*2){
            while(USER == 0){}
        }
        headTurn = LEFT;
        DIR1 = 0;
        DIR2 = 1;
        OC1RS = 1200; // set PWM duty cycle for right motor
        OC4RS = 1200; // set PWM duty cycle for left motor
        
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<48000000*2){
            while(USER == 0){}
        }
        headTurn = RIGHT;
        DIR1 = 1;
        DIR2 = 0;
        */
        
        I++;
        sprintf(message,"I = %d   ", I);
        drawString(140,82,message);
        
        unsigned char d[2000];
        
 
         // vertical read
        int c = ov7670_count_vert(d);
        sprintf(message, "c = %d   ",c);
        drawString(140,92,message); // there are 290 rows
        
        int error = 0;
        int target_edge = c/4;
        int curr_edge = 0;
        int prev_y = 0;
        
        int x = 0, x2 = 0;
        int y = 0;
        int dim = 0;
        for(x = 0; x < c/2; x++, x2=x2+2){
            dim = d[x2]>>3;
            if (x%15 == 0){
                prev_y = dim;
                sprintf(message, "%2d",prev_y);
                drawString(x,65,message);
            }
            if ((dim - prev_y) > 6){
                        curr_edge = x;
                    }
            for(y=0;y<32;y++){
                if (y == dim){
                    
                    LCD_drawPixel(x,y+30,ILI9341_BLACK);
                }
                else {
                    LCD_drawPixel(x,y+30,ILI9341_WHITE);
                }
            }
        }
        
        // at this point, every other index of d[] is a brightness
        // loop through d[] and calculate where the middle of the dip or bump is
        // then set the motor speed and direction to follow the line

                
        sprintf(message, "AE = %d, CE = %3d",target_edge,curr_edge);
        drawString(140,200,message);
        
        error = curr_edge - target_edge;
         
    }
}