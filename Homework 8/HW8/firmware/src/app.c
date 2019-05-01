/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "i2c_master_noint.h"
#include "ili9341.h"
#include <stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
#define LSM6DS33_ADDR 0x6B // device op code

void initIMU(void);
void I2C_read_multiple(unsigned char addr, unsigned char rgstr, unsigned char * data, int lnt);
void LCD_writeCharacter(unsigned short x, unsigned short y, char c);
void LCD_writeString(unsigned short x, unsigned short y, char* m);
void LCD_IMUBar(unsigned short x, unsigned short y, int accelX, int accelY);


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
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
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            unsigned char d[15];
            char message[100];
            I2C_read_multiple(LSM6DS33_ADDR, 0x20, d, 14);
            short temperature = (d[0] << 8) | d[1]; // shift high byte and OR with low byte 
            short gyroX = (d[2] << 8) | d[3];
            short gyroY = (d[4] << 8) | d[5];
            short gyroZ = (d[6] << 8) | d[7];
            short accelX = (d[8] << 8) | d[9];
            short accelY = (d[10] << 8) | d[11];
            short accelZ = (d[12] << 8) | d[13];
            sprintf(message,"accelX:%6d",accelX);
            LCD_writeString(20,20,message);
            sprintf(message,"accelY:%6d",accelY);
            LCD_writeString(20,40,message);
        
            LCD_IMUBar(120,160,accelX,accelY);
        
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() < 1200000){
                ;
            }
            LATAbits.LATA4 = !LATAbits.LATA4;
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
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
    int i, j;
    // X direction
    for (i = 0; i < 100; i++){
        for (j = 0; j < 5; j++){
            if (i < (accelX/100)){
               LCD_drawPixel(x+i,y+j,ILI9341_RED); 
            }
            else{
                LCD_drawPixel(x+i,y+j,ILI9341_LIGHTGREY);
            }
            if (-i > (accelX/100)){
                LCD_drawPixel(x-i,y+j,ILI9341_RED); 
            }
            else{
                LCD_drawPixel(x-i,y+j,ILI9341_LIGHTGREY);
            }
            // Y direction
            if (i < (accelY/100)){
               LCD_drawPixel(x+j,y+i,ILI9341_RED); 
            }
            else{
                LCD_drawPixel(x+j,y+i,ILI9341_LIGHTGREY);
            }
            if (-i >= (accelY/100)){
                LCD_drawPixel(x+j,y-i,ILI9341_RED); 
            }
            else{
                LCD_drawPixel(x+j,y-i,ILI9341_LIGHTGREY);
            }
        }
    }
  
}

/*******************************************************************************
 End of File
 */
