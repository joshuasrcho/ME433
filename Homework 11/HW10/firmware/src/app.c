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
#include <stdio.h>
#include <xc.h>
#include "i2c_master_noint.h"
#include "ili9341.h"

#define LSM6DS33_ADDR 0x6B // device op code

// Function prototypes
void initIMU(void);
void I2C_read_multiple(unsigned char addr, unsigned char rgstr, unsigned char * data, int lnt);
void LCD_writeCharacter(unsigned short x, unsigned short y, char c);
void LCD_writeString(unsigned short x, unsigned short y, char* m);
void LCD_IMUBar(unsigned short x, unsigned short y, int accelX, int accelY);



// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
bool rstate = false;
int startTime = 0; // to remember the loop time

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

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

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

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    /* PUT YOUR LCD, IMU, AND PIN INITIALIZATIONS HERE */
    
    // setup and initialization
    i2c_master_setup();                       // initialize I2C2, which we use as a master
    initIMU();                           // initialize LSM6DS33 IMU chip
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_YELLOW);

    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);

                        /* AT THIS POINT, appData.readBuffer[0] CONTAINS A LETTER
                        THAT WAS SENT FROM THE COMPUTER */
                        /* YOU COULD PUT AN IF STATEMENT HERE TO DETERMINE WHICH LETTER
                        WAS RECEIVED (USUALLY IT IS THE NULL CHARACTER BECAUSE NOTHING WAS
                      TYPED) */

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
            if (appData.readBuffer[0] == 'r'){
                    rstate = true;
                }
                    
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

             /* WAIT FOR 100HZ TO PASS OR UNTIL A LETTER IS RECEIVED */
            if (appData.isReadComplete) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }
            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            /* PUT THE TEXT YOU WANT TO SEND TO THE COMPUTER IN dataOut
            AND REMEMBER THE NUMBER OF CHARACTERS IN len */
            /* THIS IS WHERE YOU CAN READ YOUR IMU, PRINT TO THE LCD, ETC */
       
            // actual code
            if (rstate){
                i = 0;
                while (i < 100){
                    unsigned char d[15];

                    I2C_read_multiple(LSM6DS33_ADDR, 0x20, d, 14);
                    short temperature = (d[0] << 8) | d[1]; // shift high byte and OR with low byte 
                    short gyroX = (d[2] << 8) | d[3];
                    short gyroY = (d[4] << 8) | d[5];
                    short gyroZ = (d[6] << 8) | d[7];
                    short accelX = (d[8] << 8) | d[9];
                    short accelY = (d[10] << 8) | d[11];
                    short accelZ = (d[12] << 8) | d[13];


                    len = sprintf(dataOut, "%d  %6d  %6d  %6d  %6d  %6d  %6d\r\n",
                            i,accelX,accelY,accelZ,gyroX,gyroY,gyroZ);
                     // increment the index so we see a change in the text
                    i++;

                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle, dataOut, len,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                    startTime = _CP0_GET_COUNT(); // reset the timer for accurate delays
                    while(_CP0_GET_COUNT() - startTime < (48000000 / 2 / 100)){
                        ;
                    }
                }
            }
            else{ // send NULL character
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle, '\0', 1,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
            }
            appData.readBuffer[0] = '\0'; // reset buffer to null character
            rstate = false;
            break;
            

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
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
