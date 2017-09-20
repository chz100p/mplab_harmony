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

/* Modify this value to alter the LED blink rate */
#define APP_LED_BLINK_DELAY     50  //50 mili second

#define STATE_LED_1 BSP_LED_1
#define STATE_LED_W (500/APP_LED_BLINK_DELAY)
#define STATE_LED_WH (STATE_LED_W/2)
void TimerCallBack(uintptr_t context, uint32_t tickCount)
{
    static uint32_t s = 0;
    static uint32_t w = STATE_LED_W;
    
    if(s){
        if(w==STATE_LED_W){
            BSP_LEDOn(STATE_LED_1);
        }
        if(w==STATE_LED_W-1){
            BSP_LEDOff(STATE_LED_1);
        }
    }else{
        if(w==STATE_LED_W){
            BSP_LEDOn(STATE_LED_1);
        }
        if(w==STATE_LED_WH){
            BSP_LEDOff(STATE_LED_1);
        }
    }
    if (w) {
        --w;
    }
    if(w==0) {
        w = STATE_LED_W;
        if(s){
            --s;
        } else {
            s = (uint32_t) (appData.state);
        }
    }
}

/*********************************************
 * Application USB Device Layer Event Handler
 *********************************************/

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    uint8_t * configurationValue;
    USB_SETUP_PACKET * setupPacket;
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Device is reset or deconfigured. Provide LED indication.*/
            BSP_LEDOn  (APP_USB_LED_1);
            BSP_LEDOn (APP_USB_LED_2);
            BSP_LEDOff (APP_USB_LED_3);

            appData.deviceIsConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration */
            configurationValue = (uint8_t *)eventData;
            if(*configurationValue == 1 )
            {
                /* The device is in configured state. Update LED indication */
                BSP_LEDOff  (APP_USB_LED_1);
                BSP_LEDOff (APP_USB_LED_2);
                BSP_LEDOn (APP_USB_LED_3);

                /* Reset endpoint data send & receive flag  */
                appData.deviceIsConfigured = true;
            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Device is suspended. Update LED indication */
            BSP_LEDOff  (APP_USB_LED_1);
            BSP_LEDOn (APP_USB_LED_2);
            BSP_LEDOn (APP_USB_LED_3);
            break;


        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS is detected. Attach the device */
            USB_DEVICE_Attach(appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is removed. Detach the device */
            USB_DEVICE_Detach (appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:
            /* This means we have received a setup packet */
            setupPacket = (USB_SETUP_PACKET *)eventData;
            if(setupPacket->bRequest == USB_REQUEST_SET_INTERFACE)
            {
                /* If we have got the SET_INTERFACE request, we just acknowledge
                 for now. This demo has only one alternate setting which is already
                 active. */
                USB_DEVICE_ControlStatus(appData.usbDevHandle,USB_DEVICE_CONTROL_STATUS_OK);
            }
            else if(setupPacket->bRequest == USB_REQUEST_GET_INTERFACE)
            {
                /* We have only one alternate setting and this setting 0. So
                 * we send this information to the host. */

                USB_DEVICE_ControlSend(appData.usbDevHandle, &appData.altSetting, 1);
            }
            else
            {
                /* We have received a request that we cannot handle. Stall it*/
                USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            }
            break;

        case USB_DEVICE_EVENT_ENDPOINT_READ_COMPLETE:
           /* Endpoint read is complete */
            appData.epDataReadPending = false;
            break;

        case USB_DEVICE_EVENT_ENDPOINT_WRITE_COMPLETE:
            /* Endpoint write is complete */
            appData.epDataWritePending = false;
            break;

        /* These events are not used in this demo. */
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

//General Declarations
#define MILLIDEBOUNCE 20  //Debounce time in milliseconds
#define NUMBUTTONS 14  //Number of all buttons
#define NUMBUTTONSONLY 10 //Number of just buttons
#define interval 150  //interval in milliseconds to update LED
#define USB_TIMEOUT 12840  //packet timeout for USB

//LED STYLE DEFINES
#define NO_LED 0
#define ONBOARD_LED 1
#define EXTERNAL_LED 2

//LED Pattern Defines
#define ALLOFF 0x00
#define ALLBLINKING 0x01
#define FLASHON1 0x02
#define FLASHON2 0x03
#define FLASHON3 0x04
#define FLASHON4 0x05
#define ON1  0x06
#define ON2  0x07
#define ON3  0x08
#define ON4  0x09
#define ROTATING 0x0A
#define BLINK   0x0B
#define SLOWBLINK 0x0C
#define ALTERNATE 0x0D

//BUTTON MASK DEFINES
#define R3_MASK 0x80
#define L3_MASK 0x40
#define BACK_MASK 0x20
#define START_MASK 0x10
#define DPAD_RIGHT_MASK 0x08
#define DPAD_LEFT_MASK 0x04
#define DPAD_DOWN_MASK 0x02
#define DPAD_UP_MASK 0x01
#define Y_MASK 0x80
#define X_MASK 0x40
#define B_MASK 0x20
#define A_MASK 0x10
#define LOGO_MASK 0x04
#define RB_MASK 0x02
#define LB_MASK 0x01

//Byte location Definitions
#define BUTTON_PACKET_1 2
#define BUTTON_PACKET_2 3
#define LEFT_TRIGGER_PACKET 4
#define RIGHT_TRIGGER_PACKET 5
#define LEFT_STICK_X_PACKET_LSB 6
#define LEFT_STICK_X_PACKET_MSB 7
#define LEFT_STICK_Y_PACKET_LSB 8
#define LEFT_STICK_Y_PACKET_MSB 9
#define RIGHT_STICK_X_PACKET_LSB 10
#define RIGHT_STICK_X_PACKET_MSB 11
#define RIGHT_STICK_Y_PACKET_LSB 12
#define RIGHT_STICK_Y_PACKET_MSB 13

//Pin Declarations
#define pinUP 5  //Up on stick is pin 5
#define pinDN 6  //Down on stick is pin 6
#define pinLT 7  //Left on stick is pin 7
#define pinRT 8  //Right on stick is pin 8
#define pinB1 9  //Button 1 is pin 9 (Start of top row and across)
#define pinB2 10  //Button 2 is pin 10
#define pinB3 11  //Button 3 is pin 11
#define pinB4 12  //Button 4 is pin 12
#define pinB5 14  //Button 5 is pin 13 (Start of second row and across)
#define pinB6 15  //Button 6 is pin 14
#define pinB7 16  //Button 7 is pin 15
#define pinB8 17  //Button 8 is pin 16
#define pinST 18  //Start Button is pin 17
#define pinSL 19  //Select Button is pin 18
#define pinOBLED 13  //Onboard LED pin

//Position of a button in the button status array
#define POSUP 0
#define POSDN 1
#define POSLT 2
#define POSRT 3
#define POSB1 4
#define POSB2 5
#define POSB3 6
#define POSB4 7
#define POSB5 8
#define POSB6 9
#define POSB7 10
#define POSB8 11
#define POSST 12
#define POSSL 13

//Global Variables
uint8_t buttonStatus[NUMBUTTONS]; //array Holds a "Snapshot" of the button status to parse and manipulate
uint8_t TXData[20] = {0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //Holds USB transmit packet data
uint8_t RXData[3] = {0x00, 0x00, 0x00}; //Holds USB receive packet data

//Update the debounced button statuses
//We are looking for falling edges since the boards are built
//for common ground sticks

void buttonUpdate() {
    //  if (joystickUP.update()) {buttonStatus[POSUP] = joystickUP.fallingEdge();}
    //  if (joystickDOWN.update()) {buttonStatus[POSDN] = joystickDOWN.fallingEdge();}
    //  if (joystickLEFT.update()) {buttonStatus[POSLT] = joystickLEFT.fallingEdge();}
    //  if (joystickRIGHT.update()) {buttonStatus[POSRT] = joystickRIGHT.fallingEdge();}
    //  if (button1.update()) {buttonStatus[POSB1] = button1.fallingEdge();}
    //  if (button2.update()) {buttonStatus[POSB2] = button2.fallingEdge();}
    //  if (button3.update()) {buttonStatus[POSB3] = button3.fallingEdge();}
    //  if (button4.update()) {buttonStatus[POSB4] = button4.fallingEdge();}
    //  if (button5.update()) {buttonStatus[POSB5] = button5.fallingEdge();}
    //  if (button6.update()) {buttonStatus[POSB6] = button6.fallingEdge();}
    //  if (button7.update()) {buttonStatus[POSB7] = button7.fallingEdge();}
    //  if (button8.update()) {buttonStatus[POSB8] = button8.fallingEdge();}
    //  if (buttonSTART.update()) {buttonStatus[POSST] = buttonSTART.fallingEdge();}
    //  if (buttonSELECT.update()) {buttonStatus[POSSL] = buttonSELECT.fallingEdge();}
    if (BSP_SwitchStateGet(APP_USB_SWITCH_1) == BSP_SWITCH_STATE_PRESSED) {
        buttonStatus[POSST] = buttonStatus[POSSL] = 0x01;
    } else {
        buttonStatus[POSST] = buttonStatus[POSSL] = 0x00;
    }

}

//ProcessInputs
//Button layout on fight stick
//      SL ST
//5  6  7  8
//1  2  3  4
//X360 Verson
//      BK  ST
//X  Y  RB  LB
//A  B  RT  LT

void processInputs() {
    int i;
    //Zero out button values
    //Start at 2 so that you can keep the message type and packet size
    //Then fill the rest with 0x00's
    for (i = 2; i < 13; i++) {
        TXData[i] = 0x00;
    }

    //Button Packet 1 (usb data array position 2)
    //SOCD cleaner included
    //Programmed behavior is UP+DOWN=UP and LEFT+RIGHT=NEUTRAL
    //DPAD Up
    if (buttonStatus[POSUP]) {
        TXData[BUTTON_PACKET_1] |= DPAD_UP_MASK;
    }
    //DPAD Down
    if (buttonStatus[POSDN] && !buttonStatus[POSUP]) {
        TXData[BUTTON_PACKET_1] |= DPAD_DOWN_MASK;
    }
    //DPAD Left
    if (buttonStatus[POSLT] && !buttonStatus[POSRT]) {
        TXData[BUTTON_PACKET_1] |= DPAD_LEFT_MASK;
    }
    //DPAD Right
    if (buttonStatus[POSRT] && !buttonStatus[POSLT]) {
        TXData[BUTTON_PACKET_1] |= DPAD_RIGHT_MASK;
    }

    //Button Start OR Select OR Both (XBOX Logo)
    if (buttonStatus[POSST] && buttonStatus[POSSL]) {
        TXData[BUTTON_PACKET_2] |= LOGO_MASK;
    } else if (buttonStatus[POSST]) {
        TXData[BUTTON_PACKET_1] |= START_MASK;
    } else if (buttonStatus[POSSL]) {
        TXData[BUTTON_PACKET_1] |= BACK_MASK;
    }

    //Button Packet 2 (usb data array position 3)
    //Button 1
    if (buttonStatus[POSB1]) {
        TXData[BUTTON_PACKET_2] |= A_MASK;
    }
    //Button 2
    if (buttonStatus[POSB2]) {
        TXData[BUTTON_PACKET_2] |= B_MASK;
    }
    //Button 5
    if (buttonStatus[POSB5]) {
        TXData[BUTTON_PACKET_2] |= X_MASK;
    }
    //Button 6
    if (buttonStatus[POSB6]) {
        TXData[BUTTON_PACKET_2] |= Y_MASK;
    }
    //Button 7
    if (buttonStatus[POSB7]) {
        TXData[BUTTON_PACKET_2] |= RB_MASK;
    }
    //Button 8
    if (buttonStatus[POSB8]) {
        TXData[BUTTON_PACKET_2] |= LB_MASK;
    }

    //Triggers (usb data array position 4 and 5)
    //0xFF is full scale
    //Button 3
    if (buttonStatus[POSB3]) {
        TXData[LEFT_TRIGGER_PACKET] = 0xFF;
    }
    //Button 4
    if (buttonStatus[POSB4]) {
        TXData[RIGHT_TRIGGER_PACKET] = 0xFF;
    }
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

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_TIMER_OBJECT_CREATE;
    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    appData.deviceIsConfigured = false;
    appData.endpointRx = (APP_EP_XUSB_OUT | USB_EP_DIRECTION_OUT);
    appData.endpointTx = (APP_EP_XUSB_IN | USB_EP_DIRECTION_IN);
    appData.epDataReadPending = false;
    appData.epDataWritePending = false;
    appData.altSetting = 0;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks (void )
{
    switch(appData.state)
    {
        /* Initial state is to create the timer object for periodic alarm */
        case APP_STATE_TIMER_OBJECT_CREATE:
        {
            appData.tmrServiceHandle = SYS_TMR_ObjectCreate(APP_LED_BLINK_DELAY, 1, TimerCallBack, SYS_TMR_FLAG_PERIODIC);
            if(SYS_TMR_HANDLE_INVALID != appData.tmrServiceHandle)
            {
                appData.state = APP_STATE_INIT;
            }
            break;
        }
        case APP_STATE_INIT:
            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle,  APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured */
            if(appData.deviceIsConfigured == true)
            {
                if (USB_DEVICE_EndpointIsEnabled(appData.usbDevHandle, appData.endpointRx) == false )
                {
                    /* Enable Read Endpoint */
                    USB_DEVICE_EndpointEnable(appData.usbDevHandle, 0, appData.endpointRx,
                            USB_TRANSFER_TYPE_INTERRUPT, 32);
                }
                if (USB_DEVICE_EndpointIsEnabled(appData.usbDevHandle, appData.endpointTx) == false )
                {
                    /* Enable Write Endpoint */
                    USB_DEVICE_EndpointEnable(appData.usbDevHandle, 0, appData.endpointTx,
                            USB_TRANSFER_TYPE_INTERRUPT, 32);
                }
                /* Indicate that we are waiting for read */
                appData.epDataReadPending = true;

                /* Place a new read request. */
                USB_DEVICE_EndpointRead(appData.usbDevHandle, &appData.readTranferHandle,
                        appData.endpointRx, &RXData[0], sizeof(RXData) );

                /* Device is ready to run the main task */
                appData.state = APP_STATE_MAIN_TASK;
            }
            break;

        case APP_STATE_MAIN_TASK:

            if(!appData.deviceIsConfigured)
            {
                /* This means the device got deconfigured. Change the
                 * application state back to waiting for configuration. */
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;

                /* Disable the endpoint*/
                USB_DEVICE_EndpointDisable(appData.usbDevHandle, appData.endpointRx);
                USB_DEVICE_EndpointDisable(appData.usbDevHandle, appData.endpointTx);
                appData.epDataReadPending = false;
                appData.epDataWritePending = false;
            }
            else if (appData.epDataReadPending == false)
            {
//                /* Look at the data the host sent, to see what kind of
//                 * application specific command it sent. */
//
//                switch(receivedDataBuffer[0])
//                {
//                    case 0x80:
//
//                        /* This is the toggle LED command */
//                        BSP_LEDToggle( APP_USB_LED_1 );
//                        BSP_LEDToggle( APP_USB_LED_2 );
//                        break;
//
//                    case 0x81:
//
//                        /* This is a switch check command. Check if the TX is free
//                         * for us to send a status. */
//
//                        if(appData.epDataWritePending == false)
//                        {
//                            /* Echo back to the host PC the command we are fulfilling
//                             * in the first byte.  In this case, the Get Pushbutton
//                             * State command. */
//
//                            transmitDataBuffer[0] = 0x81;
//
//                            if(BSP_SwitchStateGet(APP_USB_SWITCH_1) == BSP_SWITCH_STATE_PRESSED)
//                            {
//                                transmitDataBuffer[1] = 0x00;
//                            }
//                            else
//                            {
//                                transmitDataBuffer[1] = 0x01;
//                            }
//
//                            /* Send the data to the host */
//
//                            appData.epDataWritePending = true;
//
//                            USB_DEVICE_EndpointWrite ( appData.usbDevHandle, &appData.writeTranferHandle,
//                                    appData.endpointTx, &transmitDataBuffer[0],
//                                    sizeof(transmitDataBuffer),
//                                    USB_DEVICE_TRANSFER_FLAGS_MORE_DATA_PENDING);
//                        }
//                        break;
//                    default:
//                        break;
//                }

                appData.epDataReadPending = true ;

                /* Place a new read request. */
                USB_DEVICE_EndpointRead ( appData.usbDevHandle, &appData.readTranferHandle,
                        appData.endpointRx, &RXData[0], sizeof(RXData) );
            } else if (appData.epDataWritePending == false) {
                //Poll Buttons
                buttonUpdate();

                //Process all inputs and load up the usbData registers correctly
                processInputs();

                /* Send the data to the host */

                appData.epDataWritePending = true;

                USB_DEVICE_EndpointWrite(appData.usbDevHandle, &appData.writeTranferHandle,
                        appData.endpointTx, &TXData[0],
                        sizeof (TXData),
                        USB_DEVICE_TRANSFER_FLAGS_MORE_DATA_PENDING);
            }

            break;

        case APP_STATE_ERROR:
            break;

        default:
            break;
    }
}
 

/*******************************************************************************
 End of File
 */
