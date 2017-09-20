/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits,
    and allocates any necessary global system resources, such as the
    sysObj structure that contains the object handles to all the MPLAB Harmony
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#include "system_config.h"
#include "system_definitions.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config PWP =        OFF
#pragma config BWP =        OFF
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      FRCPLL
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    EC
#pragma config OSCIOFNC =   OFF
#pragma config FPBDIV =     DIV_2
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     OFF
#pragma config FWDTWINSZ =  WINSZ_50
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_2
#pragma config FPLLMUL =    MUL_20
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLIDIV =   DIV_1
#pragma config UPLLEN =     ON
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON
#pragma config FVBUSONIO =  ON
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_Timer Initialization Data">
/*** TMR Driver Initialization Data ***/

const DRV_TMR_INIT drvTmr0InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX0,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX0,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX0,
    .prescale = DRV_TMR_PRESCALE_IDX0,
    .mode = DRV_TMR_OPERATION_MODE_16_BIT,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX0,
    .asyncWriteEnable = false,
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_USB Initialization Data">
/******************************************************
 * USB Driver Initialization
 ******************************************************/
/****************************************************
 * Endpoint Table needed by the Device Layer.
 ****************************************************/
uint8_t __attribute__((aligned(512))) endPointTable[DRV_USBFS_ENDPOINTS_NUMBER * 32];
const DRV_USBFS_INIT drvUSBInit =
{
    /* Assign the endpoint table */
    .endpointTable= endPointTable,

    /* Interrupt Source for USB module */
    .interruptSource = INT_SOURCE_USB_1,
    
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    .operationMode = DRV_USBFS_OPMODE_DEVICE,
    
    .operationSpeed = USB_SPEED_FULL,
    
    /* Stop in idle */
    .stopInIdle = false,

    /* Suspend in sleep */
    .suspendInSleep = false,

    /* Identifies peripheral (PLIB-level) ID */
    .usbID = USB_ID_1
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="SYS_TMR Initialization Data">
/*** TMR Service Initialization Data ***/
const SYS_TMR_INIT sysTmrInitData =
{
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    .drvIndex = DRV_TMR_INDEX_0,
    .tmrFreq = 1000, 
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="USB Stack Initialization Data">


/**************************************************
 * USB Device Function Driver Init Data
 **************************************************/
/**************************************************
 * USB Device Layer Function Driver Registration 
 * Table
 **************************************************/
const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1] =
{
    /* Function 1 */
    { 
        .configurationValue = 1,    /* Configuration value */ 
        .interfaceNumber = 0,       /* First interfaceNumber of this function */ 
        .speed = USB_SPEED_FULL,    /* Function Speed */ 
        .numberOfInterfaces = 4,    /* Number of interfaces */
        .funcDriverIndex = 0,  /* Index of Vendor Driver */
        .driver = NULL,            /* No Function Driver data */ 
        .funcDriverInit = NULL     /* No Function Driver Init data */
    },
};

/*******************************************
 * USB Device Layer Descriptors
 *******************************************/
/*******************************************
 *  USB Device Descriptor 
 *******************************************/
const USB_DEVICE_DESCRIPTOR deviceDescriptor =
{
    0x12,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,          // DEVICE descriptor type
    0x0200,                         // USB Spec Release Number in BCD format
    0xFF,                           // Class Code
    0xFF,                           // Subclass code
    0xFF,                           // Protocol code
    USB_DEVICE_EP0_BUFFER_SIZE,     // Max packet size for EP0, see system_config.h
    0x22ea,                         // Vendor ID
    0xff07,                         // Product ID
    0x0100,                         // Device release number in BCD format
    0x01,                           // Manufacturer string index
    0x02,                           // Product string index
    0x03,                           // Device serial number string index
    0x01                            // Number of possible configurations
};


/*******************************************
 *  USB Full Speed Configuration Descriptor
 *******************************************/
const uint8_t fullSpeedConfigurationDescriptor[]=
{
    /* Configuration Descriptor */

    0x09,                                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                       // Descriptor Type
    153,0,                //(153 Bytes)Size of the Config descriptor.e
    4,                                               // Number of interfaces in this cfg
    0x01,                                               // Index value of this configuration
    0x00,                                               // Configuration string index
    USB_ATTRIBUTE_DEFAULT | USB_ATTRIBUTE_SELF_POWERED, // Attributes
    0xFA,                                                 // Max power consumption (2X mA)
    /* Descriptor for Function 1 - Vendor     */ 
    
  9,				//bLength (length of interface descriptor 9 bytes)
  4,				//bDescriptorType (4 is interface)
  0,				//bInterfaceNumber (This is interface 0)
  0,				//bAlternateSetting (used to select alternate setting.  notused)
  2,				//bNumEndpoints (this interface has 2 endpoints)
  0xFF,			//bInterfaceClass (Vendor Defined is 255)
  0x5D,			//bInterfaceSubClass
  0x01,			//bInterfaceProtocol
  0,				//iInterface (Index of string descriptor for describing this notused)
  //Some sort of common descriptor? I pulled this from Message Analyzer dumps of an actual controller
  17,33,0,1,1,37,129,20,0,0,0,0,19,2,8,0,0,
  //Endpoint 1 IN
  7,				//bLength (length of ep1in in descriptor 7 bytes)
  5,				//bDescriptorType (5 is endpoint)
  0x81,			//bEndpointAddress (0x81 is IN1)
  0x03,			//bmAttributes (0x03 is interrupt no synch, usage type data)
  0x20, 0x00,		//wMaxPacketSize (0x0020 is 1x32 bytes)
  4,				//bInterval (polling interval in frames 4 frames)
  //Endpoint 2 OUT
  7,				//bLength (length of ep2out in descriptor 7 bytes)
  5,				//bDescriptorType (5 is endpoint)
  0x02,			//bEndpointAddress (0x02 is OUT2)
  0x03,			//bmAttributes (0x03 is interrupt no synch, usage type data)
  0x20, 0x00,		//wMaxPacketSize (0x0020 is 1x32 bytes)
  8,				//bInterval (polling interval in frames 8 frames)
//Interface 1
  9,				//bLength (length of interface descriptor 9 bytes)
  4,				//bDescriptorType (4 is interface)
  1,				//bInterfaceNumber (This is interface 1)
  0,				//bAlternateSetting (used to select alternate setting.  notused)
  4,				//bNumEndpoints (this interface has 4 endpoints)
  0xFF,			//bInterfaceClass (Vendor Defined is 255)
  0x5D,			//bInterfaceSubClass (93)
  0x03,			//bInterfaceProtocol (3)
  0,				//iInterface (Index of string descriptor for describing this notused)
  //A different common descriptor? I pulled this from Message Analyzer dumps of an actual controller
  27,33,0,1,1,1,131,64,1,4,32,22,133,0,0,0,0,0,0,22,5,0,0,0,0,0,0,
  //Endpoint 3 IN
  7,				//bLength (length of ep3in descriptor 7 bytes)
  5,				//bDescriptorType (5 is endpoint)
  0x83,			//bEndpointAddress (0x83 is IN3)
  0x03,			//bmAttributes (0x03 is interrupt no synch, usage type data)
  0x20, 0x00,		//wMaxPacketSize (0x0020 is 1x32 bytes)
  2,				//bInterval (polling interval in frames 2 frames)
  //Endpoint 4 OUT
  7,				//bLength (length of ep4out descriptor 7 bytes)
  5,				//bDescriptorType (5 is endpoint)
  0x04,			//bEndpointAddress (0x04 is OUT4)
  0x03,			//bmAttributes (0x03 is interrupt no synch, usage type data)
  0x20, 0x00,		//wMaxPacketSize (0x0020 is 1x32 bytes)
  4,				//bInterval (polling interval in frames 4 frames)
  //Endpoint 5 IN
  7,				//bLength (length of ep5in descriptor 7 bytes)
  5,				//bDescriptorType (5 is endpoint)
  0x85,			//bEndpointAddress (0x85 is IN5)
  0x03,			//bmAttributes (0x03 is interrupt no synch, usage type data)
  0x20, 0x00,		//wMaxPacketSize (0x0020 is 1x32 bytes)
  64,				//bInterval (polling interval in frames 64 frames)
  //Endpoint 5 OUT (shares endpoint number with previous)
  7,				//bLength (length of ep5out descriptor 7 bytes)
  5,				//bDescriptorType (5 is endpoint)
  0x05,			//bEndpointAddress (0x05 is OUT5)
  0x03,			//bmAttributes (0x03 is interrupt no synch, usage type data)
  0x20, 0x00,		//wMaxPacketSize (0x0020 is 1x32 bytes)
  16,				//bInterval (polling interval in frames 16 frames)
//Interface 2
  9,				//bLength (length of interface descriptor 9 bytes)
  4,				//bDescriptorType (4 is interface)
  2,				//bInterfaceNumber (This is interface 2)
  0,				//bAlternateSetting (used to select alternate setting.  notused)
  1,				//bNumEndpoints (this interface has 4 endpoints)
  0xFF,			//bInterfaceClass (Vendor Defined is 255)
  0x5D,			//bInterfaceSubClass (93)
  0x02,			//bInterfaceProtocol (3)
  0,				//iInterface (Index of string descriptor for describing this notused)
  //Common Descriptor.  Seems that these come after every interface description?
  9,33,0,1,1,34,134,7,0,
  //Endpoint 6 IN
  7,				//bLength (length of ep6in descriptor 7 bytes)
  5,				//bDescriptorType (5 is endpoint)
  0x86,			//bEndpointAddress (0x86 is IN6)
  0x03,			//bmAttributes (0x03 is interrupt no synch, usage type data)
  0x20, 0x00,		//wMaxPacketSize (0x0020 is 1x32 bytes)
  16,				//bInterval (polling interval in frames 64 frames)+
//Interface 3
//This is the interface on which all the security handshaking takes place
//We don't use this but it could be used for man-in-the-middle stuff
  9,				//bLength (length of interface descriptor 9 bytes)
  4,				//bDescriptorType (4 is interface)
  3,				//bInterfaceNumber (This is interface 3)
  0,				//bAlternateSetting (used to select alternate setting.  notused)
  0,				//bNumEndpoints (this interface has 0 endpoints ???)
  0xFF,			//bInterfaceClass (Vendor Defined is 255)
  0xFD,			//bInterfaceSubClass (253)
  0x13,			//bInterfaceProtocol (19)
  4,				//iInterface (Computer never asks for this, but an x360 would. so include one day?)
  //Another interface another Common Descriptor
  6,65,0,1,1,3

};

/*******************************************
 * Array of Full speed config descriptors
 *******************************************/
USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE fullSpeedConfigDescSet[1] =
{
    fullSpeedConfigurationDescriptor
};


/**************************************
 *  String descriptors.
 *************************************/

 /*******************************************
 *  Language code string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;
        uint8_t bDscType;
        uint16_t string[1];
    }
    sd000 =
    {
        sizeof(sd000),          // Size of this descriptor in bytes
        USB_DESCRIPTOR_STRING,  // STRING descriptor type
        {0x0409}                // Language ID
    };
/*******************************************
 *  Manufacturer string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[13];    // String
    }
    sd001 =
    {
        sizeof(sd001),
        USB_DESCRIPTOR_STRING,
        {'b','i','t','-','t','r','a','d','e','-','o','n','e'}
		
    };

/*******************************************
 *  Product string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[10];    // String
    }
    sd002 =
    {
        sizeof(sd002),
        USB_DESCRIPTOR_STRING,
		{'C','o','n','t','r','o','l','l','e','r'}
    }; 

/*******************************************
 *  
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[16];    // String
    }
    sd003 =
    {
        sizeof(sd003),
        USB_DESCRIPTOR_STRING,
		{'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'}
    }; 

/*******************************************
 *  
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint8_t string[0xB0];    // String
    }
    sd004 =
    {
        sizeof(sd004),
        USB_DESCRIPTOR_STRING,
    {
//struct usb_string_descriptor_struct usb_string_xinput_security_descriptor = {
    /*0xB2, 0x03,*/ 0x58, 0x00, 0x62, 0x00, 0x6F, 0x00, 0x78, 0x00, 0x20, 0x00, 0x53, 0x00, 0x65, 0x00,
    0x63, 0x00, 0x75, 0x00, 0x72, 0x00, 0x69, 0x00, 0x74, 0x00, 0x79, 0x00, 0x20, 0x00, 0x4D, 0x00,
    0x65, 0x00, 0x74, 0x00, 0x68, 0x00, 0x6F, 0x00, 0x64, 0x00, 0x20, 0x00, 0x33, 0x00, 0x2C, 0x00,
    0x20, 0x00, 0x56, 0x00, 0x65, 0x00, 0x72, 0x00, 0x73, 0x00, 0x69, 0x00, 0x6F, 0x00, 0x6E, 0x00,
    0x20, 0x00, 0x31, 0x00, 0x2E, 0x00, 0x30, 0x00, 0x30, 0x00, 0x2C, 0x00, 0x20, 0x00, 0xA9, 0x00,
    0x20, 0x00, 0x32, 0x00, 0x30, 0x00, 0x30, 0x00, 0x35, 0x00, 0x20, 0x00, 0x4D, 0x00, 0x69, 0x00,
    0x63, 0x00, 0x72, 0x00, 0x6F, 0x00, 0x73, 0x00, 0x6F, 0x00, 0x66, 0x00, 0x74, 0x00, 0x20, 0x00,
    0x43, 0x00, 0x6F, 0x00, 0x72, 0x00, 0x70, 0x00, 0x6F, 0x00, 0x72, 0x00, 0x61, 0x00, 0x74, 0x00,
    0x69, 0x00, 0x6F, 0x00, 0x6E, 0x00, 0x2E, 0x00, 0x20, 0x00, 0x41, 0x00, 0x6C, 0x00, 0x6C, 0x00,
    0x20, 0x00, 0x72, 0x00, 0x69, 0x00, 0x67, 0x00, 0x68, 0x00, 0x74, 0x00, 0x73, 0x00, 0x20, 0x00,
    0x72, 0x00, 0x65, 0x00, 0x73, 0x00, 0x65, 0x00, 0x72, 0x00, 0x76, 0x00, 0x65, 0x00, 0x64, 0x00,
    0x2E, 0x00
    }
    };
/***************************************
 * Array of string descriptors
 ***************************************/
USB_DEVICE_STRING_DESCRIPTORS_TABLE stringDescriptors[5]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002,
    (const uint8_t *const)&sd003,
    (const uint8_t *const)&sd004
};

/*******************************************
 * USB Device Layer Master Descriptor Table 
 *******************************************/
const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor =
{
    &deviceDescriptor,          /* Full speed descriptor */
    1,                          /* Total number of full speed configurations available */
    fullSpeedConfigDescSet,     /* Pointer to array of full speed configurations descriptors*/
    NULL, 
    0, 
    NULL, 
    3,                          // Total number of string descriptors available.
    stringDescriptors,          // Pointer to array of string descriptors.
    NULL, 
    NULL
};


/****************************************************
 * USB Device Layer Initialization Data
 ****************************************************/
const USB_DEVICE_INIT usbDevInitData =
{
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    /* Number of function drivers registered to this instance of the
       USB device layer */
    .registeredFuncCount = 1,
    
    /* Function driver table registered to this instance of the USB device layer*/
    .registeredFunctions = (USB_DEVICE_FUNCTION_REGISTRATION_TABLE*)funcRegistrationTable,

    /* Pointer to USB Descriptor structure */
    .usbMasterDescriptor = (USB_DEVICE_MASTER_DESCRIPTOR*)&usbMasterDescriptor,

    /* USB Device Speed */
    .deviceSpeed = USB_SPEED_FULL,
    
    /* Index of the USB Driver to be used by this Device Layer Instance */
    .driverIndex = DRV_USBFS_INDEX_0,

    /* Pointer to the USB Driver Functions. */
    .usbDriverInterface = DRV_USBFS_DEVICE_INTERFACE,
    
    /* Specify queue size for vendor endpoint read */
    .queueSizeEndpointRead = 1,
    
    /* Specify queue size for vendor endpoint write */
    .queueSizeEndpointWrite= 1,
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)NULL);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_PORTS_Initialize();

    /* Board Support Package Initialization */
    BSP_Initialize();        

    /* Initialize Drivers */

    sysObj.drvTmr0 = DRV_TMR_Initialize(DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&drvTmr0InitData);

    SYS_INT_VectorPrioritySet(INT_VECTOR_T1, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T1, INT_SUBPRIORITY_LEVEL0);
 
 
     /* Initialize USB Driver */ 
    sysObj.drvUSBObject = DRV_USBFS_Initialize(DRV_USBFS_INDEX_0, (SYS_MODULE_INIT *) &drvUSBInit);

    /* Initialize System Services */

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();

    /*** TMR Service Initialization Code ***/
    sysObj.sysTmr  = SYS_TMR_Initialize(SYS_TMR_INDEX_0, (const SYS_MODULE_INIT  * const)&sysTmrInitData);

    /* Initialize Middleware */
    /* Set priority of USB interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL4);

    /* Set Sub-priority of USB interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);


    /* Initialize the USB device layer */
    sysObj.usbDevObject0 = USB_DEVICE_Initialize (USB_DEVICE_INDEX_0 , ( SYS_MODULE_INIT* ) & usbDevInitData);

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

