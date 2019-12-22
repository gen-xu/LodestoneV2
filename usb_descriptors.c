/*********************************************************************
 * Descriptor specific type definitions are defined in:
 * usb_device.h
 *
 * Configuration options are defined in:
 * usb_config.h
 ********************************************************************/
#ifndef __USB_DESCRIPTORS_C
#define __USB_DESCRIPTORS_C
/** INCLUDES *******************************************************/
#include "usb.h"
/* Device Descriptor */
ROM USB_DEVICE_DESCRIPTOR device_dsc =
    {
        .bLength = 0x12,                          // Size of this descriptor in bytes
        .bDescriptorType = USB_DESCRIPTOR_DEVICE, // DEVICE descriptor type
        .bcdUSB = 0x0200,                         // USB Spec Release Number in BCD format, USB2.0 Compliant (Full-Speed)
        .bDeviceClass = 0xFF,                     // Class Code
        .bDeviceSubClass = 0x00,                  // Subclass code
        .bDeviceProtocol = 0x00,                  // Protocol code
        .bMaxPacketSize0 = USB_EP0_BUFF_SIZE,     // Max packet size for EP0, see usb_config.h
        .idVendor = USB_VID,                      // Vendor ID
        .idProduct = USB_PID,                     // Product ID: PICDEM FS USB (DEMO Mode)
        .bcdDevice = 0x0200,                      // Device release number in BCD format      //v2.0
        .iManufacturer = 0x01,                    // Manufacturer string index                //Votec
        .iProduct = 0x02,                         // Product string index                     //Lodestone
        .iSerialNumber = 0x03,                    // Device serial number string index
        .bNumConfigurations = 0x01                // Number of possible configurations
};

/* Configuration 1 Descriptor */
ROM BYTE configDescriptor1[] = {
    /* Configuration Descriptor */
    0x09,                         //sizeof(USB_CFG_DSC), Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION, // CONFIGURATION descriptor type
    0x35, 0x00,                   // Total length of data for this cfg
    1,                            // Number of interfaces in this cfg
    1,                            // Index value of this configuration
    0,                            // Configuration string index
    _DEFAULT,                     // Attributes, see usb_device.h
    50,                           // Max power consumption (2X mA)

    /* Interface Descriptor */
    0x09,                     //sizeof(USB_INTF_DSC),        // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE, // INTERFACE descriptor type
    0,                        // Interface Number
    0,                        // Alternate Setting Number
    5,                        // Number of endpoints in this intf
    0xFF,                     // Class code
    0xFF,                     // Subclass code
    0xFF,                     // Protocol code
    0,                        // Interface string index

    //Magnet Values
    /* Endpoint Descriptor */
    0x07,                    //sizeof(USB_EP_DSC)
    USB_DESCRIPTOR_ENDPOINT, //Endpoint Descriptor
    _EP01_OUT,               //EndpointAddress
    _INTERRUPT,              //Attributes
    12, 0x00,                //size
    1,                       //Interval

    //Input Voltage
    /* Endpoint Descriptor */
    0x07,                    //sizeof(USB_EP_DSC)
    USB_DESCRIPTOR_ENDPOINT, //Endpoint Descriptor
    _EP02_IN,                //EndpointAddress
    _INTERRUPT,              //Attributes
    4, 0x00,                 //size
    1,                       //Interval

    //Control Message
    /* Endpoint Descriptor */
    0x07,                    //sizeof(USB_EP_DSC)
    USB_DESCRIPTOR_ENDPOINT, //Endpoint Descriptor
    _EP03_OUT,               //EndpointAddress
    _INTERRUPT,              //Attributes
    4, 0x00,                 //size
    1,                       //Interval

    //Config
    /* Endpoint Descriptor */
    0x07,                    //sizeof(USB_EP_DSC)
    USB_DESCRIPTOR_ENDPOINT, //Endpoint Descriptor
    _EP04_IN,                //EndpointAddress
    _BULK,                   //Attributes
    64, 0x00,                //size
    1,                       //Interval

    //Config
    /* Endpoint Descriptor */
    0x07,                    //sizeof(USB_EP_DSC)
    USB_DESCRIPTOR_ENDPOINT, //Endpoint Descriptor
    _EP05_OUT,               //EndpointAddress
    _BULK,                   //Attributes
    64, 0x00,                //size
    1,                       //Interval
};

//Language code string descriptor
ROM struct __attribute__((packed))
{
    BYTE bLength;
    BYTE bDscType;
    WORD string[1];
} LanguageCodeDescriptor = {
    .bLength = sizeof(LanguageCodeDescriptor),
    .bDscType = USB_DESCRIPTOR_STRING,
    .string = {0x0409}};

//Manufacturer string descriptor
ROM struct __attribute__((packed))
{
    BYTE bLength;
    BYTE bDscType;
    WORD string[5];
} VendorNameDescriptor = {
    .bLength = sizeof(VendorNameDescriptor),
    .bDscType = USB_DESCRIPTOR_STRING,
    .string = {
        'V',
        'o',
        't',
        'e',
        'k',
    }};

//Product string descriptor
ROM struct __attribute__((packed))
{
    BYTE bLength;
    BYTE bDscType;
    WORD string[9];
} ProductNameDescriptor = {
    .bLength = sizeof(ProductNameDescriptor),
    .bDscType = USB_DESCRIPTOR_STRING,
    .string = {
        'L',
        'o',
        'd',
        'e',
        's',
        't',
        'o',
        'n',
        'e',
    }};

// Serial Number String
ROM struct __attribute__((packed))
{
    BYTE bLength;
    BYTE bDscType;
    WORD string[SERIAL_NUMBER_LENGTH];
} SerialNumberDescriptor = {
    .bLength = sizeof(SerialNumberDescriptor),
    .bDscType = USB_DESCRIPTOR_STRING,
    .string = {SERIAL_NUMBER}};

// Microsoft OS String Descriptor
ROM struct __attribute__((packed))
{
    BYTE bLength;
    BYTE bDscType;
    WORD string[OS_STRING_LENGTH];
} ROM USB_SD_OS = {
    .bLength = sizeof(USB_SD_OS),
    .bDscType = USB_DESCRIPTOR_STRING,
    .string = {OS_STRING}};

// Microsoft Compatible ID Feature Descriptor
ROM struct __attribute__((packed))
{
    // Header
    DWORD dwLength;
    WORD bcdVersion;
    WORD wIndex;
    BYTE bCount;
    BYTE bReserved1[7];
    // Function Section 1
    BYTE bFirstInterfaceNumber;
    BYTE bReserved2;
    CHAR bCompatibleID[8];
    BYTE bSubCompatibleID[8];
    BYTE bReserved3[6];
} ROM Ext_CID_OS_FD = {sizeof(Ext_CID_OS_FD), 0x0100, 0x0004, 0x01, {0}, 0x00, 0x01, "WINUSB", {0}, {0}};
// Microsoft Extended Properties Feature Descriptor
ROM struct __attribute__((packed))
{
    // Header
    DWORD dwLength;
    WORD bcdVersion;
    WORD wIndex;
    WORD wCount;
    // Custom Property Section 1
    DWORD dwSize;
    DWORD dwPropertyDataType;
    WORD wPropertyNameLength;
    WORD bPropertyName[20];
    DWORD dwPropertyDataLength;
    WORD bPropertyData[39];
} ROM Ext_P_OS_FD = {
    .dwLength = 0x0000008E,
    .bcdVersion = 0x0100,
    .wIndex = 0x0005,
    .wCount = 0x0001,
    .dwSize = 0x00000084,
    .dwPropertyDataType = 0x00000001,
    .wPropertyNameLength = 0x0028,
    .bPropertyName = {'D', 'e', 'v', 'i', 'c', 'e', 'I', 'n', 't', 'e', 'r', 'f', 'a', 'c', 'e', 'G', 'U', 'I', 'D'},
    .dwPropertyDataLength = 0x0000004E,
    .bPropertyData = {'{', '7', 'D', 'D', 'C', 'C', '5', 'E', '8', '-', 'F', 'A', 'D', 'E', '-', '4', 'D', '1', '1', '-', '9', '6', '9', '0', '-', '9', 'F', '8', '6', 'A', 'A', 'C', '6', '1', 'D', 'C', 'A', '}'}};
//{7DDCC5E8-FADE-4D11-9690-9F86AAC61DCA}
//Array of configuration descriptors
ROM BYTE *ROM USB_CD_Ptr[] =
    {
        (ROM BYTE * ROM) & configDescriptor1};

//Array of string descriptors
ROM BYTE *ROM USB_SD_Ptr[] =
    {
        (ROM BYTE * ROM) & LanguageCodeDescriptor,
        (ROM BYTE * ROM) & VendorNameDescriptor,
        (ROM BYTE * ROM) & ProductNameDescriptor,
        (ROM BYTE * ROM) & SerialNumberDescriptor,
};

/** EOF usb_descriptors.c ***************************************************/

#endif
