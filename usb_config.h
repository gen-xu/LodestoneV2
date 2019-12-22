/*
********************************************************************************
                                                                                
Software License Agreement                                                      
                                                                                
Copyright � 2007-2008 Microchip Technology Inc.  All rights reserved.           
                                                                                
Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital signal controller  
that is integrated into your product or third party product (pursuant to the    
sublicense terms in the accompanying license agreement).                        
                                                                                
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
                                                                                
********************************************************************************
*/

// Created by the Microchip USBConfig Utility, Version 2.7.1.0, 1/14/2014, 15:52:58

#ifndef _usb_config_h_
#define _usb_config_h_

#include <p32xxxx.h>
#include "plib.h"
#define _USB_CONFIG_VERSION_MAJOR 2
#define _USB_CONFIG_VERSION_MINOR 7
#define _USB_CONFIG_VERSION_DOT   1
#define _USB_CONFIG_VERSION_BUILD 0

// Supported USB Configurations

#define USB_SUPPORT_DEVICE

// Hardware Configuration

#define USB_PING_PONG_MODE  USB_PING_PONG__FULL_PING_PONG

// Peripheral Configuration

#define USB_VID            0x04D8
#define USB_PID            0x000C
#define USB_INTERRUPT
#define USB_PULLUP_OPTION      USB_PULLUP_DISABLE
#define USB_TRANSCEIVER_OPTION USB_INTERNAL_TRANSCEIVER
#define USB_EP0_BUFF_SIZE     8
#define USB_MAX_NUM_INT       (0+1)
#define USB_MAX_EP_NUMBER 5
#define USB_NUM_STRING_DESCRIPTORS 4
#define SERIAL_NUMBER_LENGTH 6
#define SERIAL_NUMBER '0','0','0','0','0','A'

#define OS_STRING_LENGTH 8
#define OS_VENDOR_CODE   32
#define OS_STRING 'M','S','F','T','1','0','0', OS_VENDOR_CODE

//#define USB_DISABLE_SOF_HANDLER                 
//#define USB_DISABLE_ERROR_HANDLER               
//#define USB_DISABLE_SET_DESCRIPTOR_HANDLER      

#endif