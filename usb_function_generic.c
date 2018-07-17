

#ifndef USBGEN_H
#define USBGEN_H

#include "GenericTypeDefs.h"
#include "usb_config.h"

#define USBGenWrite(ep,data,len) USBTxOnePacket(ep,data,len)
#define USBGenRead(ep,data,len) USBRxOnePacket(ep,data,len)

#endif //USBGEN_H
