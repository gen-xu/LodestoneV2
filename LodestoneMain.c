/*
 * File: LSMain.c
 * Author: George, Gen
 * v2.5
 * Created on December 21, 2019, 11:05 PM
 * Compiled with XC32 v2.30
 * External Oscillator: 8 MHz
 * System Frequency:   40 MHz
 */
//--------------------------------- INCLUDES -----------------------------------
#include <p32xxxx.h>
#include <math.h>
#include "usb.h"
#include <plib.h>
#include <stdlib.h>

//------------------------------- CONFIG BITS ----------------------------------
// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = OFF // Peripheral Module Ensable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = ON   // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = OFF // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = ON // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (1x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_2 // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = ON      // USB PLL Enable (Enabled)
#pragma config FPLLODIV = DIV_2 // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = PRI          // Oscillator Selection Bits (Fast RC Osc (FRC w/ PLL))
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON            // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = XT         // Primary Oscillator Configuration (External clock mode)
#pragma config OSCIOFNC = OFF       // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_1       // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSECME       // Clock Switching and Monitor Selection (Clock Switch Enable, FSCM Disabled)
#pragma config WDTPS = PS1048576    // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF         // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25 // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF      // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1 // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF         // Program Flash Write Protect (Disable)
#pragma config BWP = OFF         // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF          // Code Protect (Protection Disabled)

//--------------------------------- DEFINES ------------------------------------
#define ABS(N) ((N < 0) ? (-N) : (N))

#define X 0
#define Y 1
#define Z 2

#define FORWARD 1
#define REVERSE 0

//.3 - 400,000 Hz
#define _INPLANE_PWM 2222
#define _OUTPLANE_PWM 2222

#define X_IS_FORWARD LATBbits.LATB3
#define X_IS_REVERSE LATBbits.LATB4
#define Y_IS_FORWARD LATAbits.LATA4
#define Y_IS_REVERSE LATBbits.LATB7
#define Z_IS_FORWARD LATBbits.LATB8
#define Z_IS_REVERSE LATBbits.LATB9

#define XOVER_TIME 0.025 //In milliseconds (<1ms)

#define OFFSET 0 // Voltage read offset

#define V_SMOOTH 50

#define NVM_START_ADDR (void *)(0x9D010000)

//----------------------------- GLOBAL VARIABLES -------------------------------

float inplanePeriod = 1;
float outplanePeriod = 1;
int PBDivide = 0;
int timescale2 = 0;
int timescale3 = 0;
int period2 = 0;
int period3 = 0;
BOOL X_delaying = 0;
BOOL Y_delaying = 0;
BOOL Z_delaying = 0;
unsigned int XD_past = 0;
unsigned int XD_till = 0;
unsigned int YD_past = 0;
unsigned int YD_till = 0;
unsigned int ZD_past = 0;
unsigned int ZD_till = 0;
USB_HANDLE USBHandles[5] = {0, 0, 0, 0, 0};
float voltages[V_SMOOTH] = {};

const unsigned char TERMINATE_PACKET = 0;
float voltage = 0;
unsigned int controlMessage = 0;
float Magnet[3] = {0, 0, 0};

typedef union CONFIG {
    unsigned int _DWORD[14];
    struct
    {
        unsigned int inplane_pwm;
        unsigned int outplane_pwm;
        float px_k;
        float px_b;
        float nx_k;
        float nx_b;
        float py_k;
        float py_b;
        float ny_k;
        float ny_b;
        float pz_k;
        float pz_b;
        float nz_k;
        float nz_b;
    };
} __attribute__((packed)) CONFIG;
CONFIG config = {
    .inplane_pwm = _INPLANE_PWM,
    .outplane_pwm = _OUTPLANE_PWM,
    .px_k = 0,
    .px_b = 0,
    .nx_k = 0,
    .nx_b = 0,
    .py_k = 0,
    .py_b = 0,
    .ny_k = 0,
    .ny_b = 0,
    .pz_k = 0,
    .pz_b = 0,
    .nz_k = 0,
    .nz_b = 0};


typedef struct GUID {
  unsigned long  Data1;
  unsigned short Data2;
  unsigned short Data3;
  unsigned char  Data4[8];
} __attribute__((packed)) GUID;

// Default GUID
// {B64F14EC-BB39-46DE-9A9E-9DA7B1A14952}
static const GUID guid = 
{ 0xb64f14ec, 0xbb39, 0x46de, { 0x9a, 0x9e, 0x9d, 0xa7, 0xb1, 0xa1, 0x49, 0x52 } };

// ------------------------------ PROTOTYPES -----------------------------------
void Init(void);
void ResetPWM(void);
void SetPWM(int magnet, float magnitude);
void ReadInputVoltage(void);
void ReceiveMagnetValues(void);
void SendInputVoltage(void);
void SetDirection(int magnet, BOOL direction);
void SendConfig(void);
void ReceiveConfig(void);
void ReceiveMessage(void);
void SaveConfig(void);
void LoadConfig(void);
void USBRead(USB_HANDLE *, BYTE, BYTE *, WORD);
void USBWrite(USB_HANDLE *, BYTE, BYTE *, WORD);

//---------------------------------- MAIN --------------------------------------
void main(void)
{
    Init();
    USBDeviceAttach();
    while (USBDeviceState < CONFIGURED_STATE)
        ;
    LoadConfig();
    while (1)
    {
        ReadInputVoltage();
        SendInputVoltage();
        ReceiveMagnetValues();
        SetPWM(X, Magnet[X]);
        SetPWM(Y, Magnet[Y]);
        SetPWM(Z, Magnet[Z]);
        ReceiveMessage();
    }
}

// ------------------------------ FUNCTIONS ------------------------------------

void Init(void)
{
    //Initialize periods
    inplanePeriod = 1 / config.inplane_pwm;
    outplanePeriod = 1 / config.outplane_pwm;

    //Determine PBDivide
    if ((inplanePeriod <= (1 * 0.4194304)) && (outplanePeriod <= (1 * 0.4194304)))
        PBDivide = 1; //Divide by 1
    else if ((inplanePeriod <= (2 * 0.4194304)) && (outplanePeriod <= (2 * 0.4194304)))
        PBDivide = 2; //Divide by 2
    else if ((inplanePeriod <= (4 * 0.4194304)) && (outplanePeriod <= (4 * 0.4194304)))
        PBDivide = 4; //Divide by 4
    else
        PBDivide = 8; //Divide by 8

    //Determine timescales
    if (inplanePeriod <= (0.0016384 * PBDivide * 1))
        timescale2 = 1;
    else if (inplanePeriod <= (0.0016384 * PBDivide * 2))
        timescale2 = 2;
    else if (inplanePeriod <= (0.0016384 * PBDivide * 4))
        timescale2 = 4;
    else if (inplanePeriod <= (0.0016384 * PBDivide * 8))
        timescale2 = 8;
    else if (inplanePeriod <= (0.0016384 * PBDivide * 16))
        timescale2 = 16;
    else if (inplanePeriod <= (0.0016384 * PBDivide * 32))
        timescale2 = 32;
    else if (inplanePeriod <= (0.0016384 * PBDivide * 64))
        timescale2 = 64;
    else
        timescale2 = 256;

    if (outplanePeriod <= (0.0016384 * PBDivide * 1))
        timescale3 = 1;
    else if (outplanePeriod <= (0.0016384 * PBDivide * 2))
        timescale3 = 2;
    else if (outplanePeriod <= (0.0016384 * PBDivide * 4))
        timescale3 = 4;
    else if (outplanePeriod <= (0.0016384 * PBDivide * 8))
        timescale3 = 8;
    else if (outplanePeriod <= (0.0016384 * PBDivide * 16))
        timescale3 = 16;
    else if (outplanePeriod <= (0.0016384 * PBDivide * 32))
        timescale3 = 32;
    else if (outplanePeriod <= (0.0016384 * PBDivide * 64))
        timescale3 = 64;
    else
        timescale3 = 256;

    //Determine periods
    period2 = ((inplanePeriod * 40000000) / (timescale2 * PBDivide)) - 1;
    period3 = ((outplanePeriod * 40000000) / (timescale3 * PBDivide)) - 1;

    // --- I/O ---
    TRISA = 0x00000000;
    TRISB = 0x00002000;
    LATA = 0x00000000;
    LATB = 0x00000000;

    //Pin mapping
    RPA0R = 5; // Pin RPA0 is OC1
    RPA1R = 5; // Pin RPA1 is OC2
    RPB2R = 5; // Pin RPB2 is OC4

    // --- ANALOG ---
    ANSELA = 0x00000000;     // All GPIOA are digital
    ANSELB = 0x00000008;     // GPIOB3 is analog
    AD1CHS = 0x000B0000;     // ADC uses AN11 and VR- (MUX A)
    AD1CON1bits.FORM = 0;    // 16-bit Integer
    AD1CON1bits.SSRC = 0;    // Manual ADC conversion
    AD1CON1bits.CLRASAM = 1; // One ADC conversion at a time
    AD1CON1bits.ASAM = 1;    // Sampling begins when SAMP=1
    AD1CON1bits.DONE = 0;    // Clear done flag
    AD1CON2bits.VCFG = 0;    // Use AVSS for Vref-
    AD1CON2bits.SMPI = 0;    // Interrupt every sample
    AD1CON2bits.BUFM = 0;    // One 16-word buffer
    AD1CON2bits.ALTS = 0;    // Always use MUX A
    AD1CON3bits.ADRC = 0;    // Use PBCLK
    AD1CON3bits.SAMC = 10;

    AD1CON1bits.ADON = 1; // TURN ON ADC
    AD1CON1bits.SAMP = 1;

    // --- OSCILLATOR ---
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    while (OSCCONbits.PBDIVRDY != 1)
        ;
    switch (PBDivide)
    {
    case 1:
        OSCCONbits.PBDIV = 0;
        break;
    case 2:
        OSCCONbits.PBDIV = 1;
        break;
    case 4:
        OSCCONbits.PBDIV = 2;
        break;
    case 8:
        OSCCONbits.PBDIV = 3;
        break;
    default:
        break;
    }
    OSCCONSET = 0x00000300; // Select Posc without PLL
    OSCCONSET = 0x00000001;
    while (OSCCONbits.OSWEN == 1)
        ;

    OSCCONCLR = 0x00000004;
    OSCCONbits.SOSCEN = 0; // Disable secondary oscillator

    // --- PWM ---
    // Setup output compares in 32-bit mode (Timer 2, no fault)
    OC1CON = 0x0000000E;
    OC2CON = 0x00000006;
    OC4CON = 0x00000006;

    // Setup PWM timer (16-bit timer 2). Gate disabled
    T2CON = 0;
    T3CON = 0;
    PR2 = period2;
    PR3 = period3;
    switch (timescale2)
    {
    case 1:
        T2CONbits.TCKPS = 0;
        break;
    case 2:
        T2CONbits.TCKPS = 1;
        break;
    case 4:
        T2CONbits.TCKPS = 2;
        break;
    case 8:
        T2CONbits.TCKPS = 3;
        break;
    case 16:
        T2CONbits.TCKPS = 4;
        break;
    case 32:
        T2CONbits.TCKPS = 5;
        break;
    case 64:
        T2CONbits.TCKPS = 6;
        break;
    case 256:
        T2CONbits.TCKPS = 7;
        break;
    default:
        break;
    }
    switch (timescale3)
    {
    case 1:
        T3CONbits.TCKPS = 0;
        break;
    case 2:
        T3CONbits.TCKPS = 1;
        break;
    case 4:
        T3CONbits.TCKPS = 2;
        break;
    case 8:
        T3CONbits.TCKPS = 3;
        break;
    case 16:
        T3CONbits.TCKPS = 4;
        break;
    case 32:
        T3CONbits.TCKPS = 5;
        break;
    case 64:
        T3CONbits.TCKPS = 6;
        break;
    case 256:
        T3CONbits.TCKPS = 7;
        break;
    default:
        break;
    }

    // Initilize directions
    X_IS_FORWARD = 1;
    X_IS_REVERSE = 0;
    Y_IS_FORWARD = 1;
    Y_IS_REVERSE = 0;
    Z_IS_FORWARD = 1;
    Z_IS_REVERSE = 0;

    // Setup delay timer (1 ms period)
    T1CON = 0;
    PR1 = 40000 / PBDivide;

    // Activate PWM
    T1CONbits.ON = 1;
    T2CONbits.ON = 1;
    T3CONbits.ON = 1;
    OC1CONbits.ON = 1;
    OC2CONbits.ON = 1;
    OC4CONbits.ON = 1;

    USBDeviceInit();
}

// magnitude is now just expected voltage
void SetPWM(int magnet, float magnitude)
{
    SetDirection(magnet, magnitude > 0 ? FORWARD : REVERSE);
    magnitude = ABS(magnitude / voltage);
    if (magnitude > 1)
    {
        magnitude = 1;
    }
    switch (magnet)
    {
    case X:
        OC4RS = (unsigned long)(magnitude * PR2);
        break;
    case Y:
        OC2RS = (unsigned long)(magnitude * PR2);
        break;
    case Z:
        OC1RS = (unsigned long)(magnitude * PR3);
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
// readVoltage                                                                //

//   This function uses the ADC to read the voltage supplied to the           //
//   H-bridges.  It both returns the voltage as a double, and sets the        //
//   global variable "voltage" (double).                                      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void ReadInputVoltage(void)
{
    double tempVoltage = 0;
    float maxVoltage = 36.6967935;
    float average = 0; //For average voltage calculation (V_SMOOTH)
    int i = 0;         //For average voltage calculation (V_SMOOTH)
    unsigned long multiplier;

    AD1CON1bits.SAMP = 1;
    AD1CON1bits.SAMP = 0;
    while (AD1CON1bits.DONE == 0)
        ;
    multiplier = ADC1BUF0;
    multiplier += OFFSET;
    tempVoltage = maxVoltage * multiplier;
    tempVoltage /= 1024;
    if (tempVoltage < .1)
        tempVoltage = 0;
    for (i = (V_SMOOTH - 1); i > 0; i--)
    {
        voltages[i] = voltages[i - 1];
        average += voltages[i];
    }
    voltages[0] = tempVoltage;
    average += tempVoltage;
    average /= V_SMOOTH;

    voltage = average;
}

void ResetPWM()
{
    T2CONbits.ON = 0;
    T2CONbits.ON = 0;
    OC1CONbits.ON = 0;
    OC2CONbits.ON = 0;
    OC4CONbits.ON = 0;
    PR2 = (40000000 / (timescale2 * PBDivide * config.inplane_pwm)) - 1;
    PR3 = (40000000 / (timescale2 * PBDivide * config.outplane_pwm)) - 1;
    TMR2 = 0;
    TMR3 = 0;
    T2CONbits.ON = 1;
    T2CONbits.ON = 1;
    OC1CONbits.ON = 1;
    OC2CONbits.ON = 1;
    OC4CONbits.ON = 1;
}

void USBWrite(USB_HANDLE *handle, BYTE endpoint, BYTE *data, WORD data_size)
{
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
        return;
    if (!USBHandleBusy(*handle))
    {
        *handle = USBTxOnePacket(endpoint, data, data_size);
    }
}

void USBRead(USB_HANDLE *handle, BYTE endpoint, BYTE *data, WORD data_size)
{
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
        return;
    if (!USBHandleBusy(*handle))
    {
        *handle = USBRxOnePacket(endpoint, data, data_size);
    }
}

void ReceiveMagnetValues(void)
{
    USBRead(&USBHandles[0], 1, (BYTE*)Magnet, 12);
}

void SendInputVoltage(void)
{
    USBWrite(&USBHandles[1], 2, (BYTE*)&voltage, 4);
}

void ReceiveMessage(void)
{
    USBRead(&USBHandles[2], 3, (BYTE*)&controlMessage, 4);
    switch (controlMessage)
    {
    case 1:
        SendConfig();
        break;
    case 2:
        ReceiveConfig();
        break;
    default:
        break;
    }
    controlMessage = 0;
}

void SendConfig(void)
{
    USBWrite(&USBHandles[3], 4, (BYTE*)&config, 56);
    USBWrite(&USBHandles[3], 4, (BYTE*)&TERMINATE_PACKET, 0);
}

void ReceiveConfig(void)
{
    USBRead(&USBHandles[4], 5, (BYTE*)&config, 56);
    SaveConfig();
    ResetPWM();
}

void SaveConfig(void)
{
    unsigned int res = 1;
    unsigned offset;
    NVMErasePage(NVM_START_ADDR);
    for (offset = 0; offset < 56; offset += 4)
    {
        res = NVMWriteWord((NVM_START_ADDR + offset), config._DWORD[offset / 4]);
    }
}

void LoadConfig(void)
{
    unsigned offset;
    for (offset = 0; offset < 56; offset += 4)
    {
        config._DWORD[offset / 4] = *(unsigned int *)(NVM_START_ADDR + offset);
    }
    for (offset = 0; offset < 56; offset += 4)
    {
        if (*(unsigned int *)(NVM_START_ADDR + offset) == *(unsigned int *)(NVM_START_ADDR + 56))
        {
            config.inplane_pwm = _INPLANE_PWM;
            config.outplane_pwm = _OUTPLANE_PWM;
            for (offset = 2; offset < 14; ++offset)
            {
                config._DWORD[offset] = 0;
            }
            SaveConfig();
            break;
        }
    }
}

void SetDirection(int magnet, BOOL direction)
{
    switch (magnet)
    {
    case X:
        if (!X_delaying && (direction ^ X_IS_FORWARD))
        {
            OC4CONbits.ON = 0;
            LATBbits.LATB2 = 0;
            if (direction == FORWARD)
            {
                LATBCLR = BIT_4;
                LATBSET = BIT_3;
            }
            if (!direction)
            {
                LATBCLR = BIT_3;
                LATBSET = BIT_4;
            }
            XD_past = TMR1;
            XD_till = XD_past + PR1 * XOVER_TIME;
            if (XD_till >= PR1)
                XD_till -= PR1;
            X_delaying = 1;
        }
        else if (X_delaying)
        {
            if (((XD_past > XD_till) && (TMR1 < XD_past) && (TMR1 >= XD_till)) || ((XD_till > XD_past) && (TMR1 > XD_till)))
            {
                TMR2 = 0;
                OC4CONbits.ON = 1;
                X_delaying = 0;
            }
        }
        break;

    case Y:
        if (!Y_delaying && ((!direction && Y_IS_FORWARD) || (direction && Y_IS_REVERSE)))
        {
            OC2CONbits.ON = 0;
            LATAbits.LATA1 = 0;
            if (direction == FORWARD)
            {
                LATBCLR = BIT_7;
                LATASET = BIT_4;
            }
            if (direction == REVERSE)
            {
                LATACLR = BIT_4;
                LATBSET = BIT_7;
            }
            YD_past = TMR1;
            YD_till = YD_past + PR1 * XOVER_TIME;
            if (YD_till >= PR1)
                YD_till -= PR1;
            Y_delaying = 1;
        }
        else if (Y_delaying)
        {
            if (((YD_past > YD_till) && (TMR1 < YD_past) && (TMR1 >= YD_till)) || ((YD_till > YD_past) && (TMR1 > YD_till)))
            {
                TMR2 = 0;
                OC2CONbits.ON = 1;
                Y_delaying = 0;
            }
        }
        break;

    case Z:
        if (!Z_delaying && ((!direction && Z_IS_FORWARD) || (direction && Z_IS_REVERSE)))
        {
            OC1CONbits.ON = 0;
            LATAbits.LATA0 = 0;
            if (direction == FORWARD)
            {
                LATBCLR = BIT_9;
                LATBSET = BIT_8;
            }
            if (direction == REVERSE)
            {
                LATBCLR = BIT_8;
                LATBSET = BIT_9;
            }
            ZD_past = TMR1;
            ZD_till = ZD_past + PR1 * XOVER_TIME;
            if (ZD_till >= PR1)
                ZD_till -= PR1;
            Z_delaying = 1;
            break;
        }
        else if (Z_delaying)
        {
            if (((ZD_past > ZD_till) && (TMR1 < ZD_past) && (TMR1 >= ZD_till)) || ((ZD_till > ZD_past) && (TMR1 > ZD_till)))
            {
                TMR3 = 0;
                OC1CONbits.ON = 1;
                Z_delaying = 0;
            }
        }
        break;

    default:
        break;
    }
}

/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 *                  SET_CONFIGURATION (wValue not = 0) request.  This
 *                  callback function should initialize the endpoints
 *                  for the device's usage according to the current
 *                  configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP()
{
    //Pipe 1: Position message to Lodestone
    USBEnableEndpoint(1, USB_OUT_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    //Pipe 2: Voltage message to MagMaestro
    USBEnableEndpoint(2, USB_IN_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    //Pipe 3: Control Byte
    USBEnableEndpoint(3, USB_OUT_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    //Pipe 4: Bulk message to MagMaestro
    USBEnableEndpoint(4, USB_IN_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    //Pipe 5: Bulk message to Lodestone
    USBEnableEndpoint(5, USB_OUT_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
}

/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *                  called when a SETUP, bRequest: SET_DESCRIPTOR request
 *                  arrives.  Typically SET_DESCRIPTOR requests are
 *                  not used in most applications, and it is
 *                  optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler() {}

/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 *                  firmware must process the request and respond
 *                  appropriately to fulfill the request.  Some of
 *                  the SETUP packets will be for standard
 *                  USB "chapter 9" (as in, fulfilling chapter 9 of
 *                  the official USB specifications) requests, while
 *                  others may be specific to the USB device class
 *                  that is being implemented.  For example, a HID
 *                  class device needs to be able to respond to
 *                  "GET REPORT" type of requests.  This
 *                  is not a standard USB chapter 9 request, and
 *                  therefore not handled by usb_device.c.  Instead
 *                  this request should be handled by class specific
 *                  firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq()
{
    //Other messages to EP0
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF (Start Of Frame)
 *                  packet to full-speed devices every 1 ms.
 *                  This interrupt may be useful for isochronous pipes.
 *                  End designers should implement callback routine
 *                  as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler()
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend()
{
    //Example power saving code.  Insert appropriate code here for the desired
    //application behavior.  If the microcontroller will be put to sleep, a
    //process similar to that shown below may be used:

    //ConfigureIOPinsForLowPower();
    //SaveStateOfAllInterruptEnableBits();
    //DisableAllInterruptEnableBits();
    //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();
    //^^ should enable at least USBActivityIF as a wake source
    //Sleep();
    //RestoreStateOfAllPreviouslySavedInterruptEnableBits();
    //^^Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
    //RestoreIOPinsToNormal();
    //^^Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

    //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
    //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
    //things to not work as intended.

#if defined(__C30__) || defined __XC16__
#if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        TRISA &= 0xFF3F;
        LATAbits.LATA6 = 1;
        Sleep();
        LATAbits.LATA6 = 0;
#endif
#endif
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *                  suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *                  mode, the host may wake the device back up by sending non-
 *                  idle state signalling.
 *
 *                  This call back is invoked when a wakeup from USB suspend
 *                  is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend()
{
    // If clock switching or other power savings measures were taken when
    // executing the USBCBSuspend() function, now would be a good time to
    // switch back to normal full power run mode conditions.  The host allows
    // a few milliseconds of wakeup time, after which the device must be
    // fully back to normal, and capable of receiving and processing USB
    // packets.  In order to do this, the USB module must receive proper
    // clocking (IE: 48MHz clock must be available to SIE for full speed USB
    // operation).
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler()
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

    // Typically, user firmware does not need to do anything special
    // if a USB error occurs.  For example, if the host sends an OUT
    // packet to your device, but the packet gets corrupted (ex:
    // because of a bad connection, or the user unplugs the
    // USB cable during the transmission) this will typically set
    // one or more USB error interrupt flags.  Nothing specific
    // needs to be done however, since the SIE will automatically
    // send a "NAK" packet to the host.  In response to this, the
    // host will normally retry to send the packet again, and no
    // data loss occurs.  The system will typically recover
    // automatically, without the need for application firmware
    // intervention.

    // Nevertheless, this callback function is provided, such as
    // for debugging purposes.
}