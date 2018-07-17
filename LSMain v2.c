/*
 * File:   LSMain.c
 * Author: 
 *
 * v2.1
 *
 * Created on October 9, 2013, 4:20 PM
 *
 * Compiled with XC32 v1.34
 * 
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
#pragma config PMDL1WAY = OFF       // Peripheral Module Ensable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = ON         // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = OFF       // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = ON       // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2     // PLL Input Divider (1x Divider)
#pragma config FPLLMUL = MUL_20     // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_2     // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = ON          // USB PLL Enable (Enabled)
#pragma config FPLLODIV = DIV_2     // System PLL Output Clock Divider (PLL Divide by 1)

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
#pragma config JTAGEN = OFF         // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1    // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF            // Program Flash Write Protect (Disable)
#pragma config BWP = OFF            // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF             // Code Protect (Protection Disabled)


//--------------------------------- DEFINES ------------------------------------
#define MAGX        OC4RS         // X-axis
#define MAGY        OC2RS         // Y-axis
#define MAGZ        OC1RS         // Z-axis

#define	M_PI		3.14159265358979323846	/* pi */

#define X           1
#define Y           2
#define Z           3

#define FORWARD     1
#define REVERSE     0

//.3 - 400,000 Hz
#define _INPLANE_HZ   2000
#define _OUTPLANE_HZ  2000
#define _RX           1.88    // ohms
#define _RY           1.9     // ohms
#define _RZ           1.2     // ohms
#define _OFFSET_XF    0.33
#define _OFFSET_XR    0.3
#define _OFFSET_YF    0.34
#define _OFFSET_YR    0.33
#define _OFFSET_ZF    0.5
#define _OFFSET_ZR    0.5
#define _X_OETOA      28
#define _Y_OETOA      28
#define _Z_OETOA      33

#define X_IS_FORWARD LATBbits.LATB3
#define X_IS_REVERSE LATBbits.LATB4
#define Y_IS_FORWARD LATAbits.LATA4
#define Y_IS_REVERSE LATBbits.LATB7
#define Z_IS_FORWARD LATBbits.LATB8
#define Z_IS_REVERSE LATBbits.LATB9

#define XOVER_TIME   0.025 //In milliseconds (<1ms)


#define OFFSET      0       // Voltage read offset

#define V_SMOOTH    50

//----------------------------- GLOBAL VARIABLES -------------------------------
float MagX_Curr      = 0;
float MagY_Curr      = 0;
float MagZ_Curr      = 0;
float MagX_Prev      = 0;
float MagY_Prev      = 0;
float MagZ_Prev      = 0;
float inplanePeriod  = 1;
float outplanePeriod = 1;
float voltage        = 0;
int PBDivide         = 0;
int timescale2       = 0;
int timescale3       = 0;
int period2          = 0;
int period3          = 0;
BOOL X_delaying      = 0;
BOOL Y_delaying      = 0;
BOOL Z_delaying      = 0;
BOOL QP_Flag         = 0;
unsigned int XD_past = 0;
unsigned int XD_till = 0;
unsigned int YD_past = 0;
unsigned int YD_till = 0;
unsigned int ZD_past = 0;
unsigned int ZD_till = 0;
unsigned int QP_past = 0;
unsigned int QP_till = 0;
unsigned int QP_pos  = 0;
float QP_max         = 0;
BOOL QP_doneFlag     = 0;
BOOL QP_clockBack    = 0;   //Set if Timer end position is smaller than start position

unsigned int INPLANE_HZ  = _INPLANE_HZ;
unsigned int OUTPLANE_HZ = _OUTPLANE_HZ;
float RX                 = _RX;
float RY                 = _RY;
float RZ                 = _RZ;
float OFFSET_XF          = _OFFSET_XF;
float OFFSET_XR          = _OFFSET_XR;
float OFFSET_YF          = _OFFSET_YF;
float OFFSET_YR          = _OFFSET_YR;
float OFFSET_ZF          = _OFFSET_ZF;
float OFFSET_ZR          = _OFFSET_ZR;
float X_OETOA            = _X_OETOA;
float Y_OETOA            = _Y_OETOA;
float Z_OETOA            = _Z_OETOA;
float qp_frequency       = _INPLANE_HZ;

#ifdef V_SMOOTH
float voltages[V_SMOOTH] = {};
#endif

const unsigned char TERMINATE_PACKET = 0;
// ------------------------------ STRUCTURES -----------------------------------
typedef union floatCondition
{
    float fHolder;
    unsigned int intHolder;
} floatCondition;

typedef union MAG_PACKET
{
    char _byte[12];
    struct
    {
        float USB_MagX;
        float USB_MagY;
        float USB_MagZ;
    };
} MAG_PACKET;
MAG_PACKET MAGPacket;

USB_HANDLE USBGenericOutHandle1 = 0;

typedef union VOLT_PACKET
{
    char _byte[4];
} VOLT_PACKET;
VOLT_PACKET INPacket;

USB_HANDLE USBGenericInHandle2 = 0;


USB_HANDLE USBGenericOutHandle3 = 0;

typedef union CONFIG_PACKET
{
    char _byte[56];
    struct
    {
        unsigned int inplane_hz;
        unsigned int outplane_hz;
        float rx;
        float ry;
        float rz;
        float offset_xf;
        float offset_xr;
        float offset_yf;
        float offset_yr;
        float offset_zf;
        float offset_zr;
        float X_OeToA;
        float Y_OeToA;
        float Z_OeToA;
    };
} CONFIG_PACKET;
CONFIG_PACKET configPacket;

typedef union QP_PACKET
{
    char _byte[13];
    struct
    {
        float inplane;
        float outplane;
        float period;
        char  CW_Flag;
    };
} QP_PACKET;
QP_PACKET QPPacket;
QP_PACKET emptyQPPacket = {0, 0, 0, 0};

char control_message = 0;

USB_HANDLE USBGenericInHandle4 = 0;

USB_HANDLE USBGenericOutHandle5 = 0;

// ------------------------------ PROTOTYPES -----------------------------------
void initPIC(void);
void PWM_Out(int magnet, float magnitude);
double readVoltage(void);
void readMagnetValues(void);
void sendVoltage(void);
void HighPriorityISR(void);
void LowPriorityISR(void);
void setDirection(int magnet, BOOL direction);
void quickPrecession(void);
void sendConfig(void);
void getConfig(void);
void QPstart(void);
void readMessages(void);

void flashWrite(void);
void flashRead(void);
extern unsigned int _NVMOperation(unsigned int nvmop);

//---------------------------------- MAIN --------------------------------------
void main(void)
{
    initPIC();

    USBDeviceAttach();
    while(USBDeviceState < CONFIGURED_STATE);
    PWM_Out(X, 0);
    PWM_Out(Y, 0);
    PWM_Out(Z, 0);
    flashRead();
    while(1)
    {
        if(QP_Flag)
        {
            quickPrecession();
            readMessages();
            if(!QP_clockBack)
            {
                if(TMR1 > QP_till)                          QP_doneFlag = TRUE;
            }
            else
            {
                if((TMR1 < QP_past) && (TMR1 > QP_till))    QP_doneFlag = TRUE;
            }
            PWM_Out(X, MagX_Curr);
            if(!QP_clockBack)
            {
                if(TMR1 > QP_till)                          QP_doneFlag = TRUE;
            }
            else
            {
                if((TMR1 < QP_past) && (TMR1 > QP_till))    QP_doneFlag = TRUE;
            }
            PWM_Out(Y, MagY_Curr);

            if(QP_doneFlag)
            {
                if(QP_pos < QP_max)     QP_pos++;
                else                    QP_pos = 0;
                QP_past = QP_till;
                QP_till = QP_past + PR1/(((unsigned int)qp_frequency)/1000);
                if(QP_till > PR1)   QP_till -= PR1;
                if(QP_till < QP_past)   QP_clockBack = TRUE;
                else                    QP_clockBack = FALSE;
            }

            if(!QP_clockBack)
                while(TMR1 < QP_till)
                {
                    if(X_delaying)  PWM_Out(X, MagX_Curr);
                    if(Y_delaying)  PWM_Out(Y, MagY_Curr);
                }
            else
                while((TMR1 > QP_past) || (TMR1 < QP_till))
                {
                    if(X_delaying)  PWM_Out(X, MagX_Curr);
                    if(Y_delaying)  PWM_Out(Y, MagY_Curr);
                }
        }
        else
        {
            voltage = readVoltage();
            sendVoltage();
            readMagnetValues();
            PWM_Out(X, MagX_Curr);
            PWM_Out(Y, MagY_Curr);
            PWM_Out(Z, MagZ_Curr);
            readMessages();
        }
    }
}

// ------------------------------ FUNCTIONS ------------------------------------

////////////////////////////////////////////////////////////////////////////////
// initPIC                                                                    //
//   This function initializes IO, Interrupt settings, and timers.            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void initPIC(void)
{
    //Initialize periods
    inplanePeriod  = 1/INPLANE_HZ;
    outplanePeriod = 1/OUTPLANE_HZ;

    //Determine PBDivide
    if      ((inplanePeriod <= (1*0.4194304)) && (outplanePeriod <= (1*0.4194304)))
        PBDivide = 1;   //Divide by 1
    else if ((inplanePeriod <= (2*0.4194304)) && (outplanePeriod <= (2*0.4194304)))
        PBDivide = 2;   //Divide by 2
    else if ((inplanePeriod <= (4*0.4194304)) && (outplanePeriod <= (4*0.4194304)))
        PBDivide = 4;   //Divide by 4
    else
        PBDivide = 8;   //Divide by 8

    //Determine timescales
    if      (inplanePeriod <= (0.0016384*PBDivide*1))     timescale2 = 1;
    else if (inplanePeriod <= (0.0016384*PBDivide*2))     timescale2 = 2;
    else if (inplanePeriod <= (0.0016384*PBDivide*4))     timescale2 = 4;
    else if (inplanePeriod <= (0.0016384*PBDivide*8))     timescale2 = 8;
    else if (inplanePeriod <= (0.0016384*PBDivide*16))    timescale2 = 16;
    else if (inplanePeriod <= (0.0016384*PBDivide*32))    timescale2 = 32;
    else if (inplanePeriod <= (0.0016384*PBDivide*64))    timescale2 = 64;
    else                                                  timescale2 = 256;

    if      (outplanePeriod <= (0.0016384*PBDivide*1))    timescale3 = 1;
    else if (outplanePeriod <= (0.0016384*PBDivide*2))    timescale3 = 2;
    else if (outplanePeriod <= (0.0016384*PBDivide*4))    timescale3 = 4;
    else if (outplanePeriod <= (0.0016384*PBDivide*8))    timescale3 = 8;
    else if (outplanePeriod <= (0.0016384*PBDivide*16))   timescale3 = 16;
    else if (outplanePeriod <= (0.0016384*PBDivide*32))   timescale3 = 32;
    else if (outplanePeriod <= (0.0016384*PBDivide*64))   timescale3 = 64;
    else                                                  timescale3 = 256;

    //Determine periods
    period2 = (( inplanePeriod*40000000)/(timescale2*PBDivide)) - 1;
    period3 = ((outplanePeriod*40000000)/(timescale3*PBDivide)) - 1;

    // --- I/O ---
    TRISA  = 0x00000000;
    TRISB  = 0x00002000;
    LATA   = 0x00000000;
    LATB   = 0x00000000;

    //Pin mapping
    RPA0R = 5;                                  // Pin RPA0 is OC1
    RPA1R = 5;                                  // Pin RPA1 is OC2
    RPB2R = 5;                                  // Pin RPB2 is OC4

    // --- ANALOG ---
    ANSELA              = 0x00000000;           // All GPIOA are digital
    ANSELB              = 0x00000008;           // GPIOB3 is analog
    AD1CHS              = 0x000B0000;           // ADC uses AN11 and VR- (MUX A)
    AD1CON1bits.FORM    = 0;                    // 16-bit Integer
    AD1CON1bits.SSRC    = 0;                    // Manual ADC conversion
    AD1CON1bits.CLRASAM = 1;                    // One ADC conversion at a time
    AD1CON1bits.ASAM    = 1;                    // Sampling begins when SAMP=1
    AD1CON1bits.DONE    = 0;                    // Clear done flag
    AD1CON2bits.VCFG    = 0;                    // Use AVSS for Vref-
    AD1CON2bits.SMPI    = 0;                    // Interrupt every sample
    AD1CON2bits.BUFM    = 0;                    // One 16-word buffer
    AD1CON2bits.ALTS    = 0;                    // Always use MUX A
    AD1CON3bits.ADRC    = 0;                    // Use PBCLK
    AD1CON3bits.SAMC    = 10;

    AD1CON1bits.ADON    = 1;                    // TURN ON ADC
    AD1CON1bits.SAMP    = 1;

    // --- OSCILLATOR ---
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    while(OSCCONbits.PBDIVRDY != 1);
    switch(PBDivide)
    {
        case 1:     OSCCONbits.PBDIV = 0;   break;
        case 2:     OSCCONbits.PBDIV = 1;   break;
        case 4:     OSCCONbits.PBDIV = 2;   break;
        case 8:     OSCCONbits.PBDIV = 3;   break;
        default:    break;
    }
    OSCCONSET   = 0x00000300;          // Select Posc without PLL
    OSCCONSET   = 0x00000001;
    while(OSCCONbits.OSWEN == 1);

    OSCCONCLR = 0x00000004;
    OSCCONbits.SOSCEN = 0;          // Disable secondary oscillator

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
    switch(timescale2)
    {
        case 1:     T2CONbits.TCKPS = 0;    break;
        case 2:     T2CONbits.TCKPS = 1;    break;
        case 4:     T2CONbits.TCKPS = 2;    break;
        case 8:     T2CONbits.TCKPS = 3;    break;
        case 16:    T2CONbits.TCKPS = 4;    break;
        case 32:    T2CONbits.TCKPS = 5;    break;
        case 64:    T2CONbits.TCKPS = 6;    break;
        case 256:   T2CONbits.TCKPS = 7;    break;
        default:    break;
    }
    switch(timescale3)
    {
        case 1:     T3CONbits.TCKPS = 0;    break;
        case 2:     T3CONbits.TCKPS = 1;    break;
        case 4:     T3CONbits.TCKPS = 2;    break;
        case 8:     T3CONbits.TCKPS = 3;    break;
        case 16:    T3CONbits.TCKPS = 4;    break;
        case 32:    T3CONbits.TCKPS = 5;    break;
        case 64:    T3CONbits.TCKPS = 6;    break;
        case 256:   T3CONbits.TCKPS = 7;    break;
        default:    break;
    }

    // Initilize directions
    X_IS_FORWARD = 1;
    Y_IS_FORWARD = 1;
    Z_IS_FORWARD = 1;

    // Setup delay timer (1 ms period)
    T1CON = 0;
    PR1 = 40000/PBDivide;

    // Activate PWM
    T1CONbits.ON = 1;
    T2CONbits.ON = 1;
    T3CONbits.ON = 1;
    OC1CONbits.ON = 1;
    OC2CONbits.ON = 1;
    OC4CONbits.ON = 1;

    USBDeviceInit();
}

////////////////////////////////////////////////////////////////////////////////
// PWM_Out                                                                    //
//   This function outputs a PWM voltage to a corresponding magnet in a       //
//   particular direction.                                                    //
//                                                                            //
//   Parameters:                                                              //
//      - (int) magnet: The magnet selection (X, Y, or Z) (defined ints)      //
//      - (float) magnitude: The desired output current (Amps)                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void PWM_Out(int magnet, float magnitude)
{
    unsigned long output = 0;
    float offset = 0;
    switch(magnet)
    {
        case X:
                // Apply direction
            if(magnitude >= 0)      {setDirection(X, FORWARD);                  offset = OFFSET_XF;}
            else                    {setDirection(X, REVERSE); magnitude *= -1; offset = OFFSET_XR;}
                // Scale current to max current
            if((voltage > 4) && (magnitude > 0.01))
                    magnitude = (RX/voltage)*(magnitude+offset);
            else
                magnitude = 0;
            if(magnitude > 1)       magnitude = 1;
                // Multiply by max period of PWM
            output = magnitude*PR2;
                // Write to magnet
            MAGX = output;
            break;

        case Y:
            if(magnitude >= 0)      {setDirection(Y, FORWARD);                  offset = OFFSET_YF;}
            else                    {setDirection(Y, REVERSE); magnitude *= -1; offset = OFFSET_YR;}
            if((voltage > 4) && (magnitude > 0.01))
                magnitude = (RY/voltage)*(magnitude+offset);
            else
                magnitude = 0;
            if(magnitude > 1)       magnitude = 1;
            output = magnitude*PR2;
            MAGY = output;
            break;

        case Z:
            if(magnitude >= 0)      {setDirection(Z, FORWARD);                  offset = OFFSET_ZF;}
            else                    {setDirection(Z, REVERSE); magnitude *= -1; offset = OFFSET_ZR;}
            if((voltage > 4) && (magnitude > 0.01))
                magnitude = (RZ/voltage)*(magnitude+offset);
            else
                magnitude = 0;
            if(magnitude > 1)       magnitude = 1;
            output = magnitude*PR3;
            MAGZ = output;
            break;

        default:
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

double readVoltage(void)
{
    double tempVoltage = 0;
    float maxVoltage = 36.6967935;
    float average = 0;              //For average voltage calculation (V_SMOOTH)
    int i = 0;                      //For average voltage calculation (V_SMOOTH)
    unsigned long multiplier;

    AD1CON1bits.SAMP = 1;
    AD1CON1bits.SAMP = 0;
    while(AD1CON1bits.DONE == 0);
    multiplier = ADC1BUF0;
    multiplier += OFFSET;
    tempVoltage = maxVoltage*multiplier;
    tempVoltage /= 1024;
    if(tempVoltage < .1)
        tempVoltage = 0;

#ifdef V_SMOOTH
    for(i=(V_SMOOTH-1); i>0; i--)
    {
        voltages[i] = voltages[i-1];
        average += voltages[i];
    }
    voltages[0] = tempVoltage;
    average += tempVoltage;
    average /= V_SMOOTH;

    return average;
#endif

    return tempVoltage;
}

void readMagnetValues(void)
{
    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl==1))   return;
    if(!USBHandleBusy(USBGenericOutHandle1))
    {
        USBGenericOutHandle1 = USBGenRead(1, (BYTE*) &MAGPacket, 12);
    }

    MagX_Curr = MAGPacket.USB_MagX;
    MagY_Curr = MAGPacket.USB_MagY;
    MagZ_Curr = MAGPacket.USB_MagZ;
}

void sendVoltage(void)
{
    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl==1))   return;
    if(!USBHandleBusy(USBGenericInHandle2))
    {
        USBGenericInHandle2 = USBGenWrite(2, (BYTE*) &voltage, 4);
    }
}

void sendConfig(void)
{
    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl==1))   return;
    if(!USBHandleBusy(USBGenericInHandle4))
    {
        flashRead();
        configPacket.inplane_hz  = INPLANE_HZ;
        configPacket.outplane_hz = OUTPLANE_HZ;
        configPacket.rx          = RX;
        configPacket.ry          = RY;
        configPacket.rz          = RZ;
        configPacket.offset_xf   = OFFSET_XF;
        configPacket.offset_xr   = OFFSET_XR;
        configPacket.offset_yf   = OFFSET_YF;
        configPacket.offset_yr   = OFFSET_YR;
        configPacket.offset_zf   = OFFSET_ZF;
        configPacket.offset_zr   = OFFSET_ZR;
        configPacket.X_OeToA     = X_OETOA;
        configPacket.Y_OeToA     = Y_OETOA;
        configPacket.Z_OeToA     = Z_OETOA;

        USBGenericInHandle4 = USBGenWrite(4, (BYTE*) &configPacket, 56);
        USBGenericInHandle4 = USBGenWrite(4, (BYTE*) &TERMINATE_PACKET, 0);
    }
}

void getConfig(void)
{
    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl==1))   return;
    if(!USBHandleBusy(USBGenericOutHandle5))
    {
        USBGenericOutHandle5 = USBGenRead(5, (BYTE*) &configPacket, 56);
        INPLANE_HZ  = configPacket.inplane_hz;
        OUTPLANE_HZ = configPacket.outplane_hz;
        RX          = configPacket.rx;
        RY          = configPacket.ry;
        RZ          = configPacket.rz;
        OFFSET_XF   = configPacket.offset_xf;
        OFFSET_XR   = configPacket.offset_xr;
        OFFSET_YF   = configPacket.offset_yf;
        OFFSET_YR   = configPacket.offset_yr;
        OFFSET_ZF   = configPacket.offset_zf;
        OFFSET_ZR   = configPacket.offset_zr;
        X_OETOA     = configPacket.X_OeToA;
        Y_OETOA     = configPacket.Y_OeToA;
        Z_OETOA     = configPacket.Z_OeToA;

        flashWrite();

        T2CONbits.ON  = 0;
        T2CONbits.ON  = 0;
        OC1CONbits.ON = 0;
        OC2CONbits.ON = 0;
        OC4CONbits.ON = 0;
        PR2 = (40000000/(timescale2*PBDivide*INPLANE_HZ )) - 1;
        PR3 = (40000000/(timescale2*PBDivide*OUTPLANE_HZ)) - 1;
        TMR2 = 0;
        TMR3 = 0;
        T2CONbits.ON  = 1;
        T2CONbits.ON  = 1;
        OC1CONbits.ON = 1;
        OC2CONbits.ON = 1;
        OC4CONbits.ON = 1;

        qp_frequency = INPLANE_HZ;
    }
}

void QPstart(void)
{
    float QPperiod = 0;
    float QPoutPlane = 100;
    QPPacket = emptyQPPacket;

    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl==1))   return;
    if(!USBHandleBusy(USBGenericOutHandle5))
    {
        QP_Flag = 1;
        USBGenericOutHandle5 = USBGenRead(5, (BYTE*) &QPPacket, 13);
        while(QPPacket.period == 0);
        QPperiod = QPPacket.period;
        QPoutPlane = QPPacket.outplane;
        QP_max = qp_frequency*QPperiod;
        QP_pos = 0;
        PWM_Out(Z, QPoutPlane);
        while(Z_delaying)   PWM_Out(Z, QPoutPlane);
        MagZ_Curr = QPoutPlane;
        QP_till = TMR1;
    }
}

void readMessages(void)
{
    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl==1))   return;
    if(!USBHandleBusy(USBGenericOutHandle3))
    {
        USBGenericOutHandle3 = USBGenRead(3, (BYTE*) &control_message, 1);
        switch(control_message)
        {
            case 1:
                sendConfig();
                break;
            case 2:
                getConfig();
                break;
            case 3:
                QPstart();
                break;
            case 4:
                QP_Flag = 0;
                break;
            default:
                //DO NOTHING
                break;
        }
        control_message = 0;
    }
}

void flashWrite(void)
{
    unsigned int res = 1;
    floatCondition converter;

    NVMADDR = 0x1D010000;
    res = _NVMOperation(0x4004);
    //converter.fHolder = INPLANE_HZ;
    //NVMDATA = converter.intHolder;
    NVMDATA = INPLANE_HZ;
    res = _NVMOperation(0x4001);

    //converter.fHolder = OUTPLANE_HZ;
    //NVMDATA = converter.intHolder;
    NVMDATA = OUTPLANE_HZ;
    NVMADDR = 0x1D010004;
    res = _NVMOperation(0x4001);

    converter.fHolder = RX;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010008;
    res = _NVMOperation(0x4001);

    converter.fHolder = RY;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D01000C;
    res = _NVMOperation(0x4001);

    converter.fHolder = RZ;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010010;
    res = _NVMOperation(0x4001);

    converter.fHolder = OFFSET_XF;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010014;
    res = _NVMOperation(0x4001);

    converter.fHolder = OFFSET_XR;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010018;
    res = _NVMOperation(0x4001);

    converter.fHolder = OFFSET_YF;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D01001C;
    res = _NVMOperation(0x4001);

    converter.fHolder = OFFSET_YR;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010020;
    res = _NVMOperation(0x4001);

    converter.fHolder = OFFSET_ZF;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010024;
    res = _NVMOperation(0x4001);

    converter.fHolder = OFFSET_ZR;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010028;
    res = _NVMOperation(0x4001);

    converter.fHolder = X_OETOA;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D01002C;
    res = _NVMOperation(0x4001);

    converter.fHolder = Y_OETOA;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010030;
    res = _NVMOperation(0x4001);

    converter.fHolder = Z_OETOA;
    NVMDATA = converter.intHolder;
    NVMADDR = 0x1D010034;
    res = _NVMOperation(0x4001);
}

void flashRead(void)
{
    //INPLANE_HZ  = _INPLANE_HZ;
    //OUTPLANE_HZ = _OUTPLANE_HZ;

    INPLANE_HZ  = *(unsigned int *)(0x9D010000);
    OUTPLANE_HZ = *(unsigned int *)(0x9D010004);
    RX          = *(float *)(0x9D010008);
    RY          = *(float *)(0x9D01000C);
    RZ          = *(float *)(0x9D010010);
    OFFSET_XF   = *(float *)(0x9D010014);
    OFFSET_XR   = *(float *)(0x9D010018);
    OFFSET_YF   = *(float *)(0x9D01001C);
    OFFSET_YR   = *(float *)(0x9D010020);
    OFFSET_ZF   = *(float *)(0x9D010024);
    OFFSET_ZR   = *(float *)(0x9D010028);
    X_OETOA     = *(float *)(0x9D01002C);
    Y_OETOA     = *(float *)(0x9D010030);
    Z_OETOA     = *(float *)(0x9D010034);

    if(*(int*)(0x9D010000) == *(int*)(0x9D010038))  INPLANE_HZ = _INPLANE_HZ;
    if(*(int*)(0x9D010004) == *(int*)(0x9D010038)) OUTPLANE_HZ = _OUTPLANE_HZ;
    if(*(int*)(0x9D010008) == *(int*)(0x9D010038))          RX = _RX;
    if(*(int*)(0x9D01000C) == *(int*)(0x9D010038))          RY = _RY;
    if(*(int*)(0x9D010010) == *(int*)(0x9D010038))          RZ = _RZ;
    if(*(int*)(0x9D010014) == *(int*)(0x9D010038))   OFFSET_XF = _OFFSET_XF;
    if(*(int*)(0x9D010018) == *(int*)(0x9D010038))   OFFSET_XR = _OFFSET_XR;
    if(*(int*)(0x9D01001C) == *(int*)(0x9D010038))   OFFSET_YF = _OFFSET_YF;
    if(*(int*)(0x9D010020) == *(int*)(0x9D010038))   OFFSET_YR = _OFFSET_YR;
    if(*(int*)(0x9D010024) == *(int*)(0x9D010038))   OFFSET_ZF = _OFFSET_ZF;
    if(*(int*)(0x9D010028) == *(int*)(0x9D010038))   OFFSET_ZR = _OFFSET_ZR;
    if(*(int*)(0x9D01002C) == *(int*)(0x9D010038))     X_OETOA = _X_OETOA;
    if(*(int*)(0x9D010030) == *(int*)(0x9D010038))     Y_OETOA = _Y_OETOA;
    if(*(int*)(0x9D010034) == *(int*)(0x9D010038))     Z_OETOA = _Z_OETOA;
}

void quickPrecession(void)
{
    double trigValue = QP_pos;
    QP_doneFlag = FALSE;
    QP_past = QP_till;
    QP_till = QP_past + PR1/(((unsigned int)qp_frequency)/1000);
    if(QP_till > PR1)   QP_till -= PR1;
    if(QP_till < QP_past)   QP_clockBack = TRUE;
    else                    QP_clockBack = FALSE;
    trigValue = trigValue*2*M_PI;
    trigValue = trigValue/QP_max;
    MagX_Curr = cos(trigValue);
    MagX_Curr = MagX_Curr*QPPacket.inplane;
    MagY_Curr = sin(trigValue);
    MagY_Curr = MagY_Curr*QPPacket.inplane;

    if(QPPacket.CW_Flag)    MagY_Curr = MagY_Curr*(-1);

    if(QP_pos < QP_max)     QP_pos++;
    else                    QP_pos = 0;
}

void setDirection(int magnet, BOOL direction)
{
    switch(magnet)
    {
        case X:
            if(!X_delaying && (direction ^ X_IS_FORWARD))
            {
                OC4CONbits.ON  = 0;
                LATBbits.LATB2 = 0;
                if(direction == FORWARD)    {LATBCLR = BIT_4;   LATBSET = BIT_3;}
                if(!direction)
                {
                    LATBCLR = BIT_3;
                    LATBSET = BIT_4;
                }
                XD_past = TMR1;
                XD_till = XD_past + PR1*XOVER_TIME;
                if(XD_till >= PR1)   XD_till -= PR1;
                X_delaying = 1;
                break;
            }
            else if(X_delaying)
            {
                if(        ((XD_past > XD_till) && (TMR1 < XD_past) && (TMR1 >= XD_till))
                        || ((XD_till > XD_past) && (TMR1 > XD_till)))
                {
                    TMR2 = 0;
                    OC4CONbits.ON  = 1;
                    X_delaying = 0;
                }
            }
            break;

        case Y:
            if(!Y_delaying && ((!direction && Y_IS_FORWARD) || (direction && Y_IS_REVERSE)))
            {
                OC2CONbits.ON  = 0;
                LATAbits.LATA1 = 0;
                if(direction == FORWARD)    {LATBCLR = BIT_7;   LATASET = BIT_4;}
                if(direction == REVERSE)    {LATACLR = BIT_4;   LATBSET = BIT_7;}
                YD_past = TMR1;
                YD_till = YD_past + PR1*XOVER_TIME;
                if(YD_till >= PR1)   YD_till -= PR1;
                Y_delaying = 1;
                break;
            }
            else if (Y_delaying)
            {
                if(        ((YD_past > YD_till) && (TMR1 < YD_past) && (TMR1 >= YD_till))
                        || ((YD_till > YD_past) && (TMR1 > YD_till)))
                {
                    TMR2 = 0;
                    OC2CONbits.ON = 1;
                    Y_delaying = 0;
                }
            }
            break;
            
        case Z:
            if(!Z_delaying && ((!direction && Z_IS_FORWARD) || (direction && Z_IS_REVERSE)))
            {
                OC1CONbits.ON  = 0;
                LATAbits.LATA0 = 0;
                if(direction == FORWARD)    {LATBCLR = BIT_9;   LATBSET = BIT_8;}
                if(direction == REVERSE)    {LATBCLR = BIT_8;   LATBSET = BIT_9;}
                ZD_past = TMR1;
                ZD_till = ZD_past + PR1*XOVER_TIME;
                if(ZD_till >= PR1)   ZD_till -= PR1;
                Z_delaying = 1;
                break;
            }
            else if(Z_delaying)
            {
                if(        ((ZD_past > ZD_till) && (TMR1 < ZD_past) && (TMR1 >= ZD_till))
                        || ((ZD_till > ZD_past) && (TMR1 > ZD_till)))
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
    USBEnableEndpoint(1,USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Pipe 2: Voltage message to MagMaestro
    USBEnableEndpoint(2,USB_IN_ENABLED |USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Pipe 3: Control Byte
    USBEnableEndpoint(3,USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Pipe 4: Bulk message to MagMaestro
    USBEnableEndpoint(4,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Pipe 5: Bulk message to Lodestone
    USBEnableEndpoint(5,USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
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
void USBCBStdSetDscHandler() { }

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