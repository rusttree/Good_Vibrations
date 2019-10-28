/*
 * File:   main.c
 * Author: dan
 *
 * Created on October 16, 2019, 2:36 PM
 */

// PIC16F18324 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 32000000

unsigned char sleep = 0;
unsigned char foo = 0xFF;
unsigned char record = 0;

void __interrupt() isr(void)
{
    unsigned char tmr;
    static unsigned char override = 0;
    static unsigned short ocount = 0;

    if ( IOCAFbits.IOCAF5 )
    {
        //Interrupt-on-change on A5
        IOCAFbits.IOCAF5 = 0;
        if ( PORTAbits.RA5 )
        {
            //Rising edge
            TMR0 = 0;
            //Switch to falling edge IOC
            IOCANbits.IOCAN5 = 1;
            IOCAPbits.IOCAP5 = 0;
        }
        else if ( !PORTAbits.RA5 )
        {
            //Since Timer0 is 0.5us per tic, the value in TMR0 divided by 2 is
            //the number of microseconds of the measured pulse width.
            tmr = TMR0;

            //Switch to rising edge IOC
            IOCAPbits.IOCAP5 = 1;
            IOCANbits.IOCAN5 = 0;

            //Special case: If in ramp function, override to hold the high point for 1 full second
            //longer and then rapidly ramp down to "catch up" at the low point. The max duty
            //cycle value of tmr during the ramp appears to be 0xA9, so use that value as the 
            //indicator that we're in the ramp function.
            if ( (tmr >= 0xA8) && (tmr <= 0xAA) )
            {
                override = 1;
                ocount = 0;
            }

            if ( override )
            {
                if ( tmr < 0xA8 )
                {
                    //Override tmr value to 10000 cycles to keep vibration at ramp max for 1 second
                    tmr = 0xA9;
                    ocount++;
                    if ( ocount > 10000 )
                    {
                        override = 0;
                        ocount = 0;
                    }
                }
            }

            //Timer0 and Timer2 are setup such that tmr and PWM5DCH are 1:1.
            //So to get a lower duty cycle, just subject a constant value from tmr.
            PWM5DCH = tmr - 18;
        }
    }

    if ( PIR0bits.TMR0IF )
    {
        //If no edges detected for 128us, then go to sleep
        PIR0bits.TMR0IF = 0;
        sleep = 1;
    }
}

void Init_Ports(void)
{
    LATA = 0;
    LATC = 0;

    TRISAbits.TRISA2 = 0; //NC
    TRISAbits.TRISA4 = 0; //PWM_OUT
    TRISAbits.TRISA5 = 1; //PWM_IN

    TRISCbits.TRISC0 = 0; //NC
    TRISCbits.TRISC1 = 0; //NC
    TRISCbits.TRISC2 = 0; //NC
    TRISCbits.TRISC3 = 0; //NC
    TRISCbits.TRISC4 = 0; //NC
    TRISCbits.TRISC5 = 0; //NC

    ANSELA = 0;
    ANSELC = 0;

    RA4PPS = 0b00010; //RA4 is PWM5
}

void Init_Timer0(void)
{
    //Timer0 measures PWM input
    //1 tic = 0.5us
    T0CON0bits.T016BIT = 0; //8-bit timer
    T0CON1bits.T0CS = 0b010; //FOSC/4
    T0CON1bits.T0CKPS = 0b0010; //1:4 prescaler
    T0CON0bits.T0EN = 1;
    PIE0bits.TMR0IE = 1;
}

void Init_Timer2(void)
{
    //Timer2 controls PWM output
    //10kHz PWM
    T2CONbits.T2CKPS = 0b01; //Prescaler is 4
    PR2 = 199;

    T2CONbits.TMR2ON = 1;
}

void main(void)
{
    
    Init_Ports();
    Init_Timer0();
    Init_Timer2();
    PIE0bits.IOCIE = 1; //Enable IOC interrupt
    IOCAPbits.IOCAP5 = 1; //Enable RA5 rising edge IOC

    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;

    PWMTMRSbits.P5TSEL = 0b01; //Associate PWM5 with TMR2
    PWM5CONbits.PWM5EN = 1; //Enable PWM5
    PWM5DCH = 0b1011010;
    PWM5DCL = 0b00;
    while(1)
    {
        //Go into sleep mode if we detect the vibrator has been turned off 
        if (sleep)
        {
            sleep = 0;
            //Turn off all current consumers
            PWM5CONbits.PWM5EN = 0;
            T0CON0bits.T0EN = 0;
            T2CONbits.TMR2ON = 0;
            INTCONbits.GIE = 0; //Disable interrupts so it won't jump straight to isr on wake up
            asm("SLEEP");
            PWM5CONbits.PWM5EN = 1;
            T0CON0bits.T0EN = 1;
            T2CONbits.TMR2ON = 1;
            INTCONbits.GIE = 1; //Now let it go to the interrupt
        }
    }
}
