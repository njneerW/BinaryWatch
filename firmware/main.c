/*
 * File:   main.c
 * Author: Jordan Wills
 *
 * Created on October 9, 2014, 1:50 PM
 *
 * This program causes the PIC to act as a binary clock.  The following list
 * details the functionality of each pin in the system.  If a pin is not listed,
 * it is assumed to be unused.
 *
 * A0: Pushbutton 1.  Ultra low power wake used for deep sleep wake on btn press
 *
 * B0: Pushbutton 0.  INT0 used for deep sleep wake on button press
 * B1: Lower LED row digit 6 (minutes "32")
 * B2: Upper LED row digit 2 (hours "2")
 * B3: Upper LED row digit 3 (hours "4")
 * B4: Upper LED row digit 4 (hours "8")
 * B5: Upper LED row digit 1 (hours "1")
 * B6: Date LED / programmer clock (PGC)
 * B7: Time LED / programmer data (PGD)
 *
 * C0: RTCC crystal OSO
 * C1: RTCC crystal OSI
 * C3: Lower LED row digit 4 (minutes "8")
 * C4: Lower LED row digit 3 (minutes "4")
 * C5: Lower LED row digit 2 (minutes "2")
 * C6: Lower LED row digit 1 (minutes "1")
 * C7: Lower LED row digit 5 (minutes "16")
 *
 * The following list details the functionality of various peripherals:
 * Timer0: Debounce Pushbutton 0
 * Timer1: Debounce Pushbutton 1
 * Timer2: LED change animation
 * Timer3: During "set" mode, blinks the row currently being modified
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright © 2014 Jordan Wills.  All rights reserved.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/
#include <p18f24j11.h>

#pragma config WDTEN = OFF, OSC = INTOSC, RTCOSC = T1OSCREF, XINST = OFF

//*****************************************************************************
// Chip wide defines
//*****************************************************************************
//
// Calibration settings
//
#define CONFIG_BUILD        0
#define RTCCAL_VAL          0

//
// Clock values
//
#define INTOSC_FREQ         ((unsigned long)4000000)

//
// Timer0 and PB0
//
#define T0_PSCALE           16
#define T0_PSCALE_BITS      0x3
#define T0_uS_PER_TICK      16
// debounce = debounce time(uS) / 16
#define T0_DEBOUNCE         3125
// hold = hold time(uS) / 16
#define T0_HOLD             62500
//
// Timer1 and PB1
//
#define T1_PSCALE           1
#define T1_PSCALE_BITS      0x0
#define T1_uS_PER_TICK      30
// debounce = debounce time(uS) / 8
#define T1_DEBOUNCE         1640
// hold = hold time(uS) / 8
#define T1_HOLD             32787

//
// Pushbutton state machine defines
//
#define PB_STATE_IDLE       0
#define PB_STATE_DEBOUNCING 1
#define PB_STATE_MID_PRESS  2
#define PB_STATE_HELD       3
#define PB_STATE_RELEASED   4

//
// LED display states
//
#define DISP_STATE_TIME_S   0
#define DISP_STATE_TIME_H   1
#define DISP_STATE_DATE     2
#define DISP_STATE_SET_TIME 3
#define DISP_STATE_SET_DATE 4

//
// set time states
//
#define SET_STATE_LOWER 0
#define SET_STATE_UPPER 1

//
// LED port and pin definitions
//
#define PB0_PORT        PORTB
#define PB0_PORT_BITS   PORTBbits
#define PB0_PIN         RB0
#define PB0             PB0_PORT_BITS.PB0_PIN

#define PB1_PORT        PORTA
#define PB1_PORT_BITS   PORTAbits
#define PB1_PIN         RA0
#define PB1             PB1_PORT_BITS.PB1_PIN

#define L_ROW_PORT      PORTC
#define L_ROW_PORT_PINS PORTCbits
#define L6_PORT         PORTB
#define L6_PORT_PINS    PORTBbits
#define L_L1_PIN        RC6
#define L_L2_PIN        RC5
#define L_L3_PIN        RC4
#define L_L4_PIN        RC3
#define L_L5_PIN        RC7
#define L_L6_PIN        RB1
#define LED_L1          L_ROW_PORT_PINS.L_L1_PIN
#define LED_L2          L_ROW_PORT_PINS.L_L2_PIN
#define LED_L3          L_ROW_PORT_PINS.L_L3_PIN
#define LED_L4          L_ROW_PORT_PINS.L_L4_PIN
#define LED_L5          L_ROW_PORT_PINS.L_L5_PIN
#define LED_L6          L6_PORT_PINS.L_L6_PIN

#define U_ROW_PORT      PORTB
#define U_ROW_PORT_PINS PORTBbits
#define U_L1_PIN        RB5
#define U_L2_PIN        RB2
#define U_L3_PIN        RB3
#define U_L4_PIN        RB4
#define LED_U1          U_ROW_PORT_PINS.U_L1_PIN
#define LED_U2          U_ROW_PORT_PINS.U_L1_PIN
#define LED_U3          U_ROW_PORT_PINS.U_L1_PIN
#define LED_U4          U_ROW_PORT_PINS.U_L1_PIN

#define DT_PORT         PORTB
#define DT_PORT_PINS    PORTBbits
#define DT_TIME_PIN     RB7
#define DT_DATE_PIN     RB6
#define LED_DATE        DT_PORT_PINS.DT_DATE_PIN
#define LED_TIME        DT_PORT_PINS.DT_TIME_PIN

//
// Type for mapping out bits of dleep sleep persistent memory byte0
//
typedef union t_DSGPR0map
{
    struct
    {
        //
        // No matter how much I use a PIC, I will never be comfortable with
        // using bitfields
        //
        unsigned char PB0State : 2;
        unsigned char PB1State : 2;
    };
    unsigned char ucRaw;
} DSGPR0map;

//
// Type for mapping out bits of dleep sleep persistent memory byte1
//
typedef union t_DSGPR1map
{
    struct
    {
        unsigned char dispState : 3;
        unsigned char setState : 1;
        unsigned char blinkState : 1;
        unsigned char blackout : 1;
    };
    unsigned char ucRaw;
} DSGPR1map;

//
// Macros for deep sleep fields... deprecated by bit fields
//
#define PB0_DSREG           DSGPR0
#define PB0_STATE_MASK      0x03
#define PB0_STATE_SHIFT     0

#define PB1_DSREG           DSGPR0
#define PB1_STATE_MASK      0x0C
#define PB1_STATE_SHIFT     2

#define DISP_STATE_DSREG    DSGPR1
#define DISP_STATE_MASK     0x07
#define DISP_STATE_SHIFT    0

#define DISP_BLINK_DSREG    DSGPR1
#define DISP_BLINK_MASK     0x08
#define DISP_BLINK_SHIFT    3

//*****************************************************************************
// Global variables
//*****************************************************************************
unsigned char g_ucPB0State = PB_STATE_IDLE;
unsigned char g_ucPB1State = PB_STATE_IDLE;
volatile unsigned char g_ucNapTime = 0;
DSGPR0map g_sDSGPR0;
DSGPR1map g_sDSGPR1;

//*****************************************************************************
// Function prototypes
//*****************************************************************************
void PressPB0(void);
void HoldPB0(void);
void PressPB1(void);
void HoldPB1(void);
int displayTime(void);

//*****************************************************************************
//  CODE CODE CODE CODE CODE CODE CODE CODE CODE CODE CODE CODE CODE CODE CODE
//*****************************************************************************

//*****************************************************************************
//
// These should never be entered.  If we get here, something has gone awry.  In
// release mode, do nothing.  In dev mode, turn on & hold the date and time
// LEDs to signify that bad has occurred.
//
//*****************************************************************************
//#pragma interrupt HiISR
void HiISR(void)
{
    LED_DATE = 1;
    LED_TIME = 1;
    //while(1);
}

//#pragma interruptlow LoISR
void LoISR(void)
{
    LED_DATE = 1;
    LED_TIME = 1;
    //while(1);
}

//*****************************************************************************
//
// Configure the GPIOs.  Note that this gets called on both cold boot and deep
// sleep wake, so don't set anything here that you don't want done on a DS wake
// (like default LED values)
//
//*****************************************************************************
void configGPIOs(void)
{
    //
    // Set all GPIOs as digital, not analog
    //
    ANCON0 = 0xFF;
    ANCON1 = 0x1F;

    //
    // Set PA0 to input, PA1..7 as output
    //
    TRISA = 1;

    //
    // Set PB0 to input, PB1..7 as output
    //
    TRISB = 1;

    //
    // Set port b to enable weak pull-up
    //
    INTCON2bits.NOT_RBPU = 1;

    //
    // Set all PC to output
    //
    TRISC = 0;

    //
    // Set unused ports to drive low, per low power recommendations in datasheet
    //
    PORTCbits.RC2 = 0;
    PORTAbits.RA1 = 0;
    PORTAbits.RA2 = 0;
    PORTAbits.RA3 = 0;
    PORTAbits.RA5 = 0;

}

//*****************************************************************************
//
// Configure the Real time clock/calendar peripheral.
//
//*****************************************************************************
void configRTCC(void)
{
    //
    // unlock writes to the RTCC
    //
    EECON2 = 0x55;
    EECON2 = 0xAA;
    RTCCFGbits.RTCWREN = 1;

    //
    // enable the RTCC, set pointers to read seconds and minutes
    //
    RTCCFGbits.RTCEN = 1;
    RTCCFGbits.RTCPTR0 = 0;
    RTCCFGbits.RTCPTR1 = 0;

    //
    // enable the RTCC alarm, chime @ 1 Hz
    //
    ALRMCFGbits.CHIME = 1;
    ALRMCFGbits.AMASK0 = 1;
    ALRMCFGbits.AMASK1 = 0;
    ALRMCFGbits.AMASK2 = 0;
    ALRMCFGbits.AMASK3 = 0;
    ALRMCFGbits.ALRMEN = 1;
}

//*****************************************************************************
//
// Configure the deep sleep module.  We want to be able to wake from the RTCC,
// INT0, and the ULPWU interrupt.  Also, make sure deep sleep really means deep
// sleep, not low power sleep.
//
//*****************************************************************************
void configDS(void)
{
    WDTCONbits.REGSLP = 1;
    OSCCONbits.IDLEN = 0;
    DSCONHbits.RTCWDIS = 0;
    DSCONHbits.DSULPEN = 0;
    DSCONLbits.ULPWDIS = 0;
}

//*****************************************************************************
//
// Configure Timer3.  Source from internal Oscillator, maximum prescaler, enable
// 16 bit mode.
//
//*****************************************************************************
void configTimer3(void)
{
    //
    // set source to intOSC/4, 1 MHz
    //
    T3CONbits.TMR3CS = 0;

    //
    // Set prescaler to 1:8
    //
    T3CONbits.T3CKPS = 0x3;

    //
    // Enable 16 bit operations
    //
    T3CONbits.RD16 = 1;

    //
    // zero out timer
    //
    TMR3H = 0;
    TMR3L = 0;

    //
    // Turn on timer
    //
    //T3CONbits.TMR3ON = 1;

    return;
}

//*****************************************************************************
//
// Configure Timer1.  Source from T1OSC (external 32.768 crystal), no prescaler,
// 8 bit width, no counting/gating
//
//*****************************************************************************
void configTimer1(void)
{
    //
    // set source to T1OSC (T1CKI pin)
    //
    T1CONbits.TMR1CS = 2;
    
    //
    // set the prescaler to 0, giving us a max timeout of 2 s
    //
    // set prescaler to 1:1, giving us a range of:
    //   (1/32.768KHz) * prescaler = 30.5 us per tick
    //   1 = 30.5 uS, 0xFFFF = 2 s
    //
    T1CONbits.T1CKPS = 0x0;
    
    //
    // Enable T1 external oscillator
    //
    T1CONbits.T1OSCEN = 1;

    //
    // Set timer to 8 bit width
    //
    T1CONbits.RD16 = 1;

    //
    // Make sure T1 gate is diabled
    //
    T1GCONbits.TMR1GE = 0;

    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 0;
    T1CONbits.TMR1ON = 0;
}

//*****************************************************************************
//
// Configure Timer2.  Source from internal Oscillator, 1:16 prescaler
//
//*****************************************************************************
void configTimer2(void)
{
    //
    // Set prescaler to 1:16
    //
    T2CONbits.T2CKPS = 0x2;

    //
    // zero out timer
    //
    TMR2 = 0;
}

//*****************************************************************************
//
// Configure Timer0.  Source from internal Oscillator, 1:16 prescaler, enable
// 8 bit mode.
//
//*****************************************************************************
void configTimer0(void)
{
    //
    // set source to intOSC
    //
    T0CONbits.T0CS = 0;
    T0CONbits.T08BIT = 0;

    //
    // set prescaler to 1:16, giving us a range of:
    //   (1/4MHz * 4) * prescaler = 16 us per tick
    //   1 = 16 uS, 0xFFFF = 1.05 s
    //
    T0CONbits.T0PS = 0x03;
    T0CONbits.PSA = 0;

    //
    // Always write TMR0H before writing TMR0L
    //
    TMR0H = 0;
    TMR0L = 0;

    //
    // enable timer interrupt
    //
    INTCONbits.TMR0IE = 0;
    INTCONbits.TMR0IF = 0;
}

//*****************************************************************************
//
// This function will cause the system to go into deep sleep mode.  If all goes
// well, this function never returns.  On wake, the system will start executing
// at main.
//
//*****************************************************************************
void enterDS(void)
{
    DSCONHbits.DSULPEN = 1;
    DSGPR0 = g_sDSGPR0.ucRaw;
    DSGPR1 = g_sDSGPR1.ucRaw;
    DSCONHbits.DSEN = 1;
    Sleep();
}

//*****************************************************************************
//
// This function will update the state of pushbutton 0.  If it detects a press
// or hold of the pushbutton, it will call the corresponding function.  On
// return, it will signify whether it is safe for the state machine to go to
// sleep (and thus disable and reset the timer being used).  This function uses
// the Timer0 peripheral to gauge debounce and hold times.
//
// pfnPress: a pointer to the function called when the button is pressed
// pfnHold: a pointer to the function called when the button is held
//
// return: 0 if the it is safe to sleep system, 1 if it needs to stay awake.
//
//*****************************************************************************
int debouncePB0(void (*pfnPress)(void), void (*pfnHold)(void))
{
    unsigned int uiTime;

    if((g_ucPB0State == PB_STATE_IDLE) && !PB0)
    {
        g_ucPB0State = PB_STATE_DEBOUNCING;
        TMR0H = 0;
        TMR0L = 0;
        T0CONbits.TMR0ON = 1;
        return 1;
    }

    if(PB0)
    {
        //
        // If the button isn't pressed and we're in idle state, go back to bed
        //
        if(g_ucPB0State == PB_STATE_IDLE)
        {
            return 0;
        }
        //
        // If the button is not pressed and we're in debounce state, the button
        // went from press to not press during the bounce window.  Act like
        // nothing happened, go back to bed.
        //
        if(g_ucPB0State == PB_STATE_DEBOUNCING)
        {
            g_ucPB0State = PB_STATE_IDLE;
            T0CONbits.TMR0ON = 0;
            return 0;
        }
        //
        // If the button is not pressed and we're past the bounce window but
        // not past the hold window, we have a normal pb press.  Take action.
        //
        else if(g_ucPB0State == PB_STATE_MID_PRESS)
        {
            pfnPress();
            T0CONbits.TMR0ON = 0;
            g_ucPB0State = PB_STATE_IDLE;
            return 0;
        }
        //
        // If the button is not pressed and was previously held, we should kick
        // off the debounce timer for release
        //
        else if(g_ucPB0State == PB_STATE_HELD)
        {
            TMR0H = 0;
            TMR0L = 0;
            T0CONbits.TMR0ON = 1;

            g_ucPB0State = PB_STATE_RELEASED;

            return 1;
        }
        //
        // If the button has been released from a hold, we wait until the timer
        // is past its debounce value, then are safe to go to sleep
        //
        else if(g_ucPB0State == PB_STATE_RELEASED)
        {
            uiTime = TMR0L;
            uiTime = uiTime | ((unsigned int)TMR0H << 8);

            //
            // Make sure that the button is debounced on release before saying
            // we're good to go to sleep
            //
            if(uiTime > T0_DEBOUNCE)
            {
                T0CONbits.TMR0ON = 0;
                g_ucPB0State = PB_STATE_IDLE;
                return 0;
            }
            return 1;
        }
    }
    else
    {
        uiTime = TMR0L;
        uiTime = uiTime | ((unsigned int)TMR0H << 8);

        //
        // If the button is pressed and we're past the hold window, evaluate
        // the current state.
        //
        if(uiTime > T0_HOLD)
        {
            //
            // If the current state has not already transitioned to held, take
            // the hold action and update it.  If the state already is held,
            // there's nothing to do until the user releases the button.
            //
            if(g_ucPB0State != PB_STATE_HELD)
            {
                g_ucPB0State = PB_STATE_HELD;
                T0CONbits.TMR0ON = 0;
                pfnHold();
                return 1;
            }
            return 1;
        }
        //
        // If the button is pressed and we're past the bounce window, mark the
        // state as mid-press.  We'll evaluate if it's a press or a hold later.
        //
        else if(uiTime > T0_DEBOUNCE)
        {
            g_ucPB0State = PB_STATE_MID_PRESS;
            return 1;
        }
        return 1;
    }
}

//*****************************************************************************
//
// This function will update the state of pushbutton 1.  If it detects a press
// or hold of the pushbutton, it will call the corresponding function.  On
// return, it will signify whether it is safe for the state machine to go to
// sleep (and thus disable and reset the timer being used).  This function uses
// the Timer1 peripheral to gauge debounce and hold times.
//
// pfnPress: a pointer to the function called when the button is pressed
// pfnHold: a pointer to the function called when the button is held
//
// return: 0 if the it is safe to sleep system, 1 if it needs to stay awake.
//
//*****************************************************************************
int debouncePB1(void (*pfnPress)(void), void (*pfnHold)(void))
{
    static unsigned long ulTime = 0;

    if((g_ucPB1State == PB_STATE_IDLE) && !PB1)
    {
        g_ucPB1State = PB_STATE_DEBOUNCING;
        TMR1H = 0;
        TMR1L = 0;
        PIR1bits.TMR1IF = 0; //delme
        T1CONbits.TMR1ON = 1;
        return 1;
    }

    if(PB1)
    {
        if(g_ucPB1State == PB_STATE_IDLE)
        {
            return 0;
        }
        //
        // If the button is not pressed and we're in debounce state, the button
        // went from press to not press during the bounce window.  Act like
        // nothing happened, go back to bed.
        //
        if(g_ucPB1State == PB_STATE_DEBOUNCING)
        {
            g_ucPB1State = PB_STATE_IDLE;
            PIR1bits.TMR1IF = 0; //delme
            T1CONbits.TMR1ON = 0;
            return 0;
        }
        //
        // If the button is not pressed and we're past the bounce window but
        // not past the hold window, we have a normal pb press.  Take action.
        //
        else if(g_ucPB1State == PB_STATE_MID_PRESS)
        {
            pfnPress();
            PIR1bits.TMR1IF = 0; //delme
            T1CONbits.TMR1ON = 0;
            g_ucPB1State = PB_STATE_IDLE;
            return 0;
        }
        //
        // If the button is not pressed and we're past the hold window, we have
        // a release after a hold.  Action should have already occured, go back
        // to sleep.
        //
        else if(g_ucPB1State == PB_STATE_HELD)
        {
            g_ucPB1State = PB_STATE_IDLE;
            return 1;
        }
    }
    else
    {
        //
        // if we overflow the counter (which we will on a hold), add 0x10000 to
        // the time, then and off the bottom 16 bits.  
        //
        //if(PIR1bits.TMR1IF)
        //{
        //    ulTime = 0x10000;
        //}
        //else
        //{
        //    ulTime = 0;
        //}
        //ulTime += TMR1L;
        ulTime = TMR1L;
        ulTime |= ((unsigned int)TMR1H << 8);
        
        //
        // If the button is pressed and we're past the hold window, evaluate
        // the current state.
        //
        //if(uiTime > T0_HOLD)
        if(ulTime > T1_HOLD)
        {
            //
            // If the current state has not already transitioned to held, take
            // the hold action and update it.  If the state already is held,
            // there's nothing to do until the user releases the button.
            //
            if(g_ucPB1State != PB_STATE_HELD)
            {
                pfnHold();
                g_ucPB1State = PB_STATE_HELD;
                T1CONbits.TMR1ON = 0;
            }
            return 1;
        }
        //
        // If the button is pressed and we're past the bounce window, mark the
        // state as mid-press.  We'll evaluate if it's a press or a hold later.
        //
        else if(ulTime > T1_DEBOUNCE)
        {
            g_ucPB1State = PB_STATE_MID_PRESS;
            return 1;
        }
        return 1;
    }
}

//*****************************************************************************
//
// This function is called whenever PB0 is pressed. If the watch is in display
// mode, a PB0 press will cycle to the next display mode (seconds, minutes & 
// hours, or days & months).  If the watch is in set mode, a PB0 press will
// increment the field that is currently selected, rolling over when necessary.
//
// return: none
//
//*****************************************************************************
void PressPB0(void)
{
    unsigned char ucMinute;
    unsigned char ucHour;
    unsigned char ucMonth;
    unsigned char ucDay;
    unsigned char ucValH;

    if(g_sDSGPR1.blackout)
    {
        g_sDSGPR1.blackout = 0;
        return;
    }

    g_ucNapTime = 0;
    //
    // If we're in display mode, PB0 will toggle between showing date or time.
    //
    if( g_sDSGPR1.dispState == DISP_STATE_TIME_S)
    {
        g_sDSGPR1.dispState = DISP_STATE_TIME_H;
    }
    else if( g_sDSGPR1.dispState == DISP_STATE_TIME_H)
    {
        g_sDSGPR1.dispState = DISP_STATE_DATE;
    }
    else if(g_sDSGPR1.dispState == DISP_STATE_DATE)
    {
        g_sDSGPR1.dispState = DISP_STATE_TIME_S;
    }
    else if(g_sDSGPR1.dispState == DISP_STATE_SET_TIME)
    {
        //
        // When setting the time, lower is minutes, upper is hours
        //
        if(g_sDSGPR1.setState == SET_STATE_LOWER)
        {
            RTCCFGbits.RTCPTR1 = 0;
            RTCCFGbits.RTCPTR0 = 0;
            ucMinute = (RTCVALH & 0xF) + (10*(RTCVALH >> 4));
            ucMinute++;

            //
            // Minutes are easy to increment.  Go up until you reach 60, then
            // back to 0.
            //
            if(ucMinute >= 60)
            {
                ucMinute = 0;
            }

            //
            // Write the value back to RTCC peripheral in BCD form
            //
            RTCVALH = (ucMinute % 10) | ((ucMinute/10) << 4);
        }
        else
        {
            RTCCFGbits.RTCPTR0 = 1;
            RTCCFGbits.RTCPTR1 = 0;
            ucHour = (RTCVALL & 0xF) + (10*(RTCVALL >> 4));
            ucHour++;

            //
            // Hours are pretty easy to increment as well.  Go up until you
            // hit 24, then back to 0.
            //
            if(ucHour >= 24)
            {
                ucHour = 0;
            }
            
            //
            // Write the value back to RTCC peripheral in BCD form
            //
            RTCVALL = (ucHour % 10) | ((ucHour/10) << 4);
        }
    }
    else if(g_sDSGPR1.dispState == DISP_STATE_SET_DATE)
    {
        //
        // When setting the date, lower is days, upper is months
        //
        if(g_sDSGPR1.setState == SET_STATE_LOWER)
        {
            RTCCFG &= ~0x03;
            RTCCFG |= 0x2;
            ucDay = (RTCVALL & 0xF) + (10*(RTCVALL >> 4));
            ucValH = RTCVALH;
            ucMonth = (ucValH & 0xF) + (10*(ucValH >> 4));
            ucDay++;

            //
            // firguring out how to roll over months is irritating.  Stupid
            // Roman calendar.
            //
            if(ucMonth == 2)
            {
                if(ucDay >= 29)
                {
                    ucDay = 1;
                }
            }
            //
            // Thirty days have September,
            // April, June, and November.
            // All the rest have 31,
            // Except for February, because he's a jerk.
            //
            else if((ucMonth == 1) || (ucMonth == 3) || (ucMonth == 5) ||
                    (ucMonth == 7) || (ucMonth == 8) || (ucMonth == 10) ||
                    (ucMonth == 12))
            {
                if(ucDay >= 32)
                {
                    ucDay = 1;
                }
            }
            else
            {
                if(ucDay >= 31)
                {
                    ucDay = 1;
                }
            }

            //
            // Write the day back to the RTCC in BCD form
            //
            RTCCFG &= ~0x03;
            RTCCFG |= 0x2;
            RTCVALL = (ucDay % 10) | ((ucDay/10) << 4);
        }
        else
        {
            RTCCFG &= ~0x03;
            RTCCFG |= 0x2;
            ucValH = RTCVALH;
            ucMonth = (ucValH & 0xF) + (10*(ucValH >> 4));
            ucMonth++;

            //
            // Month is easy to figure out... just be careful about 1..12 as
            // as opposed to 0..11.
            //
            if(ucMonth >= 13)
            {
                ucMonth = 1;
            }

            //
            // Write the month back to the RTCC in BCD form
            //
            RTCCFG &= ~0x03;
            RTCCFG |= 0x2;
            RTCVALH = (ucMonth % 10) | ((ucMonth/10) << 4);
        }
    }
}

//*****************************************************************************
//
// This function is called whenever PB0 is held. Regardless of watch state,
// holding PB0 will cause the watch to go into deep sleep with all LEDs turned
// off, waking up when another button is pressed.
//
// return: none
//
//*****************************************************************************
void HoldPB0(void)
{
    //
    // If the button is held while in anything other than a "display" mode,
    // ignore the press
    //
    if((g_sDSGPR1.dispState == DISP_STATE_TIME_S) ||
       (g_sDSGPR1.dispState == DISP_STATE_TIME_H) ||
       (g_sDSGPR1.dispState == DISP_STATE_DATE))
    {
        g_ucNapTime = 1;
    }
}

//*****************************************************************************
//
// This function is called whenever PB1 is held. PB1 is used to enter "set"
// mode, or move to setting the next value if the watch is already in a set
// mode.
//
// return: none
//
//*****************************************************************************
void HoldPB1(void)
{
    if(g_sDSGPR1.blackout)
    {
        g_sDSGPR1.blackout = 0;
        return;
    }

    //
    // If we're in any display mode, PB1 press will enter set mode.  We always
    // start by setting time until PB1 is pressed again, then set date until
    // PB1 press, then we go back to display time mode.
    //
    if( (g_sDSGPR1.dispState == DISP_STATE_TIME_S) ||
        (g_sDSGPR1.dispState == DISP_STATE_TIME_H) ||
        (g_sDSGPR1.dispState == DISP_STATE_DATE))
    {
        //
        // unlock writes to the RTCC, then disable clock while we're modifying
        // values
        //
        EECON2 = 0x55;
        EECON2 = 0xAA;
        RTCCFGbits.RTCWREN = 1;
        RTCCFGbits.RTCEN = 0;

        TMR3H = 0;
        TMR3L = 0;
        T3CONbits.TMR3ON = 1;

        g_sDSGPR1.dispState = DISP_STATE_SET_TIME;
        LED_DATE = 0;
        LED_TIME = 1;
    }

    return;
}

//*****************************************************************************
//
// This function is called whenever PB1 is pressed. Pressing PB1 will toggle
// whether or not display updates blink in or shift in (default).  I like the
// shiftin, but some find it annoying.
//
// return: none
//
//*****************************************************************************
void PressPB1(void)
{
    //
    // If we're in the middle of setting the time, press pb1 to move to the next
    // set field.  If we're at the last set field, set_date upper, then a press
    // should bring us out of set mode and into display hour.
    //
    if(g_sDSGPR1.dispState == DISP_STATE_SET_TIME)
    {
        if( g_sDSGPR1.setState == SET_STATE_LOWER )
        {
            g_sDSGPR1.setState = SET_STATE_UPPER;
        }
        else
        {
            g_sDSGPR1.setState = SET_STATE_LOWER;

            TMR3H = 0;
            TMR3L = 0;
            T3CONbits.TMR3ON = 1;

            g_sDSGPR1.dispState = DISP_STATE_SET_DATE;
            LED_DATE = 1;
            LED_TIME = 0;
        }
    }
    else if(g_sDSGPR1.dispState == DISP_STATE_SET_DATE)
    {
        if( g_sDSGPR1.setState == SET_STATE_LOWER )
        {
            g_sDSGPR1.setState = SET_STATE_UPPER;
        }
        else
        {
            g_sDSGPR1.setState = SET_STATE_LOWER;
            //
            // unlock writes to the RTCC, then start the clock again
            //
            EECON2 = 0x55;
            EECON2 = 0xAA;
            RTCCFGbits.RTCWREN = 1;
            RTCCFGbits.RTCEN = 1;
            TMR3H = 0;
            TMR3L = 0;
            T3CONbits.TMR3ON = 0;
            g_sDSGPR1.dispState = DISP_STATE_TIME_H;
        }
    }
    //
    // If we're not in a set mode, then pressing pb1 will just en/disable the
    // blinky
    //
    else
    {
        g_sDSGPR1.blinkState = !g_sDSGPR1.blinkState;
    }
    
    return;
}

//*****************************************************************************
//
// This function will return what bits are being displayed by the lower six
// LEDs.
//
// return: The binary value being displayed by the LEDs.  Bit 0 corresponds to
//         the rightmost LED, bit 5 corresponds to the leftmost LED.
//
//*****************************************************************************
unsigned char readLower(void)
{
    unsigned char retVal;

    retVal = LED_L1;
    retVal |= (LED_L2 << 1);
    retVal |= (LED_L3 << 2);
    retVal |= (LED_L4 << 3);
    retVal |= (LED_L5 << 4);
    retVal |= (LED_L6 << 5);

    return retVal;
}

//*****************************************************************************
//
// This function will return what bits are being displayed by the upper four
// LEDs.
//
// return: The binary value being displayed by the LEDs.  Bit 0 corresponds to
//         the rightmost LED, bit 3 corresponds to the leftmost LED.
//
//*****************************************************************************
unsigned char readUpper(void)
{
    unsigned char retVal;

    retVal = LED_U1;
    retVal |= (LED_U2 << 1);
    retVal |= (LED_U3 << 2);
    retVal |= (LED_U4 << 3);

    return retVal;
}
//*****************************************************************************
//
// This function will update the LED rows to display the binary representation
// of the input values.  Mappings for which pin corresponds to which LED can
// be found in the comment header at the top of this file.
//
// ucUpperRow: The value to display in the upper row of LEDs, 4 bits max
// ucLowerRow: The value to display in the ower row of LEDs, 6 bits max
//
// return: none
//
//*****************************************************************************
void updateRows(unsigned char ucUpperRow, unsigned char ucLowerRow)
{
    //TODO: Update these to use the pin and port macros we created for all of
    //      these.
    //DONE: JTW: replaced PORTxbits.Rxn references with macro'd LED_<row><num>
    //           references.

    //
    // Turn the lower row LEDs (minutes, days, or seconds) on or off
    //
    if(ucLowerRow & 0x01)
    {
        LED_L1 = 1;
    }
    else
    {
        LED_L1 = 0;
    }
    if(ucLowerRow & 0x02)
    {
        LED_L2 = 1;
    }
    else
    {
        LED_L2 = 0;
    }
    if(ucLowerRow & 0x04)
    {
        LED_L3 = 1;
    }
    else
    {
        LED_L3 = 0;
    }
    if(ucLowerRow & 0x08)
    {
        LED_L4 = 1;
    }
    else
    {
        LED_L4 = 0;
    }
    if(ucLowerRow & 0x10)
    {
        LED_L5 = 1;
    }
    else
    {
        LED_L5 = 0;
    }
    if(ucLowerRow & 0x20)
    {
        LED_L6 = 1;
    }
    else
    {
        LED_L6 = 0;
    }

    //
    // Turn the upper row LEDs (hour or month) on or off
    //
    if(ucUpperRow & 0x01)
    {
        LED_U1 = 1;
    }
    else
    {
        LED_U1 = 0;
    }
    if(ucUpperRow & 0x02)
    {
        LED_U2 = 1;
    }
    else
    {
        LED_U2 = 0;
    }
    if(ucUpperRow & 0x04)
    {
        LED_U3 = 1;
    }
    else
    {
        LED_U3 = 0;
    }
    if(ucUpperRow & 0x08)
    {
        LED_U4 = 1;
    }
    else
    {
        LED_U4 = 0;
    }
}

//*****************************************************************************
//
// This function is used to update the time display.  It will determine what
// display or set state the system is currently in and turn the appropriate
// LEDs on or off.
//
// return: none
//
//*****************************************************************************
int displayTime(void)
{
    unsigned char ucLowerRow;
    unsigned char ucUpperRow;
    unsigned char ucValH;
    unsigned char ucSecond;
    static unsigned char ucToggle = 0;
    unsigned char ucLastUpper = 0xff;
    unsigned char ucLastLower = 0xff;
    static unsigned int uiLoopCount = 0;
    static unsigned char ucShiftAmount = 5;
    static unsigned char ucShiftUpper = 3;
    unsigned int uiStrobeTime;
    int retVal = 0;

    RTCCFGbits.RTCPTR1 = 0;
    RTCCFGbits.RTCPTR0 = 0;
    ucSecond = (RTCVALL & 0xF) + (10*(RTCVALL >> 4));

    uiStrobeTime = TMR3L;
    uiStrobeTime = uiStrobeTime | ((unsigned int)TMR3H << 8);

    //
    // When we're setting the date or time, toggle the LEDs of the field that
    // being changed at 2 Hz
    //
    if(uiStrobeTime > 0xA000)
    {
        TMR3H = 0;
        TMR3L = 0;
        if(ucToggle)
        {
            ucToggle = 0;
        }
        else
        {
            ucToggle = 1;
        }
    }

    //
    // Determine which (if any) of the D/T LEDs should be illuminated:
    // if we're setting the date/time, activate that LED constantly.  If we're
    // displaying the time/date, turn on the appropriate LED on each odd second,
    // giving us a nice, aesthetic one second pulse.  If we're displaying
    // seconds, D and T LEDs should be off.
    //
    // TODO: replace these with port and pin macros
    // DONE: JTW: replaced PORTxbits.Rxn references with macro'd LED_<row><num>
    //           references.
    if(g_sDSGPR1.dispState == DISP_STATE_SET_TIME)
    {
        LED_TIME = 1;
        LED_DATE = 0;
    }
    else if(g_sDSGPR1.dispState == DISP_STATE_SET_DATE)
    {
        LED_DATE = 1;
        LED_TIME = 0;
    }
    else if(g_sDSGPR1.dispState == DISP_STATE_TIME_H)
    {
        if(ucSecond & 0x1)
        {
            LED_TIME = 1;
            LED_DATE = 0;
        }
        else
        {
            LED_TIME = 0;
            LED_DATE = 0;
        }
    }
    else if(g_sDSGPR1.dispState == DISP_STATE_DATE)
    {
        if(ucSecond & 0x1)
        {
            LED_TIME = 0;
            LED_DATE = 1;
        }
        else
        {
            LED_TIME = 0;
            LED_DATE = 0;
        }
    }
    else
    {
        LED_TIME = 0;
        LED_DATE = 0;
    }

    //
    // Determine which LEDs should be illuminated on the upper and lower rows
    //
    if((g_sDSGPR1.dispState == DISP_STATE_TIME_H) ||
       (g_sDSGPR1.dispState == DISP_STATE_SET_TIME))
    {
        RTCCFGbits.RTCPTR0 = 1;
        RTCCFGbits.RTCPTR1 = 0;
        ucUpperRow = (RTCVALL & 0xF) + (10*(RTCVALL >> 4));
        
        //
        // If we're setting the time, we need to know if we're setting AM or PM
        // in order to know when to update the day on the calendar side.
        // Overload the date LED to give us an AM/PM toggle when setting.
        //
        if(g_sDSGPR1.dispState == DISP_STATE_SET_TIME)
        {
            if(ucUpperRow >= 12)
            {
                LED_DATE = 1;
            }
        }

        //
        // If we're setting the upper row, blink it in and out so the user knows
        // what they're updating
        //
        if((g_sDSGPR1.dispState == DISP_STATE_SET_TIME) &&
           (g_sDSGPR1.setState == SET_STATE_UPPER) && ucToggle)
        {
            ucUpperRow = 0;
        }
        else
        {
            //
            // in TIME_H, we display hours on top, minutes on bottom
            // Hours are 0-23 in RTCC, and we want to display them 1-12 on the
            // clock.  Some simple math required.
            //
            if (ucUpperRow == 0)
            {
                //
                // 0 means midnight, which I call 12 AM.
                //
                ucUpperRow = 12;
            }

            //
            // 24 hour format.  Display 1-12
            //
            if(ucUpperRow > 12)
            {
                ucUpperRow -= 12;
            }
        }

        //
        // If we're setting the lower row, blink it in and out so the user knows
        // what they're updating
        //
        if((g_sDSGPR1.dispState == DISP_STATE_SET_TIME) &&
           (g_sDSGPR1.setState == SET_STATE_LOWER) && ucToggle)
        {
            ucLowerRow = 0;
        }
        else
        {
            //
            // Minutes are 0-59 in RTCC, and we want to display them 0-59 on the
            // clock.  Only math needed is BCD->decimal conversion
            //
            RTCCFGbits.RTCPTR0 = 0;
            RTCCFGbits.RTCPTR1 = 0;
            ucValH = RTCVALH;
            ucLowerRow = (ucValH & 0xF) + (10*(ucValH >> 4));
        }
        
    }
    else if(g_sDSGPR1.dispState == DISP_STATE_TIME_S)
    {
        //
        // in TIME_S, we display nothing on top, seconds on bottom
        //
        RTCCFGbits.RTCPTR0 = 0;
        RTCCFGbits.RTCPTR1 = 0;
        ucUpperRow = 0;
        ucLowerRow = (RTCVALL & 0xF) + (10*(RTCVALL >> 4));
    }
    else
    {
        //
        // in _DATE modes, upper is month, lower is day.
        //

        //
        // If we're setting the lower row, blink it in and out so the user knows
        // what they're updating
        //
        if((g_sDSGPR1.dispState == DISP_STATE_SET_DATE) &&
           (g_sDSGPR1.setState == SET_STATE_LOWER) && ucToggle)
        {
            ucLowerRow = 0;
        }
        else
        {
            //
            // Days are 1-31 in RTCC, and we want to display them 1-31 on the
            // clock.  Only math needed is BCD->decimal conversion
            //
            RTCCFG &= ~0x03;
            RTCCFG |= 0x2;
            ucLowerRow = (RTCVALL & 0xF) + (10*(RTCVALL >> 4));
        }

        //
        // If we're setting the upper row, blink it in and out so the user knows
        // what they're updating
        //
        if((g_sDSGPR1.dispState == DISP_STATE_SET_DATE) &&
           (g_sDSGPR1.setState == SET_STATE_UPPER) && ucToggle)
        {
            ucUpperRow = 0;
        }
        else
        {
            //
            // Months are 1-12 in RTCC, and we want to display them 1-12 on the
            // clock.  Only math needed is BCD->decimal conversion
            //
            RTCCFG &= ~0x03;
            RTCCFG |= 0x2;
            ucValH = RTCVALH;
            ucUpperRow = (ucValH & 0xF) + (10*(ucValH >> 4));
        }
    }

    //
    // If we're displaying the date or time, shift in the LED values, because
    // that lookss pretty cool.  NEVER DO THIS DURING NAP TIME!!!
    //
    if(!g_ucNapTime && g_sDSGPR1.blinkState &&
       ((g_sDSGPR1.dispState == DISP_STATE_TIME_H) ||
       (g_sDSGPR1.dispState == DISP_STATE_DATE)))
    {
        //
        // Compare the value currently being displayed to the value we want to
        // display, which tells us if we need to shift in a new value
        //
        ucLastLower = readLower();
        ucLastUpper = readUpper();
        if((ucLowerRow != ucLastLower) ||
           (ucUpperRow != ucLastUpper))
        {
            //
            // Turn on the timer.  The timer rolls over if it gets above 0xFF,
            // so we can't get too close to that ceiling
            //
            T2CONbits.TMR2ON = 1;
            if(TMR2 >= 0xC0)
            {
                uiLoopCount++;
                TMR2 = 0;
            }
            //
            // It'd be great if we could use a bigger pre-scaler or a 16 bit
            // value for this, but... work with what you got, which is an 8 bit
            // timer.  Count how often it rolls over.  Change this number to
            // make the shift faster or slower.
            //
            if(uiLoopCount > 30)
            {
                //
                // Make sure we don't decrement past zero.  Really only needed
                // for shiftUpper, as it decrements 3 times, but lower
                // decrements 5 times.
                //
                if(ucShiftAmount)
                {
                    ucShiftAmount--;
                }
                if(ucShiftUpper)
                {
                    ucShiftUpper--;
                }
                uiLoopCount = 0;
            }

            //
            // Sanity check our shift values
            //
            if((ucShiftAmount > 5) || (ucShiftUpper > 3))
            {
                ucShiftAmount = 5;
                ucShiftUpper = 3;
            }
            
            //
            // If we're done decrementing everything, the shift is complete,
            // so no more need to shift all that stuff.
            //
            if((ucShiftAmount == 0) && (ucShiftUpper == 0))
            {
                ucLastLower = ucLowerRow;
                ucLastUpper = ucUpperRow;
                T2CONbits.TMR2ON = 0;
                TMR2 = 0;
                ucShiftAmount = 5;
                ucShiftUpper = 3;
            }
            
            //
            // If we're still shifting, make sure to return 1 so that we don't
            // go into deep sleep and lose all of our static variables
            //
            else
            {
                ucLowerRow = ucLowerRow << ucShiftAmount;
                ucUpperRow = ucUpperRow << ucShiftUpper;
                retVal = 1;
            }
        }
    }

    //
    // Send our calculated upper and lower LED values to the display
    //
    updateRows(ucUpperRow, ucLowerRow);
    return retVal;
}


//*****************************************************************************
//
// The main event.  Configure all peripherals, do cool stuff
//
//*****************************************************************************
void main(void) {
    unsigned char stayUp = 0;

    configDS();
    configRTCC();
    configGPIOs();
    configTimer0();
    configTimer1();
    configTimer2();
    configTimer3();

    //
    // if we woke due to deep sleep, we won't be able to change the GPIO values
    // until we release them from deep sleep lock.
    //
    if(WDTCONbits.DS == 1)
    {
        //
        // Clear deep sleep status bits and release GPIOS from deep sleep lock
        //
        WDTCONbits.DS = 0;
        DSCONLbits.RELEASE = 0;

        //
        // load deep sleep persistent status values from retention register into
        // RAM
        //
        g_sDSGPR0.ucRaw = DSGPR0;
        g_sDSGPR1.ucRaw = DSGPR1;

        //
        // copy debounce states to global variables
        //
        g_ucPB0State = g_sDSGPR0.PB0State;
        g_ucPB1State = g_sDSGPR0.PB1State;

        //
        // Now that we're awake, turn on the LEDs
        //
        displayTime();

        //
        // Determine wake source
        //
        if(DSWAKELbits.DSRTC)
        {
            //
            // Eventually, we might do something nifty here.... I can't think of
            // a functionality to map to this yet.
            //
            // If we're here, we woke up because of RTC chime
        }
        if(DSWAKEHbits.DSINT0)
        {
            //
            // What he said ^
            //
            // If we're here, we woke up because of INT0 status change
            //
        }

    }
    else
    {
        //
        // This isn't a deep sleep wake, it's a full on cold boot.
        //

        //TODO: shouldn't this be moved to the configGPIO function??
        //
        // Init all GPIO outs.
        //
        LATA = 0;
        LATB = 0;
        LATC = 0;

        //
        // Init deep sleep persistent variables
        //
        g_sDSGPR0.PB0State = PB_STATE_IDLE;
        g_sDSGPR0.PB1State = PB_STATE_IDLE;
        g_sDSGPR1.blackout = 0;
        g_sDSGPR1.blinkState = 1;
        g_sDSGPR1.dispState = DISP_STATE_TIME_S;
        g_sDSGPR1.setState = SET_STATE_LOWER;
        DSGPR0 = g_sDSGPR0.ucRaw;
        DSGPR1 = g_sDSGPR1.ucRaw;
    }

    //
    // enable global interrupts
    //
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;

    INTCON2bits.INTEDG0 = 1;
    INTCONbits.INT0IF = 0;
    INTCONbits.INT0IE = 1;

    g_ucNapTime = 0;
    stayUp = 0;
    
//#if CONFIG_BUILD == 1
//
// In previous builds, the RTC CLK was routed to to the RTCCFG test point, and
// a special build could be used to sample that output for calibration.  Since
// switching the RTC clock source from INTOSC to accurate external XTAL, this is
// no longer necessary.  Might be useful for reference someday, though.
//
//    //ALRMCFGbits.CHIME = 0;
//    //ALRMCFGbits.ALRMEN = 0;
//    PADCFG1bits.RTSECSEL0 = 0;
//    PADCFG1bits.RTSECSEL1 = 1;
//    RTCCFGbits.RTCOE = 1;
//    while(1);
//#endif

    while (1)
    {

//#define DEBUG 1
#ifdef DEBUG
        //
        // Random one off functionality tests go here
        //
        while(1)
        {
            LATCbits.LATC6 = 1;
            LATCbits.LATC5 = 1;
            LATCbits.LATC7 = 1;
            TRISCbits.TRISC5 = 0;
            TRISCbits.TRISC6 = 0;
            TRISCbits.TRISC7 = 0;
            PORTCbits.RC6 = 1;
            PORTCbits.RC5 = 1;
            PORTCbits.RC7 = 1;
            //updateRows(0xff, 0xff);
        }
        //
        // Set prescaler to 1:4
        //
        T2CONbits.T2CKPS = 0x2;

        //
        // zero out timer
        //
        TMR2 = 0;

        T2CONbits.TMR2ON = 1;

        while(1)

            /*
            //
            // Turn on timer
            //
            //T3CONbits.TMR3ON = 1;
            if(TMR2 >= 0xFF)
            {
                TMR2 = 0;
                stayUp++;
                if(stayUp >= 50)
                {
                    if(toggle)
                    {
                        PORTBbits.RB2 = 1;
                        PORTBbits.RB3 = 1;
                        PORTBbits.RB4 = 1;
                        PORTBbits.RB5 = 1;
                        toggle = 0;
                    }
                    else
                    {
                        PORTBbits.RB2 = 0;
                        PORTBbits.RB3 = 0;
                        PORTBbits.RB4 = 0;
                        PORTBbits.RB5 = 0;
                        toggle = 1;
                    }
                    stayUp = 0;
                }

            }
        }
             * */
        displayTime();

        if(PB0_PORT_BITS.PB0_PIN == 0)
        {
            U_ROW_PORT_PINS.U_L1_PIN = 1;
        }
        else
        {
            U_ROW_PORT_PINS.U_L1_PIN = 0;
        }

        if(PB1_PORT_BITS.PB1_PIN == 0)
        {
            U_ROW_PORT_PINS.U_L2_PIN = 1;
        }
        else
        {
            U_ROW_PORT_PINS.U_L2_PIN = 0;
        }

        if(PORTBbits.RB0)
        {
            INTCON2bits.INTEDG0 = 0;
        }
        else
        {
            INTCON2bits.INTEDG0 = 1;
        }
        if(PB0_PORT_BITS.PB0_PIN == 1)
        {
            while(RTCCFGbits.RTCSYNC);

            //
            // make sure all ints are cleared and enabled, then go to sleep
            //
            //PIR3bits.RTCCIF = 0;
            //PIE3bits.RTCCIE = 0;

            if(INTCONbits.INT0IF)
            {
                INTCONbits.INT0IF = 0;
                if(PORTBbits.RB0)
                {
                    INTCON2bits.INTEDG0 = 0;
                }
                else
                {
                    INTCON2bits.INTEDG0 = 1;
                }
            }
            INTCONbits.TMR0IF = 0;
            PIR1bits.TMR1IF = 0;
            //INTCONbits.INT0IF = 0;
            //INTCONbits.INT0IE = 1;
            //ALRMCFGbits.ALRMEN = 0;
            //ALRMCFGbits.CHIME = 0;
            enterDS();
        }
#else

        //
        // Determine if either button is being interfaced with
        //
        stayUp = 0;
        stayUp += debouncePB0(&PressPB0, &HoldPB0);
        stayUp += debouncePB1(&PressPB1, &HoldPB1);
        
        //
        // Update the display LEDs
        //
        stayUp += displayTime();

        //
        // If the sleep button was held, black out all the lights.  Doing this
        // outside of the below "go to deep sleep" loop gives us the ability to
        // get the LEDs to appear at ~half brightness once the user has held the
        // sleep pushbutton long enough to go to sleep.  This was necessary b/c
        // if we sleep as soon as the user has held the PB long enough, we lose
        // the ability to debounce the PB release.  But if we don't sleep until
        // the PB is released, the user loses the visual feedback letting them
        // know the button has been held long enough for sleep to occur.
        // FIXME: this could cause problems if sleep is held while in the
        //        middle of setting the time...
        // FIXED: JTW: HoldPB0 updated to only change nap time if we're
        //        in a display state
        //
        if(g_ucNapTime)
        {
            PORTC = 0;
            PORTB = 0;
            g_sDSGPR1.blackout = 1;
        }

        //
        // If the push buttons don't need us to stay awake, and we're in a
        // display state (as opposed to a set state), we can go into deep sleep.
        //        
        if( !stayUp &&
            ((g_sDSGPR1.dispState == DISP_STATE_TIME_S) ||
            (g_sDSGPR1.dispState == DISP_STATE_TIME_H) ||
            (g_sDSGPR1.dispState == DISP_STATE_DATE)))
        {
            //
            // determine polarity of PB0, so we can know what edge to wake on
            //
            if(PB0)
            {
                INTCON2bits.INTEDG0 = 0;
            }
            else
            {
                INTCON2bits.INTEDG0 = 1;
            }
            
            //
            // If we're going down because the blackout PB was held, turn off
            // the LEDs and only wake up when a pushbutton is hit again
            //
            if(g_ucNapTime)
            {
                PORTC = 0;
                PORTB = 0;
                ALRMCFGbits.CHIME = 0;
                ALRMCFGbits.ALRMEN = 0;
                INTCON2bits.INTEDG0 = 0;
                g_sDSGPR1.blackout = 1;
            }

            //
            // wait until RTCSYNC is clear
            //
            while(RTCCFGbits.RTCSYNC);

            //
            // make sure all ints are cleared and enabled, then go to sleep
            //
            INTCONbits.INT0IF = 0;
            INTCONbits.INT0IE = 1;
            INTCONbits.TMR0IF = 0;
            PIR1bits.TMR1IF = 0;
            PIR3bits.RTCCIF = 0;
            enterDS();
          }
#endif
    }
}



