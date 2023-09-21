/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F24Q10
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
        MPLAB 	          :  MPLAB X 6.00	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set ANA0 aliases
#define ANA0_TRIS                 TRISAbits.TRISA0
#define ANA0_LAT                  LATAbits.LATA0
#define ANA0_PORT                 PORTAbits.RA0
#define ANA0_WPU                  WPUAbits.WPUA0
#define ANA0_OD                   ODCONAbits.ODCA0
#define ANA0_ANS                  ANSELAbits.ANSELA0
#define ANA0_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define ANA0_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define ANA0_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define ANA0_GetValue()           PORTAbits.RA0
#define ANA0_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define ANA0_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define ANA0_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define ANA0_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define ANA0_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define ANA0_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define ANA0_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define ANA0_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set ANA1 aliases
#define ANA1_TRIS                 TRISAbits.TRISA1
#define ANA1_LAT                  LATAbits.LATA1
#define ANA1_PORT                 PORTAbits.RA1
#define ANA1_WPU                  WPUAbits.WPUA1
#define ANA1_OD                   ODCONAbits.ODCA1
#define ANA1_ANS                  ANSELAbits.ANSELA1
#define ANA1_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define ANA1_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define ANA1_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define ANA1_GetValue()           PORTAbits.RA1
#define ANA1_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define ANA1_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define ANA1_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define ANA1_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define ANA1_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define ANA1_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define ANA1_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define ANA1_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set RB0 procedures
#define RB0_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define RB0_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define RB0_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define RB0_GetValue()              PORTBbits.RB0
#define RB0_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define RB0_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define RB0_SetPullup()             do { WPUBbits.WPUB0 = 1; } while(0)
#define RB0_ResetPullup()           do { WPUBbits.WPUB0 = 0; } while(0)
#define RB0_SetAnalogMode()         do { ANSELBbits.ANSELB0 = 1; } while(0)
#define RB0_SetDigitalMode()        do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set RB1 procedures
#define RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define RB1_GetValue()              PORTBbits.RB1
#define RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define RB1_SetPullup()             do { WPUBbits.WPUB1 = 1; } while(0)
#define RB1_ResetPullup()           do { WPUBbits.WPUB1 = 0; } while(0)
#define RB1_SetAnalogMode()         do { ANSELBbits.ANSELB1 = 1; } while(0)
#define RB1_SetDigitalMode()        do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set SSDA aliases
#define SSDA_TRIS                 TRISBbits.TRISB2
#define SSDA_LAT                  LATBbits.LATB2
#define SSDA_PORT                 PORTBbits.RB2
#define SSDA_WPU                  WPUBbits.WPUB2
#define SSDA_OD                   ODCONBbits.ODCB2
#define SSDA_ANS                  ANSELBbits.ANSELB2
#define SSDA_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define SSDA_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define SSDA_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define SSDA_GetValue()           PORTBbits.RB2
#define SSDA_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define SSDA_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define SSDA_SetPullup()          do { WPUBbits.WPUB2 = 1; } while(0)
#define SSDA_ResetPullup()        do { WPUBbits.WPUB2 = 0; } while(0)
#define SSDA_SetPushPull()        do { ODCONBbits.ODCB2 = 0; } while(0)
#define SSDA_SetOpenDrain()       do { ODCONBbits.ODCB2 = 1; } while(0)
#define SSDA_SetAnalogMode()      do { ANSELBbits.ANSELB2 = 1; } while(0)
#define SSDA_SetDigitalMode()     do { ANSELBbits.ANSELB2 = 0; } while(0)

// get/set SSCL aliases
#define SSCL_TRIS                 TRISBbits.TRISB3
#define SSCL_LAT                  LATBbits.LATB3
#define SSCL_PORT                 PORTBbits.RB3
#define SSCL_WPU                  WPUBbits.WPUB3
#define SSCL_OD                   ODCONBbits.ODCB3
#define SSCL_ANS                  ANSELBbits.ANSELB3
#define SSCL_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define SSCL_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define SSCL_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define SSCL_GetValue()           PORTBbits.RB3
#define SSCL_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define SSCL_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define SSCL_SetPullup()          do { WPUBbits.WPUB3 = 1; } while(0)
#define SSCL_ResetPullup()        do { WPUBbits.WPUB3 = 0; } while(0)
#define SSCL_SetPushPull()        do { ODCONBbits.ODCB3 = 0; } while(0)
#define SSCL_SetOpenDrain()       do { ODCONBbits.ODCB3 = 1; } while(0)
#define SSCL_SetAnalogMode()      do { ANSELBbits.ANSELB3 = 1; } while(0)
#define SSCL_SetDigitalMode()     do { ANSELBbits.ANSELB3 = 0; } while(0)

// get/set Tst1 aliases
#define Tst1_TRIS                 TRISBbits.TRISB5
#define Tst1_LAT                  LATBbits.LATB5
#define Tst1_PORT                 PORTBbits.RB5
#define Tst1_WPU                  WPUBbits.WPUB5
#define Tst1_OD                   ODCONBbits.ODCB5
#define Tst1_ANS                  ANSELBbits.ANSELB5
#define Tst1_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define Tst1_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define Tst1_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define Tst1_GetValue()           PORTBbits.RB5
#define Tst1_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define Tst1_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define Tst1_SetPullup()          do { WPUBbits.WPUB5 = 1; } while(0)
#define Tst1_ResetPullup()        do { WPUBbits.WPUB5 = 0; } while(0)
#define Tst1_SetPushPull()        do { ODCONBbits.ODCB5 = 0; } while(0)
#define Tst1_SetOpenDrain()       do { ODCONBbits.ODCB5 = 1; } while(0)
#define Tst1_SetAnalogMode()      do { ANSELBbits.ANSELB5 = 1; } while(0)
#define Tst1_SetDigitalMode()     do { ANSELBbits.ANSELB5 = 0; } while(0)

// get/set Vsns aliases
#define Vsns_TRIS                 TRISCbits.TRISC2
#define Vsns_LAT                  LATCbits.LATC2
#define Vsns_PORT                 PORTCbits.RC2
#define Vsns_WPU                  WPUCbits.WPUC2
#define Vsns_OD                   ODCONCbits.ODCC2
#define Vsns_ANS                  ANSELCbits.ANSELC2
#define Vsns_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define Vsns_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define Vsns_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define Vsns_GetValue()           PORTCbits.RC2
#define Vsns_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define Vsns_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define Vsns_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define Vsns_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define Vsns_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define Vsns_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define Vsns_SetAnalogMode()      do { ANSELCbits.ANSELC2 = 1; } while(0)
#define Vsns_SetDigitalMode()     do { ANSELCbits.ANSELC2 = 0; } while(0)

// get/set RC6 procedures
#define RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define RC6_GetValue()              PORTCbits.RC6
#define RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define RC6_SetPullup()             do { WPUCbits.WPUC6 = 1; } while(0)
#define RC6_ResetPullup()           do { WPUCbits.WPUC6 = 0; } while(0)
#define RC6_SetAnalogMode()         do { ANSELCbits.ANSELC6 = 1; } while(0)
#define RC6_SetDigitalMode()        do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set RC7 procedures
#define RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RC7_GetValue()              PORTCbits.RC7
#define RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define RC7_SetPullup()             do { WPUCbits.WPUC7 = 1; } while(0)
#define RC7_ResetPullup()           do { WPUCbits.WPUC7 = 0; } while(0)
#define RC7_SetAnalogMode()         do { ANSELCbits.ANSELC7 = 1; } while(0)
#define RC7_SetDigitalMode()        do { ANSELCbits.ANSELC7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/