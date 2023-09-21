#pragma warning disable 759  //expression generates no code
#pragma warning disable 760  //portion of expression has no effect
#pragma warning disable 520  //function is never called
#pragma warning disable 1498 //pointer (unknown) in expression may have no targets
#pragma warning disable 1510 //non-reentrant function appears in multiple call graphs and has been duplicated by the compiler
#pragma warning disable 1090 //variable is not used
#pragma warning disable 2053 //function is never called

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "mcc_generated_files/mcc.h"

#include "lis2dh12_reg.h"

/*---------------------------------------------------------------------------*/
static volatile __persistent uint16_t delay_1ms;
static volatile __persistent uint16_t _1ms;
static volatile __persistent bool _1ms_flag;

void delay_ms(uint16_t ms)
{
 delay_1ms = ms;
 while (delay_1ms) CLRWDT(); 
}

void tmr0_hnd(void) 
{ //1000us
 if (delay_1ms) delay_1ms--;
 
 _1ms++;
 _1ms_flag = true;
 
 ADCON0bits.ADON = 1;
 ADCON0bits.ADGO = 1;
}

void get_ad(void)
{
 while(ADCON0bits.ADGO);   /* Wait for AD conversion complete ~37u */
 int16_t res = (int16_t)ADCC_GetConversionResult();
 ADCON0bits.ADON = 0;
} 
/*---------------------------------------------------------------------------*/

static volatile  uint8_t Int1Flag = 0;
static volatile  uint8_t Int2Flag = 0;
static volatile  bool interrupted1 = false;
static volatile  bool interrupted2 = false;

void Int1(void)
{
 Int1Flag = 1;
 interrupted1 = true;
}

void Int2(void)
{
 Int2Flag = 1;
 interrupted2 = true;
}
/*---------------------------------------------------------------------------*/


/*
                         Main application
 */
static int16_t data_raw_acceleration[3];
static int16_t acceleration_mg[3];
static uint8_t whoamI;

void main(void)
{

 // Initialize the device
 SYSTEM_Initialize();
 RB0_SetDigitalInput();
 RB1_SetDigitalInput();
 RB0_SetPullup();
 RB1_SetPullup();
 Vsns_LAT = 0;
 TMR0_SetInterruptHandler(tmr0_hnd);
 ADCC_SetADIInterruptHandler(get_ad);

 INT0_SetInterruptHandler(Int1);
 INT1_SetInterruptHandler(Int2);

 // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
 // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
 // Use the following macros to:

 // Enable the Global Interrupts
 INTERRUPT_GlobalInterruptEnable();

 // Disable the Global Interrupts
 //INTERRUPT_GlobalInterruptDisable();

 // Enable the Peripheral Interrupts
 INTERRUPT_PeripheralInterruptEnable();
 EXT_INT0_InterruptDisable();

 printf("\e[2J\e[H"); //Clear screen and home
 printf("Hello!\n\r");
 delay_ms(10);
 lis2dh12_device_id_get(&whoamI);
 printf("LIS2DH12 ID: %d\n\r", whoamI);
 if (whoamI != LIS2DH12_ID) printf("Device not found\n\r");
 else
   {//AN5005 6.3.3 Using the HP filter
    LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG0, 0x90); // Disconnect SDO/SA0 pull-up.
    LIS2DH12_Write1ByteRegister(LIS2DH12_TEMP_CFG_REG, 0x00); //Temperature sensor disabled
    LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG1, 0x3F); // Turn on the sensor and enable X, Y, and Z
                                                           // low-power mode, ODR = 50 Hz
    LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG2, 0x09); // High-pass filter disabled
    LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG3, 0x40); // Interrupt activity 1 driven to INT1 pad
    LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG4, 0x00); // FS = �2 g
    state_resolution = LIS2DH12_LP_8bit;
    state_scale = LIS2DH12_2g;
    LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG5, 0x08); // Interrupt 1 pin latched
    //LIS2DH12_Write1ByteRegister(LIS2DH12_INT1_THS, 0x10);  // Threshold = 250 mg
    LIS2DH12_Write1ByteRegister(LIS2DH12_INT1_THS, 0x8);  // Threshold = 125 mg
    LIS2DH12_Write1ByteRegister(LIS2DH12_INT1_DURATION, 0x00); // Duration = 0
                                                           // Dummy read to force the HP filter to
    LIS2DH12_Read1ByteRegister(LIS2DH12_REFERENCE);        // current acceleration value
                                                           // (i.e. set reference acceleration/tilt value)
    LIS2DH12_Write1ByteRegister(LIS2DH12_INT1_CFG, 0x2A);  // Enable XH and YH interrupt generation (2A for XYZ)
    EXT_INT0_InterruptEnable();
    while(1)
    {
     lis2dh12_int1_src_t src;
     if (Int1Flag)
      {
       Int1Flag = false;
       src.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT1_SRC);
       lis2dh12_acceleration_raw_get(data_raw_acceleration);
       acceleration_mg[0] = lis2dh12_getAcceleration(data_raw_acceleration[0]);   
       acceleration_mg[1] = lis2dh12_getAcceleration(data_raw_acceleration[1]);   
       acceleration_mg[2] = lis2dh12_getAcceleration(data_raw_acceleration[2]);
       printf("\r\nInt1 %02X [mg]:% 5d % 5d % 5d\r\n", src.byte,
           acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      } 
     if (_1ms_flag)
      {
       if ((_1ms%1000) == 0) printf(". "); 
       _1ms_flag = false;
      }
     CLRWDT();
    }
   }   

/**
 End of File
 */