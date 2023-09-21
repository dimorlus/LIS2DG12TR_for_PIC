/**
 * Based on ST lis2dh12 driver. 
 * Modified and optimized for PIC (and other 8-bit MCUs). 
 * Use functions with the "_o" suffix, the original ones are left for 
 * compatibility.
  ******************************************************************************
  * @file    lis2dh12_reg.c
  * @author  Sensors Software Solution Team
  * @brief   LIS2DH12 driver file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include <xc.h>
#include <stdbool.h>
#include "mcc_generated_files/mcc.h"
#include "lis2dh12_reg.h"



lis2dh12_fs_t state_scale = 0xff;
lis2dh12_op_md_t state_resolution = 0xff;


/**
  * @defgroup  LIS2DH12_Interfaces_Functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
  * @{
  *
  */
typedef struct
{
    uint8_t *data;
    uint8_t *dataSize;
} pageWriteSize_t;


static i2c1_operations_t rdBlockCompleteHandler(void *payload)
 {
  pageWriteSize_t *pgData = payload;
  I2C1_SetBuffer(pgData->data,*pgData->dataSize);
  I2C1_SetDataCompleteCallback(NULL,NULL);
  return I2C1_RESTART_READ;
 }

static i2c1_operations_t writeData(void *payload)
 {
  pageWriteSize_t *pgData = payload;
  I2C1_SetBuffer(pgData->data,*pgData->dataSize);
  I2C1_SetDataCompleteCallback(NULL,NULL);
  return I2C1_CONTINUE;
 }

static i2c1_operations_t rd1RegCompleteHandler(void *ptr)
{
 I2C1_SetBuffer(ptr,1);
 I2C1_SetDataCompleteCallback(NULL,NULL);
 return I2C1_RESTART_READ;
}

static i2c1_operations_t wr1RegCompleteHandler(void *ptr)
{
 I2C1_SetBuffer(ptr,1);
 I2C1_SetDataCompleteCallback(NULL,NULL);
 return I2C1_CONTINUE;
}

uint8_t LIS2DH12_Read1ByteRegister(uint8_t reg)
{
 uint8_t returnValue = 0x00;

 reg |= 0x80;
 while(!I2C1_Open(LIS2DH12_I2C_ADD)); // sit here until we get the bus..
 I2C1_SetDataCompleteCallback(rd1RegCompleteHandler,&returnValue);
 I2C1_SetBuffer(&reg,1);
 I2C1_SetAddressNackCallback(NULL,NULL); //NACK polling?
 I2C1_MasterWrite();
 while(I2C1_BUSY == I2C1_Close()); // sit here until finished.

 return returnValue;
}

void LIS2DH12_Write1ByteRegister(uint8_t reg, uint8_t data)
{
 reg |= 0x80;
 while(!I2C1_Open(LIS2DH12_I2C_ADD)); // sit here until we get the bus..
 I2C1_SetDataCompleteCallback(wr1RegCompleteHandler,&data);
 I2C1_SetBuffer(&reg,1);
 I2C1_SetAddressNackCallback(NULL,NULL); //NACK polling?
 I2C1_MasterWrite();
 while(I2C1_BUSY == I2C1_Close()); // sit here until finished.
}

int8_t lis2dh12_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
 i2c1_error_t e;
 pageWriteSize_t pgData = {0};
 pgData.data = data;
 pgData.dataSize = &len;
 
 reg |= 0x80; //For multi byte reads we must set the first bit to 1
 while(!I2C1_Open(LIS2DH12_I2C_ADD));
 I2C1_SetDataCompleteCallback(rdBlockCompleteHandler,&pgData);
 I2C1_SetBuffer(&reg, 1);
 I2C1_SetAddressNackCallback(NULL,NULL); //NACK polling?
 I2C1_MasterWrite();
 while(I2C1_BUSY == (e = I2C1_Close())); // sit here until finished.
 if (e==I2C1_NOERR) return 0;
 else return 1;
}


int8_t lis2dh12_write_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
 i2c1_error_t e;
 
 reg |= 0x80; //For multi byte writes we must set the first bit to 1
 while(!I2C1_Open(LIS2DH12_I2C_ADD)); // sit here until we get the bus..
 I2C1_SetDataCompleteCallback(wr1RegCompleteHandler, data);
 I2C1_SetBuffer(&reg,1);
 I2C1_SetAddressNackCallback(NULL,NULL); //NACK polling?
 I2C1_MasterWrite();
 while(I2C1_BUSY == (e = I2C1_Close())); // sit here until finished.
 if (e==I2C1_NOERR) return 0;
 else return 1;
}


/**
  * @defgroup  LIS2DH12_Data_generation
  * @brief     This section group all the functions concerning data generation.
  * @{
  *
  */

/**
  * @brief  Temperature status register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_temp_status_reg_get(uint8_t *buff)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_STATUS_REG_AUX, buff, 1);

  return ret;
}

uint8_t lis2dh12_temp_status_reg_get_o(void)
{
  return LIS2DH12_Read1ByteRegister(LIS2DH12_STATUS_REG_AUX);
}

/**
  * @brief  Temperature data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tda in reg STATUS_REG_AUX
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_temp_data_ready_get(uint8_t *val)
{
  lis2dh12_status_reg_aux_t status_reg_aux;
  int8_t ret;
  ret = lis2dh12_read_reg(LIS2DH12_STATUS_REG_AUX, (uint8_t *)&status_reg_aux, 1);
  *val = status_reg_aux.tda;
  return ret;
}

uint8_t lis2dh12_temp_data_ready_get_o(void)
{
  lis2dh12_status_reg_aux_t status_reg_aux;
  status_reg_aux.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_STATUS_REG_AUX);
  return status_reg_aux.tda;
}
/**
  * @brief  Temperature data overrun.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tor in reg STATUS_REG_AUX
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_temp_data_ovr_get(uint8_t *val)
{
  lis2dh12_status_reg_aux_t status_reg_aux;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_STATUS_REG_AUX, (uint8_t *)&status_reg_aux, 1);
  *val = status_reg_aux.tor;

  return ret;
}

uint8_t lis2dh12_temp_data_ovr_get_o(void)
{
  lis2dh12_status_reg_aux_t status_reg_aux;
  status_reg_aux.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_STATUS_REG_AUX);
  return status_reg_aux.tor;
}

/**
  * @brief  Temperature output value.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_temperature_raw_get(int16_t *val)
{
  uint8_t buff[2];
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_OUT_TEMP_L, buff, 2);
  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

int16_t lis2dh12_temperature_raw_get_o(void)
{
  uint8_t buff[2] = {0};
  int16_t val;
  (void)lis2dh12_read_reg(LIS2DH12_OUT_TEMP_L, buff, 2);
  val = (int16_t)buff[1];
  val = (val * 256) + (int16_t)buff[0];

  return val;
}
/**
  * @brief  Temperature sensor enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of temp_en in reg TEMP_CFG_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_temperature_meas_set(lis2dh12_temp_en_t val)
{
  lis2dh12_temp_cfg_reg_t temp_cfg_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_TEMP_CFG_REG, (uint8_t *)&temp_cfg_reg, 1);

  if (ret == 0)
  {
    temp_cfg_reg.temp_en = (uint8_t) val;
    ret = lis2dh12_write_reg(LIS2DH12_TEMP_CFG_REG, (uint8_t *)&temp_cfg_reg, 1);
  }

  return ret;
}

void lis2dh12_temperature_meas_set_o(lis2dh12_temp_en_t val)
{
  lis2dh12_temp_cfg_reg_t temp_cfg_reg;
  temp_cfg_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_TEMP_CFG_REG);
  temp_cfg_reg.temp_en = (uint8_t) val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_TEMP_CFG_REG, temp_cfg_reg.byte);
}

/**
 * @brief  Temperature sensor enable.[get]
 *
 * @param  ctx      read / write interface definitions
 * @param  val      get the values of temp_en in reg TEMP_CFG_REG
 * @retval          interface status (MANDATORY: return 0 -> no Error)
 *
 */
int8_t lis2dh12_temperature_meas_get(lis2dh12_temp_en_t *val)
{
 lis2dh12_temp_cfg_reg_t temp_cfg_reg;
 int8_t ret;

 ret = lis2dh12_read_reg(LIS2DH12_TEMP_CFG_REG, (uint8_t *) & temp_cfg_reg, 1);

 switch(temp_cfg_reg.temp_en)
 {
  case LIS2DH12_TEMP_DISABLE:
   *val = LIS2DH12_TEMP_DISABLE;
   break;

  case LIS2DH12_TEMP_ENABLE:
   *val = LIS2DH12_TEMP_ENABLE;
   break;

  default:
   *val = LIS2DH12_TEMP_DISABLE;
   break;
 }

 return ret;
}

lis2dh12_temp_en_t lis2dh12_temperature_meas_get_o(void)
{
 lis2dh12_temp_cfg_reg_t temp_cfg_reg;
 temp_cfg_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_TEMP_CFG_REG);
 switch(temp_cfg_reg.temp_en)
 {
  case LIS2DH12_TEMP_DISABLE:
   return LIS2DH12_TEMP_DISABLE;
  case LIS2DH12_TEMP_ENABLE:
   return LIS2DH12_TEMP_ENABLE;
  default:
   return LIS2DH12_TEMP_DISABLE;
 }
}

/**
  * @brief  Operating mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpen in reg CTRL_REG1
  *                  and HR in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_operating_mode_set(lis2dh12_op_md_t val)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);

  if (ret == 0)
  {
    ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  }
  state_resolution = 0xff; 
  if (ret == 0)
  {
    if (val == LIS2DH12_HR_12bit)
    {
      ctrl_reg1.lpen = 0;
      ctrl_reg4.hr   = 1;
      state_resolution = LIS2DH12_HR_12bit;
    }
    else 
    if (val == LIS2DH12_NM_10bit)
    {
      ctrl_reg1.lpen = 0;
      ctrl_reg4.hr   = 0;
      state_resolution = LIS2DH12_NM_10bit;
    }
    else
    if (val == LIS2DH12_LP_8bit)
    {
      ctrl_reg1.lpen = 1;
      ctrl_reg4.hr   = 0;
      state_resolution = LIS2DH12_LP_8bit;
    }

    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
  }

  if (ret == 0)
  {
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  }

  return ret;
}

void lis2dh12_operating_mode_set_o(lis2dh12_op_md_t val)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg1.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG1);
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  state_resolution = 0xff; 
  if (val == LIS2DH12_HR_12bit)
  {
    ctrl_reg1.lpen = 0;
    ctrl_reg4.hr   = 1;
    state_resolution = LIS2DH12_HR_12bit;
  }
  else 
  if (val == LIS2DH12_NM_10bit)
  {
    ctrl_reg1.lpen = 0;
    ctrl_reg4.hr   = 0;
    state_resolution = LIS2DH12_NM_10bit;
  }
  else
  if (val == LIS2DH12_LP_8bit)
  {
    ctrl_reg1.lpen = 1;
    ctrl_reg4.hr   = 0;
    state_resolution = LIS2DH12_LP_8bit;
  }
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG1, ctrl_reg1.byte);
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG4, ctrl_reg4.byte);
}
/**
  * @brief  Operating mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpen in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_operating_mode_get(lis2dh12_op_md_t *val)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);

  if (ret == 0)
  {
    ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

    if (ctrl_reg1.lpen == PROPERTY_ENABLE)
    {
      *val = LIS2DH12_LP_8bit;
      state_resolution = LIS2DH12_LP_8bit;
    }
    else 
    if (ctrl_reg4.hr == PROPERTY_ENABLE)
    {
      *val = LIS2DH12_HR_12bit;
      state_resolution = LIS2DH12_HR_12bit;
    }
    else
    {
      *val = LIS2DH12_NM_10bit;
      state_resolution = LIS2DH12_NM_10bit;
    }
  }

  return ret;
}

lis2dh12_op_md_t lis2dh12_operating_mode_get_o(void)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  lis2dh12_op_md_t val;
  ctrl_reg1.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG1);
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);

  if (ctrl_reg1.lpen == PROPERTY_ENABLE)
   {
    val = LIS2DH12_LP_8bit;
    state_resolution = LIS2DH12_LP_8bit;
   }
  else 
  if (ctrl_reg4.hr == PROPERTY_ENABLE)
   {
    val = LIS2DH12_HR_12bit;
    state_resolution = LIS2DH12_HR_12bit;
   }
  else
   {
    val = LIS2DH12_NM_10bit;
    state_resolution = LIS2DH12_NM_10bit;
   }
  return val;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_data_rate_set(lis2dh12_odr_t val)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);

  if (ret == 0)
  {
    ctrl_reg1.odr = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
  }

  return ret;
}

void lis2dh12_data_rate_set_o(lis2dh12_odr_t val)
 {
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  ctrl_reg1.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG1);
  ctrl_reg1.odr = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG1, ctrl_reg1.byte);
 }

/**
 * Convert sample_rate values to frequency.
 */
int lis2dh12_odr_to_hz(lis2dh12_odr_t sample_rate)
{
 sample_rate &= LIS2DH12_ODR_MASK;
 switch(sample_rate) 
  {
    case LIS2DH12_POWER_DOWN: return 0;
    case LIS2DH12_ODR_1Hz: return 1;
    case LIS2DH12_ODR_10Hz: return 10;
    case LIS2DH12_ODR_25Hz: return 25;
    case LIS2DH12_ODR_50Hz: return 50;
    case LIS2DH12_ODR_100Hz: return 100;
    case LIS2DH12_ODR_200Hz: return 200;
    case LIS2DH12_ODR_400Hz: return 400;
    default: return 0; /** 1k+ rates not implemented */
  }
}


/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of odr in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_data_rate_get(lis2dh12_odr_t *val)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);

  switch (ctrl_reg1.odr)
  {
    case LIS2DH12_POWER_DOWN:
      *val = LIS2DH12_POWER_DOWN;
      break;

    case LIS2DH12_ODR_1Hz:
      *val = LIS2DH12_ODR_1Hz;
      break;

    case LIS2DH12_ODR_10Hz:
      *val = LIS2DH12_ODR_10Hz;
      break;

    case LIS2DH12_ODR_25Hz:
      *val = LIS2DH12_ODR_25Hz;
      break;

    case LIS2DH12_ODR_50Hz:
      *val = LIS2DH12_ODR_50Hz;
      break;

    case LIS2DH12_ODR_100Hz:
      *val = LIS2DH12_ODR_100Hz;
      break;

    case LIS2DH12_ODR_200Hz:
      *val = LIS2DH12_ODR_200Hz;
      break;

    case LIS2DH12_ODR_400Hz:
      *val = LIS2DH12_ODR_400Hz;
      break;

    case LIS2DH12_ODR_1kHz620_LP:
      *val = LIS2DH12_ODR_1kHz620_LP;
      break;

    case LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP:
      *val = LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP;
      break;

    default:
      *val = LIS2DH12_POWER_DOWN;
      break;
  }

  return ret;
}

lis2dh12_odr_t lis2dh12_data_rate_get_o(void)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  ctrl_reg1.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG1);
  switch (ctrl_reg1.odr)
  {
    case LIS2DH12_POWER_DOWN:
      return LIS2DH12_POWER_DOWN;
    case LIS2DH12_ODR_1Hz:
      return LIS2DH12_ODR_1Hz;
    case LIS2DH12_ODR_10Hz:
      return LIS2DH12_ODR_10Hz;
    case LIS2DH12_ODR_25Hz:
      return LIS2DH12_ODR_25Hz;
    case LIS2DH12_ODR_50Hz:
      return LIS2DH12_ODR_50Hz;
    case LIS2DH12_ODR_100Hz:
      return LIS2DH12_ODR_100Hz;
    case LIS2DH12_ODR_200Hz:
      return LIS2DH12_ODR_200Hz;
    case LIS2DH12_ODR_400Hz:
      return LIS2DH12_ODR_400Hz;
    case LIS2DH12_ODR_1kHz620_LP:
      return LIS2DH12_ODR_1kHz620_LP;
    case LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP:
      return LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP;
    default:
      return LIS2DH12_POWER_DOWN;
  }
}
/**
  * @brief   High pass data from internal filter sent to output register
  *          and FIFO.
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fds in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_high_pass_on_outputs_set(uint8_t val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);

  if (ret == 0)
  {
    ctrl_reg2.fds = val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
  }

  return ret;
}

void lis2dh12_high_pass_on_outputs_set_o(uint8_t val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  ctrl_reg2.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG2);
  ctrl_reg2.fds = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG2, ctrl_reg2.byte);
}
/**
  * @brief   High pass data from internal filter sent to output register
  *          and FIFO.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fds in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_high_pass_on_outputs_get(uint8_t *val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
  *val = (uint8_t)ctrl_reg2.fds;

  return ret;
}

uint8_t lis2dh12_high_pass_on_outputs_get_o(void)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  ctrl_reg2.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG2);
  return (uint8_t)ctrl_reg2.fds;
}

/**
  * @brief   High-pass filter cutoff frequency selection.[set]
  *
  * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hpcf in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_high_pass_bandwidth_set(lis2dh12_hpcf_t val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);

  if (ret == 0)
  {
    ctrl_reg2.hpcf = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
  }

  return ret;
}

void lis2dh12_high_pass_bandwidth_set_o(lis2dh12_hpcf_t val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  ctrl_reg2.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG2);
  ctrl_reg2.hpcf = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG2, ctrl_reg2.byte);
}

/**
  * @brief   High-pass filter cutoff frequency selection.[get]
  *
  * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of hpcf in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_high_pass_bandwidth_get(lis2dh12_hpcf_t *val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);

  switch (ctrl_reg2.hpcf)
  {
    case LIS2DH12_AGGRESSIVE:
      *val = LIS2DH12_AGGRESSIVE;
      break;

    case LIS2DH12_STRONG:
      *val = LIS2DH12_STRONG;
      break;

    case LIS2DH12_MEDIUM:
      *val = LIS2DH12_MEDIUM;
      break;

    case LIS2DH12_LIGHT:
      *val = LIS2DH12_LIGHT;
      break;

    default:
      *val = LIS2DH12_LIGHT;
      break;
  }

  return ret;
}

lis2dh12_hpcf_t lis2dh12_high_pass_bandwidth_get_o(void)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  ctrl_reg2.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG2);
  switch (ctrl_reg2.hpcf)
  {
    case LIS2DH12_AGGRESSIVE:
      return LIS2DH12_AGGRESSIVE;
    case LIS2DH12_STRONG:
      return LIS2DH12_STRONG;
    case LIS2DH12_MEDIUM:
      return LIS2DH12_MEDIUM;
    case LIS2DH12_LIGHT:
      return LIS2DH12_LIGHT;
    default:
      return LIS2DH12_LIGHT;
  }
}

/**
  * @brief  High-pass filter mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hpm in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_high_pass_mode_set(lis2dh12_hpm_t val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);

  if (ret == 0)
  {
    ctrl_reg2.hpm = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
  }

  return ret;
}

void lis2dh12_high_pass_mode_set_o(lis2dh12_hpm_t val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  ctrl_reg2.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG2);
  ctrl_reg2.hpm = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG2, ctrl_reg2.byte);
}

/**
  * @brief  High-pass filter mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of hpm in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_high_pass_mode_get(lis2dh12_hpm_t *val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);

  switch (ctrl_reg2.hpm)
  {
    case LIS2DH12_NORMAL_WITH_RST:
      *val = LIS2DH12_NORMAL_WITH_RST;
      break;

    case LIS2DH12_REFERENCE_MODE:
      *val = LIS2DH12_REFERENCE_MODE;
      break;

    case LIS2DH12_NORMAL:
      *val = LIS2DH12_NORMAL;
      break;

    case LIS2DH12_AUTORST_ON_INT:
      *val = LIS2DH12_AUTORST_ON_INT;
      break;

    default:
      *val = LIS2DH12_NORMAL_WITH_RST;
      break;
  }

  return ret;
}

lis2dh12_hpm_t lis2dh12_high_pass_mode_get_o(void)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  ctrl_reg2.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG2);
  switch (ctrl_reg2.hpm)
  {
    case LIS2DH12_NORMAL_WITH_RST:
      return LIS2DH12_NORMAL_WITH_RST;
    case LIS2DH12_REFERENCE_MODE:
      return LIS2DH12_REFERENCE_MODE;
    case LIS2DH12_NORMAL:
      return LIS2DH12_NORMAL;
    case LIS2DH12_AUTORST_ON_INT:
      return LIS2DH12_AUTORST_ON_INT;
    default:
      return LIS2DH12_NORMAL_WITH_RST;
  }
}

/**
  * @brief  Full-scale configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_full_scale_set(lis2dh12_fs_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  if (ret == 0)
  {
    ctrl_reg4.fs = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  }
  state_scale = val;
  return ret;
}

void lis2dh12_full_scale_set_o(lis2dh12_fs_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  ctrl_reg4.fs = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG4, ctrl_reg4.byte);
  state_scale = val;
}

/**
 * Return full scale in mg for current state
 */
int lis2dh12_get_full_scale(void)
{
 switch(state_scale)
  {
   case LIS2DH12_2g:  return 2000;
   case LIS2DH12_4g:  return 4000;
   case LIS2DH12_8g:  return 8000;
   case LIS2DH12_16g: return 16000;
   default:           return 0;
  }
}

/**
  * @brief  Full-scale configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of fs in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_full_scale_get(lis2dh12_fs_t *val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  switch (ctrl_reg4.fs)
  {
    case LIS2DH12_2g:
      *val = LIS2DH12_2g;
      state_scale = LIS2DH12_2g;
      break;

    case LIS2DH12_4g:
      *val = LIS2DH12_4g;
      state_scale = LIS2DH12_4g;
      break;

    case LIS2DH12_8g:
      *val = LIS2DH12_8g;
      state_scale = LIS2DH12_8g;
      break;

    case LIS2DH12_16g:
      *val = LIS2DH12_16g;
      state_scale = LIS2DH12_16g;
      break;

    default:
      *val = LIS2DH12_2g;
      state_scale = LIS2DH12_2g;
      break;
  }
  return ret;
}

lis2dh12_fs_t lis2dh12_full_scale_get_o()
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  switch (ctrl_reg4.fs)
   {
    case LIS2DH12_2g:
      state_scale = LIS2DH12_2g;
      break;
    case LIS2DH12_4g:
      state_scale = LIS2DH12_4g;
      break;
    case LIS2DH12_8g:
      state_scale = LIS2DH12_8g;
      break;
    case LIS2DH12_16g:
      state_scale = LIS2DH12_16g;
      break;
    default:
      state_scale = LIS2DH12_2g;
      break;
   }
 return state_scale; 
}

/**
  * @brief  Block Data Update.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_block_data_update_set(uint8_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  if (ret == 0)
  {
    ctrl_reg4.bdu = val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  }

  return ret;
}

void lis2dh12_block_data_update_set_o(uint8_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  ctrl_reg4.bdu = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG4, ctrl_reg4.byte);
}

/**
  * @brief  Block Data Update.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_block_data_update_get(uint8_t *val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  *val = (uint8_t)ctrl_reg4.bdu;

  return ret;
}
uint8_t lis2dh12_block_data_update_get_o(void)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  return (uint8_t)ctrl_reg4.bdu;
}

/**
  * @brief  Reference value for interrupt generation.[set]
  *         LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_filter_reference_set(uint8_t *buff)
{
  int8_t ret;

  ret = lis2dh12_write_reg(LIS2DH12_REFERENCE, buff, 1);

  return ret;
}

void lis2dh12_filter_reference_set_o(uint8_t buff)
{
  LIS2DH12_Write1ByteRegister(LIS2DH12_REFERENCE, buff);
}

/**
  * @brief  Reference value for interrupt generation.[get]
  *         LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_filter_reference_get(uint8_t *buff)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_REFERENCE, buff, 1);

  return ret;
}

uint8_t lis2dh12_filter_reference_get_o(void)
{
  return LIS2DH12_Read1ByteRegister(LIS2DH12_REFERENCE);
}

/**
  * @brief  Acceleration set of data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of zyxda in reg STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_xl_data_ready_get(uint8_t *val)
{
  lis2dh12_status_reg_t status_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_STATUS_REG, (uint8_t *)&status_reg, 1);
  *val = status_reg.zyxda;

  return ret;
}

uint8_t lis2dh12_xl_data_ready_get_o(void)
{
  lis2dh12_status_reg_t status_reg;
  status_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_STATUS_REG);
  return status_reg.zyxda;
}

/**
  * @brief  Acceleration set of data overrun.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of zyxor in reg STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_xl_data_ovr_get(uint8_t *val)
{
  lis2dh12_status_reg_t status_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_STATUS_REG, (uint8_t *)&status_reg, 1);
  *val = status_reg.zyxor;

  return ret;
}

uint8_t lis2dh12_xl_data_ovr_get_o(void)
{
  lis2dh12_status_reg_t status_reg;
  status_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_STATUS_REG);
  return (uint8_t)status_reg.zyxor;
}

/**
  * @brief  Acceleration output value.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_acceleration_raw_get(int16_t *val)
{
  uint8_t buff[6]={0};
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_OUT_X_L, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DH12_Common
  * @brief     This section group common useful functions
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI .[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_device_id_get(uint8_t *buff)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_WHO_AM_I, buff, 1);

  return ret;
}

uint8_t lis2dh12_device_id_get_o(void)
{
  return LIS2DH12_Read1ByteRegister(LIS2DH12_WHO_AM_I);
}

/**
  * @brief  Self Test.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of st in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_self_test_set(lis2dh12_st_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  if (ret == 0)
  {
    ctrl_reg4.st = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  }

  return ret;
}

void lis2dh12_self_test_set_o(lis2dh12_st_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  ctrl_reg4.st = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG4, ctrl_reg4.byte);
}

/**
  * @brief  Self Test.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of st in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_self_test_get(lis2dh12_st_t *val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  switch (ctrl_reg4.st)
  {
    case LIS2DH12_ST_DISABLE:
      *val = LIS2DH12_ST_DISABLE;
      break;

    case LIS2DH12_ST_POSITIVE:
      *val = LIS2DH12_ST_POSITIVE;
      break;

    case LIS2DH12_ST_NEGATIVE:
      *val = LIS2DH12_ST_NEGATIVE;
      break;

    default:
      *val = LIS2DH12_ST_DISABLE;
      break;
  }

  return ret;
}

lis2dh12_st_t lis2dh12_self_test_get_o(void)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  switch (ctrl_reg4.st)
  {
    case LIS2DH12_ST_DISABLE:
      return LIS2DH12_ST_DISABLE;
    case LIS2DH12_ST_POSITIVE:
      return LIS2DH12_ST_POSITIVE;
    case LIS2DH12_ST_NEGATIVE:
      return LIS2DH12_ST_NEGATIVE;
    default:
      return LIS2DH12_ST_DISABLE;
  }
}
/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ble in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_data_format_set(lis2dh12_ble_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  if (ret == 0)
  {
    ctrl_reg4.ble = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  }

  return ret;
}

void lis2dh12_data_format_set_o(lis2dh12_ble_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  ctrl_reg4.ble = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG4, ctrl_reg4.byte);
}
/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of ble in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_data_format_get(lis2dh12_ble_t *val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  switch (ctrl_reg4.ble)
  {
    case LIS2DH12_LSB_AT_LOW_ADD:
      *val = LIS2DH12_LSB_AT_LOW_ADD;
      break;

    case LIS2DH12_MSB_AT_LOW_ADD:
      *val = LIS2DH12_MSB_AT_LOW_ADD;
      break;

    default:
      *val = LIS2DH12_LSB_AT_LOW_ADD;
      break;
  }

  return ret;
}

lis2dh12_ble_t lis2dh12_data_format_get_o(void)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  ctrl_reg4.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG4);
  switch (ctrl_reg4.ble)
  {
    case LIS2DH12_LSB_AT_LOW_ADD:
      return LIS2DH12_LSB_AT_LOW_ADD;
    case LIS2DH12_MSB_AT_LOW_ADD:
      return LIS2DH12_MSB_AT_LOW_ADD;
    default:
      return LIS2DH12_LSB_AT_LOW_ADD;
  }
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of boot in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_boot_set(uint8_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0)
  {
    ctrl_reg5.boot = val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

void lis2dh12_boot_set_o(uint8_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  ctrl_reg5.boot = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG5, ctrl_reg5.byte);
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of boot in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_boot_get(uint8_t *val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  *val = (uint8_t)ctrl_reg5.boot;

  return ret;
}

uint8_t lis2dh12_boot_get_o(void)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  return (uint8_t)ctrl_reg5.boot;
}

/**
  * @brief  Info about device status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_status_get(lis2dh12_status_reg_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_STATUS_REG, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_status_reg_t lis2dh12_status_get_o(void)
{
 lis2dh12_status_reg_t status_reg;
 status_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_STATUS_REG);
 return status_reg;
}


/**
  * @defgroup   LIS2DH12_Interrupts_generator_1
  * @brief      This section group all the functions that manage the first
  *             interrupts generator
  * @{
  *
  */

/**
  * @brief  Interrupt generator 1 configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register INT1_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_gen_conf_set(lis2dh12_int1_cfg_t *val)
{
  int8_t ret;

  ret = lis2dh12_write_reg(LIS2DH12_INT1_CFG, (uint8_t *) val, 1);

  return ret;
}

void lis2dh12_int1_gen_conf_set_o(lis2dh12_int1_cfg_t val)
{
  LIS2DH12_Write1ByteRegister(LIS2DH12_INT1_CFG, val.byte);
}
/**
  * @brief  Interrupt generator 1 configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register INT1_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_gen_conf_get(lis2dh12_int1_cfg_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT1_CFG, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_int1_cfg_t lis2dh12_int1_gen_conf_get_o(void)
{
  lis2dh12_int1_cfg_t val;
  val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT1_CFG);
  return val;
}

/**
  * @brief  Interrupt generator 1 source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Registers INT1_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_gen_source_get(lis2dh12_int1_src_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT1_SRC, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_int1_src_t lis2dh12_int1_gen_source_get_o(void)
{
 lis2dh12_int1_src_t val;
 val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT1_SRC);
 return val;
}
/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 1.[set]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg INT1_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_gen_threshold_set(uint8_t val)
{
  lis2dh12_int1_ths_t int1_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT1_THS, (uint8_t *)&int1_ths, 1);

  if (ret == 0)
  {
    int1_ths.ths = val;
    ret = lis2dh12_write_reg(LIS2DH12_INT1_THS, (uint8_t *)&int1_ths, 1);
  }

  return ret;
}

void lis2dh12_int1_gen_threshold_set_o(uint8_t val)
{
  lis2dh12_int1_ths_t int1_ths;
  int1_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT1_THS);
  int1_ths.ths = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_INT1_THS, int1_ths.byte);
}


/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 1.[get]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg INT1_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_gen_threshold_get(uint8_t *val)
{
  lis2dh12_int1_ths_t int1_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT1_THS, (uint8_t *)&int1_ths, 1);
  *val = (uint8_t)int1_ths.ths;

  return ret;
}

uint8_t lis2dh12_int1_gen_threshold_get_o(void)
{
  lis2dh12_int1_ths_t int1_ths;
  int1_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT1_THS);
  return (uint8_t)int1_ths.ths;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d in reg INT1_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_gen_duration_set(uint8_t val)
{
  lis2dh12_int1_duration_t int1_duration;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT1_DURATION, (uint8_t *)&int1_duration, 1);

  if (ret == 0)
  {
    int1_duration.d = val;
    ret = lis2dh12_write_reg(LIS2DH12_INT1_DURATION, (uint8_t *)&int1_duration, 1);
  }

  return ret;
}

void lis2dh12_int1_gen_duration_set_o(uint8_t val)
{
  lis2dh12_int1_duration_t int1_duration;
  int1_duration.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT1_DURATION);
  int1_duration.d = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_INT1_DURATION, int1_duration.byte);
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d in reg INT1_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_gen_duration_get(uint8_t *val)
{
  lis2dh12_int1_duration_t int1_duration;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT1_DURATION, (uint8_t *)&int1_duration, 1);
  *val = (uint8_t)int1_duration.d;

  return ret;
}

uint8_t lis2dh12_int1_gen_duration_get_o(void)
{
  lis2dh12_int1_duration_t int1_duration;
  int1_duration.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT1_DURATION);
  return (uint8_t)int1_duration.d;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LIS2DH12_Interrupts_generator_2
  * @brief      This section group all the functions that manage the second
  *             interrupts generator
  * @{
  *
  */

/**
  * @brief  Interrupt generator 2 configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers INT2_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_gen_conf_set(lis2dh12_int2_cfg_t *val)
{
  int8_t ret;

  ret = lis2dh12_write_reg(LIS2DH12_INT2_CFG, (uint8_t *) val, 1);

  return ret;
}

void lis2dh12_int2_gen_conf_set_o(lis2dh12_int2_cfg_t val)
{
 LIS2DH12_Write1ByteRegister(LIS2DH12_INT2_CFG, val.byte);
}
/**
  * @brief  Interrupt generator 2 configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers INT2_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_gen_conf_get(lis2dh12_int2_cfg_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT2_CFG, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_int2_cfg_t lis2dh12_int2_gen_conf_get_o(void)
{
  lis2dh12_int2_cfg_t val;
  val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT2_CFG);
  return val;
}
/**
  * @brief  Interrupt generator 2 source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers INT2_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_gen_source_get(lis2dh12_int2_src_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT2_SRC, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_int2_src_t lis2dh12_int2_gen_source_get_o(void)
{
  lis2dh12_int2_src_t val;
  val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT2_SRC);
  return val;
}
/**
  * @brief   User-defined threshold value for xl interrupt event on
  *          generator 2.[set]
  *          LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg INT2_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_gen_threshold_set(uint8_t val)
{
  lis2dh12_int2_ths_t int2_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT2_THS, (uint8_t *)&int2_ths, 1);

  if (ret == 0)
  {
    int2_ths.ths = val;
    ret = lis2dh12_write_reg(LIS2DH12_INT2_THS, (uint8_t *)&int2_ths, 1);
  }

  return ret;
}

void lis2dh12_int2_gen_threshold_set_o(uint8_t val)
{
  lis2dh12_int2_ths_t int2_ths;
  int2_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT2_THS);
  int2_ths.ths = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_INT2_THS, int2_ths.byte);
}

/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 2.[get]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg INT2_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_gen_threshold_get(uint8_t *val)
{
  lis2dh12_int2_ths_t int2_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT2_THS, (uint8_t *)&int2_ths, 1);
  *val = (uint8_t)int2_ths.ths;

  return ret;
}

uint8_t lis2dh12_int2_gen_threshold_get_o(void)
{
  lis2dh12_int2_ths_t int2_ths;
  int2_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT2_THS);
  return (uint8_t)int2_ths.ths;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized .[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d in reg INT2_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_gen_duration_set(uint8_t val)
{
  lis2dh12_int2_duration_t int2_duration;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT2_DURATION, (uint8_t *)&int2_duration, 1);

  if (ret == 0)
  {
    int2_duration.d = val;
    ret = lis2dh12_write_reg(LIS2DH12_INT2_DURATION, (uint8_t *)&int2_duration, 1);
  }

  return ret;
}

void lis2dh12_int2_gen_duration_set_o(uint8_t val)
{
  lis2dh12_int2_duration_t int2_duration;
  int2_duration.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT2_DURATION);
  int2_duration.d = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_INT2_DURATION, int2_duration.byte);
}
/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d in reg INT2_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_gen_duration_get(uint8_t *val)
{
  lis2dh12_int2_duration_t int2_duration;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_INT2_DURATION, (uint8_t *)&int2_duration, 1);
  *val = (uint8_t)int2_duration.d;

  return ret;
}

uint8_t lis2dh12_int2_gen_duration_get_o(void)
{
  lis2dh12_int2_duration_t int2_duration;
  int2_duration.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_INT2_DURATION);
  return (uint8_t)int2_duration.d;
}
/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DH12_Interrupt_pins
  * @brief     This section group all the functions that manage interrupt pins
  * @{
  *
  */

/**
  * @brief  High-pass filter on interrupts/tap generator.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hp in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_high_pass_int_conf_set(lis2dh12_hp_t val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);

  if (ret == 0)
  {
    ctrl_reg2.hp = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
  }

  return ret;
}

void lis2dh12_high_pass_int_conf_set_o(lis2dh12_hp_t val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  ctrl_reg2.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG2);
  ctrl_reg2.hp = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG2, ctrl_reg2.byte);
}
/**
  * @brief  High-pass filter on interrupts/tap generator.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of hp in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_high_pass_int_conf_get(lis2dh12_hp_t *val)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);

  switch (ctrl_reg2.hp)
  {
    case LIS2DH12_DISC_FROM_INT_GENERATOR:
      *val = LIS2DH12_DISC_FROM_INT_GENERATOR;
      break;

    case LIS2DH12_ON_INT1_GEN:
      *val = LIS2DH12_ON_INT1_GEN;
      break;

    case LIS2DH12_ON_INT2_GEN:
      *val = LIS2DH12_ON_INT2_GEN;
      break;

    case LIS2DH12_ON_TAP_GEN:
      *val = LIS2DH12_ON_TAP_GEN;
      break;

    case LIS2DH12_ON_INT1_INT2_GEN:
      *val = LIS2DH12_ON_INT1_INT2_GEN;
      break;

    case LIS2DH12_ON_INT1_TAP_GEN:
      *val = LIS2DH12_ON_INT1_TAP_GEN;
      break;

    case LIS2DH12_ON_INT2_TAP_GEN:
      *val = LIS2DH12_ON_INT2_TAP_GEN;
      break;

    case LIS2DH12_ON_INT1_INT2_TAP_GEN:
      *val = LIS2DH12_ON_INT1_INT2_TAP_GEN;
      break;

    default:
      *val = LIS2DH12_DISC_FROM_INT_GENERATOR;
      break;
  }

  return ret;
}

lis2dh12_hp_t lis2dh12_high_pass_int_conf_get_o(void)
{
  lis2dh12_ctrl_reg2_t ctrl_reg2;
  ctrl_reg2.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG2);
  switch (ctrl_reg2.hp)
  {
    case LIS2DH12_DISC_FROM_INT_GENERATOR:
      return LIS2DH12_DISC_FROM_INT_GENERATOR;
    case LIS2DH12_ON_INT1_GEN:
      return LIS2DH12_ON_INT1_GEN;
    case LIS2DH12_ON_INT2_GEN:
      return LIS2DH12_ON_INT2_GEN;
    case LIS2DH12_ON_TAP_GEN:
      return LIS2DH12_ON_TAP_GEN;
    case LIS2DH12_ON_INT1_INT2_GEN:
      return LIS2DH12_ON_INT1_INT2_GEN;
    case LIS2DH12_ON_INT1_TAP_GEN:
      return LIS2DH12_ON_INT1_TAP_GEN;
    case LIS2DH12_ON_INT2_TAP_GEN:
      return LIS2DH12_ON_INT2_TAP_GEN;
    case LIS2DH12_ON_INT1_INT2_TAP_GEN:
      return LIS2DH12_ON_INT1_INT2_TAP_GEN;
    default:
      return LIS2DH12_DISC_FROM_INT_GENERATOR;
  }
}
/**
  * @brief  Int1 pin routing configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CTRL_REG3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_pin_int1_config_set(lis2dh12_ctrl_reg3_t *val)
{
  int8_t ret;

  ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG3, (uint8_t *) val, 1);

  return ret;
}

void lis2dh12_pin_int1_config_set_o(lis2dh12_ctrl_reg3_t val)
{
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG3, val.byte);
}

/**
  * @brief  Int1 pin routing configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CTRL_REG3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_pin_int1_config_get(lis2dh12_ctrl_reg3_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG3, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_ctrl_reg3_t lis2dh12_pin_int1_config_get_o(void)
{
  lis2dh12_ctrl_reg3_t val;
  val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG3);
  return val;
}
/**
  * @brief  int2_pin_detect_4d: [set]  4D enable: 4D detection is enabled
  *                                    on INT2 pin when 6D bit on
  *                                    INT2_CFG (34h) is set to 1.
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d4d_int2 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_pin_detect_4d_set(uint8_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0)
  {
    ctrl_reg5.d4d_int2 = val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

void lis2dh12_int2_pin_detect_4d_set_o(uint8_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  ctrl_reg5.d4d_int2 = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG5, ctrl_reg5.byte);
}

/**
  * @brief  4D enable: 4D detection is enabled on INT2 pin when 6D bit on
  *         INT2_CFG (34h) is set to 1.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d4d_int2 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_pin_detect_4d_get(uint8_t *val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  *val = (uint8_t)ctrl_reg5.d4d_int2;

  return ret;
}

uint8_t lis2dh12_int2_pin_detect_4d_get_o(void)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  return (uint8_t)ctrl_reg5.d4d_int2;
}

/**
  * @brief   Latch interrupt request on INT2_SRC (35h) register, with
  *          INT2_SRC (35h) register cleared by reading INT2_SRC(35h)
  *          itself.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lir_int2 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_pin_notification_mode_set(lis2dh12_lir_int2_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0)
  {
    ctrl_reg5.lir_int2 = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

void lis2dh12_int2_pin_notification_mode_set_o(lis2dh12_lir_int2_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  ctrl_reg5.lir_int2 = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG5, ctrl_reg5.byte);
}
/**
  * @brief   Latch interrupt request on INT2_SRC (35h) register, with
  *          INT2_SRC (35h) register cleared by reading INT2_SRC(35h)
  *          itself.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of lir_int2 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int2_pin_notification_mode_get(lis2dh12_lir_int2_t *val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);

  switch (ctrl_reg5.lir_int2)
  {
    case LIS2DH12_INT2_PULSED:
      *val = LIS2DH12_INT2_PULSED;
      break;

    case LIS2DH12_INT2_LATCHED:
      *val = LIS2DH12_INT2_LATCHED;
      break;

    default:
      *val = LIS2DH12_INT2_PULSED;
      break;
  }

  return ret;
}

lis2dh12_lir_int2_t lis2dh12_int2_pin_notification_mode_get_o(void)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  switch (ctrl_reg5.lir_int2)
  {
    case LIS2DH12_INT2_PULSED:
      return LIS2DH12_INT2_PULSED;
    case LIS2DH12_INT2_LATCHED:
      return LIS2DH12_INT2_LATCHED;
    default:
      return LIS2DH12_INT2_PULSED;
  }
}
/**
  * @brief  4D enable: 4D detection is enabled on INT1 pin when 6D bit
  *                    on INT1_CFG(30h) is set to 1.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d4d_int1 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_pin_detect_4d_set(uint8_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0)
  {
    ctrl_reg5.d4d_int1 = val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

void lis2dh12_int1_pin_detect_4d_set_o(uint8_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  ctrl_reg5.d4d_int1 = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG5, ctrl_reg5.byte);
}

/**
  * @brief  4D enable: 4D detection is enabled on INT1 pin when 6D bit on
  *         INT1_CFG(30h) is set to 1.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d4d_int1 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_pin_detect_4d_get(uint8_t *val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  *val = (uint8_t)ctrl_reg5.d4d_int1;

  return ret;
}

uint8_t lis2dh12_int1_pin_detect_4d_get_o(void)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  return (uint8_t)ctrl_reg5.d4d_int1;
}
/**
  * @brief   Latch interrupt request on INT1_SRC (31h), with INT1_SRC(31h)
  *          register cleared by reading INT1_SRC (31h) itself.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lir_int1 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_pin_notification_mode_set(lis2dh12_lir_int1_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0)
  {
    ctrl_reg5.lir_int1 = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

void lis2dh12_int1_pin_notification_mode_set_o(lis2dh12_lir_int1_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  ctrl_reg5.lir_int1 = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG5, ctrl_reg5.byte);
}

/**
  * @brief   Latch interrupt request on INT1_SRC (31h), with INT1_SRC(31h)
  *          register cleared by reading INT1_SRC (31h) itself.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of lir_int1 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_int1_pin_notification_mode_get(lis2dh12_lir_int1_t *val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);

  switch (ctrl_reg5.lir_int1)
  {
    case LIS2DH12_INT1_PULSED:
      *val = LIS2DH12_INT1_PULSED;
      break;

    case LIS2DH12_INT1_LATCHED:
      *val = LIS2DH12_INT1_LATCHED;
      break;

    default:
      *val = LIS2DH12_INT1_PULSED;
      break;
  }

  return ret;
}

lis2dh12_lir_int1_t lis2dh12_int1_pin_notification_mode_get_o(void)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  switch (ctrl_reg5.lir_int1)
  {
    case LIS2DH12_INT1_PULSED:
      return LIS2DH12_INT1_PULSED;
    case LIS2DH12_INT1_LATCHED:
      return LIS2DH12_INT1_LATCHED;
    default:
      return LIS2DH12_INT1_PULSED;
  }
}

/**
  * @brief  Int2 pin routing configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CTRL_REG6
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_pin_int2_config_set(lis2dh12_ctrl_reg6_t *val)
{
  int8_t ret;
  ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG6, (uint8_t *) val, 1);
  return ret;
}

void lis2dh12_pin_int2_config_set_o(lis2dh12_ctrl_reg6_t val)
{
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG6, val.byte);
}

/**
  * @brief  Int2 pin routing configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CTRL_REG6
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_pin_int2_config_get(lis2dh12_ctrl_reg6_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG6, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_ctrl_reg6_t lis2dh12_pin_int2_config_get_o(void)
{
 lis2dh12_ctrl_reg6_t val;
 val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG6);
 return val;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DH12_Fifo
  * @brief     This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief  FIFO enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_en in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_set(uint8_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0)
  {
    ctrl_reg5.fifo_en = val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

void lis2dh12_fifo_set_o(uint8_t val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  ctrl_reg5.fifo_en = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG5, ctrl_reg5.byte);
}

/**
  * @brief  FIFO enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_en in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_get(uint8_t *val)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
  *val = (uint8_t)ctrl_reg5.fifo_en;

  return ret;
}

uint8_t lis2dh12_fifo_get_o(void)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  return (uint8_t)ctrl_reg5.fifo_en;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fth in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_watermark_set(uint8_t val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);

  if (ret == 0)
  {
    fifo_ctrl_reg.fth = val;
    ret = lis2dh12_write_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);
  }

  return ret;
}

void lis2dh12_fifo_watermark_set_o(uint8_t val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  fifo_ctrl_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_CTRL_REG);
  fifo_ctrl_reg.fth = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_FIFO_CTRL_REG, fifo_ctrl_reg.byte);
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fth in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_watermark_get(uint8_t *val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);
  *val = (uint8_t)fifo_ctrl_reg.fth;

  return ret;
}

uint8_t lis2dh12_fifo_watermark_get_o(void)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  fifo_ctrl_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_CTRL_REG);
  return (uint8_t)fifo_ctrl_reg.fth;
}

/**
  * @brief  Trigger FIFO selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tr in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_trigger_event_set(lis2dh12_tr_t val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);

  if (ret == 0)
  {
    fifo_ctrl_reg.tr = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);
  }

  return ret;
}

void lis2dh12_fifo_trigger_event_set_o(lis2dh12_tr_t val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  fifo_ctrl_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_CTRL_REG);
  fifo_ctrl_reg.tr = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_FIFO_CTRL_REG, fifo_ctrl_reg.byte);
}

/**
  * @brief  Trigger FIFO selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of tr in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_trigger_event_get(lis2dh12_tr_t *val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);

  switch (fifo_ctrl_reg.tr)
  {
    case LIS2DH12_INT1_GEN:
      *val = LIS2DH12_INT1_GEN;
      break;

    case LIS2DH12_INT2_GEN:
      *val = LIS2DH12_INT2_GEN;
      break;

    default:
      *val = LIS2DH12_INT1_GEN;
      break;
  }

  return ret;
}

lis2dh12_tr_t lis2dh12_fifo_trigger_event_get_o(void)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  fifo_ctrl_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_CTRL_REG);
  switch (fifo_ctrl_reg.tr)
  {
    case LIS2DH12_INT1_GEN:
      return LIS2DH12_INT1_GEN;
    case LIS2DH12_INT2_GEN:
      return LIS2DH12_INT2_GEN;
    default:
      return LIS2DH12_INT1_GEN;
  }
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fm in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_mode_set(lis2dh12_fm_t val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);

  if (ret == 0)
  {
    fifo_ctrl_reg.fm = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);
  }

  return ret;
}

void lis2dh12_fifo_mode_set_o(lis2dh12_fm_t val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  fifo_ctrl_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_CTRL_REG);
  fifo_ctrl_reg.fm = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_FIFO_CTRL_REG, fifo_ctrl_reg.byte);
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fm in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_mode_get(lis2dh12_fm_t *val)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg, 1);

  switch (fifo_ctrl_reg.fm)
  {
    case LIS2DH12_BYPASS_MODE:
      *val = LIS2DH12_BYPASS_MODE;
      break;

    case LIS2DH12_FIFO_MODE:
      *val = LIS2DH12_FIFO_MODE;
      break;

    case LIS2DH12_DYNAMIC_STREAM_MODE:
      *val = LIS2DH12_DYNAMIC_STREAM_MODE;
      break;

    case LIS2DH12_STREAM_TO_FIFO_MODE:
      *val = LIS2DH12_STREAM_TO_FIFO_MODE;
      break;

    default:
      *val = LIS2DH12_BYPASS_MODE;
      break;
  }

  return ret;
}

lis2dh12_fm_t lis2dh12_fifo_mode_get_o(void)
{
  lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
  fifo_ctrl_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_CTRL_REG);
  switch (fifo_ctrl_reg.fm)
   {
     case LIS2DH12_BYPASS_MODE:
       return LIS2DH12_BYPASS_MODE;
     case LIS2DH12_FIFO_MODE:
       return LIS2DH12_FIFO_MODE;
     case LIS2DH12_DYNAMIC_STREAM_MODE:
       return LIS2DH12_DYNAMIC_STREAM_MODE;
     case LIS2DH12_STREAM_TO_FIFO_MODE:
       return LIS2DH12_STREAM_TO_FIFO_MODE;
     default:
       return LIS2DH12_BYPASS_MODE;
   }
}

/**
  * @brief  FIFO status register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_status_get(lis2dh12_fifo_src_reg_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_SRC_REG, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_fifo_src_reg_t lis2dh12_fifo_status_get_o(void)
{
  lis2dh12_fifo_src_reg_t val;
  val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_SRC_REG);
  return val;
}
/**
  * @brief  FIFO stored data level.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fss in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_data_level_get(uint8_t *val)
{
  lis2dh12_fifo_src_reg_t fifo_src_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg, 1);
  *val = (uint8_t)fifo_src_reg.fss;

  return ret;
}
uint8_t lis2dh12_fifo_data_level_get_o(void)
{
  lis2dh12_fifo_src_reg_t fifo_src_reg;
  fifo_src_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_SRC_REG);
  return (uint8_t)fifo_src_reg.fss;
}
/**
  * @brief  Empty FIFO status flag.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of empty in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_empty_flag_get(uint8_t *val)
{
  lis2dh12_fifo_src_reg_t fifo_src_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg, 1);
  *val = (uint8_t)fifo_src_reg.empty;

  return ret;
}

uint8_t lis2dh12_fifo_empty_flag_get_o(void)
{
  lis2dh12_fifo_src_reg_t fifo_src_reg;
  fifo_src_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_SRC_REG);
  return (uint8_t)fifo_src_reg.empty;
}

/**
  * @brief  FIFO overrun status flag.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ovrn_fifo in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_ovr_flag_get(uint8_t *val)
{
  lis2dh12_fifo_src_reg_t fifo_src_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg, 1);
  *val = (uint8_t)fifo_src_reg.ovrn_fifo;

  return ret;
}

uint8_t lis2dh12_fifo_ovr_flag_get_o(void)
{
  lis2dh12_fifo_src_reg_t fifo_src_reg;
  fifo_src_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_SRC_REG);
  return (uint8_t)fifo_src_reg.ovrn_fifo;
}

/**
  * @brief  FIFO watermark status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wtm in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_fifo_fth_flag_get(uint8_t *val)
{
  lis2dh12_fifo_src_reg_t fifo_src_reg;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg, 1);
  *val = (uint8_t)fifo_src_reg.wtm;

  return ret;
}

uint8_t lis2dh12_fifo_fth_flag_get_o(void)
{
  lis2dh12_fifo_src_reg_t fifo_src_reg;
  fifo_src_reg.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_FIFO_SRC_REG);
  return (uint8_t)fifo_src_reg.wtm;
}
/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DH12_Tap_generator
  * @brief     This section group all the functions that manage the tap and
  *            double tap event generation
  * @{
  *
  */

/**
  * @brief  Tap/Double Tap generator configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CLICK_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_tap_conf_set(lis2dh12_click_cfg_t *val)
{
  int8_t ret;

  ret = lis2dh12_write_reg(LIS2DH12_CLICK_CFG, (uint8_t *) val, 1);

  return ret;
}

void lis2dh12_tap_conf_set_o(lis2dh12_click_cfg_t val)
{
  LIS2DH12_Write1ByteRegister(LIS2DH12_CLICK_CFG, val.byte);
}

/**
  * @brief  Tap/Double Tap generator configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CLICK_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_tap_conf_get(lis2dh12_click_cfg_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CLICK_CFG, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_click_cfg_t lis2dh12_tap_conf_get_o(void)
{
  lis2dh12_click_cfg_t val;
  val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CLICK_CFG);
  return val;
}

/**
  * @brief  Tap/Double Tap generator source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CLICK_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_tap_source_get(lis2dh12_click_src_t *val)
{
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CLICK_SRC, (uint8_t *) val, 1);

  return ret;
}

lis2dh12_click_src_t lis2dh12_tap_source_get_o(void)
{
  lis2dh12_click_src_t val;
  val.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CLICK_SRC);
  return val;
}

/**
  * @brief  User-defined threshold value for Tap/Double Tap event.[set]
  *         1 LSB = full scale/128
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_tap_threshold_set(uint8_t val)
{
  lis2dh12_click_ths_t click_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CLICK_THS, (uint8_t *)&click_ths, 1);

  if (ret == 0)
  {
    click_ths.ths = val;
    ret = lis2dh12_write_reg(LIS2DH12_CLICK_THS, (uint8_t *)&click_ths, 1);
  }

  return ret;
}

void lis2dh12_tap_threshold_set_o(uint8_t val)
{
  lis2dh12_click_ths_t click_ths;
  click_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CLICK_THS);
  click_ths.ths = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CLICK_THS, click_ths.byte);
}

/**
  * @brief  User-defined threshold value for Tap/Double Tap event.[get]
  *         1 LSB = full scale/128
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_tap_threshold_get(uint8_t *val)
{
  lis2dh12_click_ths_t click_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CLICK_THS, (uint8_t *)&click_ths, 1);
  *val = (uint8_t)click_ths.ths;

  return ret;
}

uint8_t lis2dh12_tap_threshold_get_o(void)
{
  lis2dh12_click_ths_t click_ths;
  click_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CLICK_THS);
  return (uint8_t)click_ths.ths;
}

/**
  * @brief   If the LIR_Click bit is not set, the interrupt is kept high
  *          for the duration of the latency window.
  *          If the LIR_Click bit is set, the interrupt is kept high until the
  *          CLICK_SRC(39h) register is read.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lir_click in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_tap_notification_mode_set(lis2dh12_lir_click_t val)
{
  lis2dh12_click_ths_t click_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CLICK_THS, (uint8_t *)&click_ths, 1);

  if (ret == 0)
  {
    click_ths.lir_click = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CLICK_THS, (uint8_t *)&click_ths, 1);
  }

  return ret;
}

void lis2dh12_tap_notification_mode_set_o(lis2dh12_lir_click_t val)
{
  lis2dh12_click_ths_t click_ths;
  click_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CLICK_THS);
  click_ths.lir_click = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CLICK_THS, click_ths.byte);
}

/**
  * @brief   If the LIR_Click bit is not set, the interrupt is kept high
  *          for the duration of the latency window.
  *          If the LIR_Click bit is set, the interrupt is kept high until the
  *          CLICK_SRC(39h) register is read.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of lir_click in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */

int8_t lis2dh12_tap_notification_mode_get(lis2dh12_lir_click_t *val)
{
  lis2dh12_click_ths_t click_ths;
  int8_t ret;
   
  ret = lis2dh12_read_reg(LIS2DH12_CLICK_THS, (uint8_t *)&click_ths, 1);

  switch (click_ths.lir_click)
  {
    case LIS2DH12_TAP_PULSED:
      *val = LIS2DH12_TAP_PULSED;
      break;

    case LIS2DH12_TAP_LATCHED:
      *val = LIS2DH12_TAP_LATCHED;
      break;

    default:
      *val = LIS2DH12_TAP_PULSED;
      break;
  }

  return ret;
}

lis2dh12_lir_click_t lis2dh12_tap_notification_mode_get_o(void)
{
  lis2dh12_click_ths_t click_ths;
  click_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CLICK_THS);
  switch (click_ths.lir_click)
  {
    case LIS2DH12_TAP_PULSED:
      return LIS2DH12_TAP_PULSED;
    case LIS2DH12_TAP_LATCHED:
      return LIS2DH12_TAP_LATCHED;
    default:
      return LIS2DH12_TAP_PULSED;
  }
}

/**
  * @brief  The maximum time (1 LSB = 1/ODR) interval that can elapse
  *         between the start of the click-detection procedure and when the
  *         acceleration falls back below the threshold.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tli in reg TIME_LIMIT
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_shock_dur_set(uint8_t val)
{
  lis2dh12_time_limit_t time_limit;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_TIME_LIMIT, (uint8_t *)&time_limit, 1);

  if (ret == 0)
  {
    time_limit.tli = val;
    ret = lis2dh12_write_reg(LIS2DH12_TIME_LIMIT, (uint8_t *)&time_limit, 1);
  }

  return ret;
}

void lis2dh12_shock_dur_set_o(uint8_t val)
{
  lis2dh12_time_limit_t time_limit;
  time_limit.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_TIME_LIMIT);
  time_limit.tli = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_TIME_LIMIT, time_limit.byte);
}

/**
  * @brief  The maximum time (1 LSB = 1/ODR) interval that can elapse between
  *         the start of the click-detection procedure and when the
  *         acceleration falls back below the threshold.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tli in reg TIME_LIMIT
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_shock_dur_get(uint8_t *val)
{
  lis2dh12_time_limit_t time_limit;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_TIME_LIMIT, (uint8_t *)&time_limit, 1);
  *val = (uint8_t)time_limit.tli;

  return ret;
}

uint8_t lis2dh12_shock_dur_get_o(void)
{
  lis2dh12_time_limit_t time_limit;
  time_limit.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_TIME_LIMIT);
  return (uint8_t)time_limit.tli;
}

/**
  * @brief  The time (1 LSB = 1/ODR) interval that starts after the first
  *         click detection where the click-detection procedure is
  *         disabled, in cases where the device is configured for
  *         double-click detection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tla in reg TIME_LATENCY
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_quiet_dur_set(uint8_t val)
{
  lis2dh12_time_latency_t time_latency;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_TIME_LATENCY, (uint8_t *)&time_latency, 1);

  if (ret == 0)
  {
    time_latency.tla = val;
    ret = lis2dh12_write_reg(LIS2DH12_TIME_LATENCY, (uint8_t *)&time_latency, 1);
  }

  return ret;
}

void lis2dh12_quiet_dur_set_o(uint8_t val)
{
  LIS2DH12_Write1ByteRegister(LIS2DH12_TIME_LATENCY, val);
}

/**
  * @brief  The time (1 LSB = 1/ODR) interval that starts after the first
  *         click detection where the click-detection procedure is
  *         disabled, in cases where the device is configured for
  *         double-click detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tla in reg TIME_LATENCY
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_quiet_dur_get(uint8_t *val)
{
  lis2dh12_time_latency_t time_latency;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_TIME_LATENCY, (uint8_t *)&time_latency, 1);
  *val = (uint8_t)time_latency.tla;

  return ret;
}

uint8_t lis2dh12_quiet_dur_get_o(void)
{
  return LIS2DH12_Read1ByteRegister(LIS2DH12_TIME_LATENCY);
}

/**
  * @brief  The maximum interval of time (1 LSB = 1/ODR) that can elapse
  *         after the end of the latency interval in which the click-detection
  *         procedure can start, in cases where the device is configured
  *         for double-click detection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tw in reg TIME_WINDOW
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_double_tap_timeout_set(uint8_t val)
{
  lis2dh12_time_window_t time_window;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_TIME_WINDOW, (uint8_t *)&time_window, 1);

  if (ret == 0)
  {
    time_window.tw = val;
    ret = lis2dh12_write_reg(LIS2DH12_TIME_WINDOW, (uint8_t *)&time_window, 1);
  }

  return ret;
}

void lis2dh12_double_tap_timeout_set_o(uint8_t val)
{
 LIS2DH12_Write1ByteRegister(LIS2DH12_TIME_WINDOW, val);
}

/**
  * @brief  The maximum interval of time (1 LSB = 1/ODR) that can elapse
  *         after the end of the latency interval in which the
  *         click-detection procedure can start, in cases where the device
  *         is configured for double-click detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tw in reg TIME_WINDOW
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_double_tap_timeout_get(uint8_t *val)
{
  lis2dh12_time_window_t time_window;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_TIME_WINDOW, (uint8_t *)&time_window, 1);
  *val = (uint8_t)time_window.tw;

  return ret;
}

uint8_t lis2dh12_double_tap_timeout_get_o(void)
{
 return LIS2DH12_Read1ByteRegister(LIS2DH12_TIME_WINDOW);
}

/**
  * @defgroup  LIS2DH12_Activity_inactivity
  * @brief     This section group all the functions concerning activity
  *            inactivity functionality
  * @{
  *
  */

/**
  * @brief    Sleep-to-wake, return-to-sleep activation threshold in
  *           low-power mode.[set]
  *           1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of acth in reg ACT_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_act_threshold_set(uint8_t val)
{
  lis2dh12_act_ths_t act_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_ACT_THS, (uint8_t *)&act_ths, 1);

  if (ret == 0)
  {
    act_ths.acth = val;
    ret = lis2dh12_write_reg(LIS2DH12_ACT_THS, (uint8_t *)&act_ths, 1);
  }

  return ret;
}

void lis2dh12_act_threshold_set_o(uint8_t val)
{
  lis2dh12_act_ths_t act_ths;
  act_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_ACT_THS);
  act_ths.acth = val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_ACT_THS, act_ths.byte);
}

/**
  * @brief  Sleep-to-wake, return-to-sleep activation threshold in low-power
  *         mode.[get]
  *         1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of acth in reg ACT_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_act_threshold_get(uint8_t *val)
{
  lis2dh12_act_ths_t act_ths;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_ACT_THS, (uint8_t *)&act_ths, 1);
  *val = (uint8_t)act_ths.acth;

  return ret;
}

uint8_t lis2dh12_act_threshold_get_o(void)
{
  lis2dh12_act_ths_t act_ths;
  act_ths.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_ACT_THS);
  return (uint8_t)act_ths.acth;
}

/**
  * @brief  Sleep-to-wake, return-to-sleep.[set]
  *         duration = (8*1[LSb]+1)/ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of actd in reg ACT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_act_timeout_set(uint8_t val)
{
  lis2dh12_act_dur_t act_dur;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_ACT_DUR, (uint8_t *)&act_dur, 1);

  if (ret == 0)
  {
    act_dur.actd = val;
    ret = lis2dh12_write_reg(LIS2DH12_ACT_DUR, (uint8_t *)&act_dur, 1);
  }

  return ret;
}

void lis2dh12_act_timeout_set_o(uint8_t val)
{
  LIS2DH12_Write1ByteRegister(LIS2DH12_ACT_DUR, val);
}

/**
  * @brief  Sleep-to-wake, return-to-sleep.[get]
  *         duration = (8*1[LSb]+1)/ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of actd in reg ACT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_act_timeout_get(uint8_t *val)
{
  lis2dh12_act_dur_t act_dur;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_ACT_DUR, (uint8_t *)&act_dur, 1);
  *val = (uint8_t)act_dur.actd;

  return ret;
}

uint8_t lis2dh12_act_timeout_get_o(void)
{
 return LIS2DH12_Read1ByteRegister(LIS2DH12_ACT_DUR);
}

/**
  * @defgroup  LIS2DH12_Serial_interface
  * @brief     This section group all the functions concerning serial
  *            interface management
  * @{
  *
  */

/**
  * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sdo_pu_disc in reg CTRL_REG0
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_pin_sdo_sa0_mode_set(lis2dh12_sdo_pu_disc_t val)
{
  lis2dh12_ctrl_reg0_t ctrl_reg0;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG0, (uint8_t *)&ctrl_reg0, 1);

  if (ret == 0)
  {
    ctrl_reg0.sdo_pu_disc = (uint8_t)val;
    ret = lis2dh12_write_reg(LIS2DH12_CTRL_REG0, (uint8_t *)&ctrl_reg0, 1);
  }

  return ret;
}

void lis2dh12_pin_sdo_sa0_mode_set_o(lis2dh12_sdo_pu_disc_t val)
{
  lis2dh12_ctrl_reg0_t ctrl_reg0;
  ctrl_reg0.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG0);
  ctrl_reg0.sdo_pu_disc = (uint8_t)val;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG0, ctrl_reg0.byte);
}

/**
  * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of sdo_pu_disc in reg CTRL_REG0
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int8_t lis2dh12_pin_sdo_sa0_mode_get(lis2dh12_sdo_pu_disc_t *val)
{
  lis2dh12_ctrl_reg0_t ctrl_reg0;
  int8_t ret;

  ret = lis2dh12_read_reg(LIS2DH12_CTRL_REG0, (uint8_t *)&ctrl_reg0, 1);

  switch (ctrl_reg0.sdo_pu_disc)
  {
    case LIS2DH12_PULL_UP_DISCONNECT:
      *val = LIS2DH12_PULL_UP_DISCONNECT;
      break;

    case LIS2DH12_PULL_UP_CONNECT:
      *val = LIS2DH12_PULL_UP_CONNECT;
      break;

    default:
      *val = LIS2DH12_PULL_UP_DISCONNECT;
      break;
  }

  return ret;
}

lis2dh12_sdo_pu_disc_t lis2dh12_pin_sdo_sa0_mode_get_o(void)
{
  lis2dh12_ctrl_reg0_t ctrl_reg0;
  ctrl_reg0.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG0);
  switch (ctrl_reg0.sdo_pu_disc)
  {
    case LIS2DH12_PULL_UP_DISCONNECT:
      return LIS2DH12_PULL_UP_DISCONNECT;
    case LIS2DH12_PULL_UP_CONNECT:
      return LIS2DH12_PULL_UP_CONNECT;
    default:
      return LIS2DH12_PULL_UP_DISCONNECT;
  }
}

/**
 * Convert raw value to acceleration in mg. Reads scale and resolution from state variables.
 *
 * @param raw_acceleration raw ADC value from LIS2DH12
 * @return int16_t representing acceleration in milli-G. 
 */
#ifdef LIS2DH12_FLOAT
float_t lis2dh12_rawToMg(int16_t raw_acceleration)
{
  switch(state_scale)
  {
    case LIS2DH12_2g:
      switch(state_resolution)
      {
        case LIS2DH12_LP_8bit:
          return ((float_t)raw_acceleration / 256.0f) * 16.0f;

        case LIS2DH12_NM_10bit:
          return ((float_t)raw_acceleration / 64.0f) *  4.0f;

        case LIS2DH12_HR_12bit:
          return ((float_t)raw_acceleration / 16.0f) * 1.0f;

        default:
          break;
      }
      break;

    case LIS2DH12_4g:
      switch(state_resolution)
      {
        case LIS2DH12_LP_8bit:
          return ((float_t)raw_acceleration / 256.0f) * 32.0f;

        case LIS2DH12_NM_10bit:
          return ((float_t)raw_acceleration / 64.0f) *  8.0f;

        case LIS2DH12_HR_12bit:
          return ((float_t)raw_acceleration / 16.0f) *  2.0f;
      }
      break;

    case LIS2DH12_8g:
      switch(state_resolution)
      {
        case LIS2DH12_LP_8bit:
          return ((float_t)raw_acceleration / 256.0f) * 64.0f;

        case LIS2DH12_NM_10bit:
          return ((float_t)raw_acceleration / 64.0f) * 16.0f;

        case LIS2DH12_HR_12bit:
          return ((float_t)raw_acceleration / 16.0f) * 4.0f;

        default:
          break;
      }
    break;

    case LIS2DH12_16g:
      switch(state_resolution)
      {
        case LIS2DH12_LP_8bit:
          return ((float_t)raw_acceleration / 256.0f) * 192.0f;

        case LIS2DH12_NM_10bit:
          return ((float_t)raw_acceleration / 64.0f) * 48.0f;

        case LIS2DH12_HR_12bit:
          return ((float_t)raw_acceleration / 16.0f) * 12.0f;

        default:
          break;
      }
    break;

    default:
      break;
  }
  // reached only in case of an error, return "smallest representable value"
  return (int16_t)0x8000;
}
#else
int16_t lis2dh12_rawToMg(int16_t raw_acceleration)
{
  switch(state_scale)
  {
    case LIS2DH12_2g:
      switch(state_resolution)
      {
        case LIS2DH12_LP_8bit:
          return (raw_acceleration / 256) * 16;

        case LIS2DH12_NM_10bit:
          return (raw_acceleration / 64) * 4;

        case LIS2DH12_HR_12bit:
          return (raw_acceleration / 16 ) * 1;

        default:
          break;
      }
      break;

    case LIS2DH12_4g:
      switch(state_resolution)
      {
        case LIS2DH12_LP_8bit:
          return (raw_acceleration / 256) * 32;

        case LIS2DH12_NM_10bit:
          return (raw_acceleration / 64) * 8;

        case LIS2DH12_HR_12bit:
          return (raw_acceleration / 16 ) * 2;
      }
      break;

    case LIS2DH12_8g:
      switch(state_resolution)
      {
        case LIS2DH12_LP_8bit:
          return (raw_acceleration / 256) * 64;

        case LIS2DH12_NM_10bit:
          return (raw_acceleration / 64) * 16;

        case LIS2DH12_HR_12bit:
          return (raw_acceleration / 16 ) * 4;

        default:
          break;
      }
    break;

    case LIS2DH12_16g:
      switch(state_resolution)
      {
        case LIS2DH12_LP_8bit:
          return (raw_acceleration / 256) * 192;

        case LIS2DH12_NM_10bit:
          return (raw_acceleration / 64) * 48;

        case LIS2DH12_HR_12bit:
          return (raw_acceleration / 16) * 12;
        default:
          break;
      }
    break;

    default:
      break;
  }
  // reached only in case of an error, return "smallest representable value"
  return (int16_t)0x8000;
}
#endif

int16_t lis2dh12_rawToC10(int16_t data_raw_temperature)
{
 switch (state_resolution)
  {
   case LIS2DH12_HR_12bit: //High resolution
   case LIS2DH12_NM_10bit: //Normal mode
   case LIS2DH12_LP_8bit: //Low power mode
    return (int16_t)(((10*(int32_t)data_raw_temperature) / 256) + 200);
  }
}


/**
 * Return correct threshold setting for activity interrupt at
 * given threshold. Scales threshold upwards to next value.
 *
 * @param threshold_mg desired threshold. Will be converted to positive value if negative value is given.
 * @return threshold to set or 0x7F (max) if given threshold is not possible.
 */
uint8_t lis2dh12_scale_interrupt_threshold(uint16_t threshold_mg)
{
  // Adjust for scale
  // 1 LSb = 16 mg @ FS = 2 g
  // 1 LSb = 32 mg @ FS = 4 g
  // 1 LSb = 62 mg @ FS = 8 g
  // 1 LSb = 186 mg @ FS = 16 g
 
 uint8_t divisor;
  switch(state_scale)
  {
    case LIS2DH12_2g:
      divisor = 16;
      break;

    case LIS2DH12_4g:
      divisor = 32;
      break;

    case LIS2DH12_8g:
      divisor = 62;
      break;

    case LIS2DH12_16g:
      divisor = 186;
      break;

    default:
      divisor = 16;
      break;
  }

  //if (threshold_mg < 0) threshold_mg = 0 - threshold_mg; 
  uint8_t threshold = (uint8_t)(threshold_mg/divisor) + 1;
  if (threshold > 0x7F) threshold = 0x7F;
  return threshold;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
