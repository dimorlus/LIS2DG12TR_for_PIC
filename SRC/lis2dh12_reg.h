/**
 * Based on ST lis2dh12 driver. 
 * Modified and optimized for PIC (and other 8-bit MCUs). 
 * Use functions with the "_o" suffix, the original ones are left for 
 * compatibility.
 * Disable (disconnect) the built-in SA0 pull-up if this pin is connected 
 * to ground to reduce consumption by approximately 120 µA.
 * Checking the temperature sensor showed its practical uselessness. 
 * The results are extremely unstable, their conversion to degrees 
 * is not described, the code for converting them to degrees in the ST source 
 * code looks strange, the result of its work strongly does not correspond 
 * to the real temperature. Turning on the temperature sensor adds 15..20 µA 
 * to the consumption.
  ******************************************************************************
  * @file    lis2dh12_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lis2dh12_reg.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIS2DH12_REGS_H
#define LIS2DH12_REGS_H

#undef LIS2DH12_FLOAT
//#define LIS2DH12_FLOAT

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#define LIS2DH12_I2C_ADD 0x18

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

 
typedef union
{
 struct
 {
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
 };
  uint8_t byte;
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

#endif /* MEMS_SHARED_TYPES */

/** Device Identification (Who am I) **/
#define LIS2DH12_ID          0x33U

#define LIS2DH12_STATUS_REG_AUX        0x07U
typedef union
{
 struct
  {
   uint8_t not_used_01       : 2;
   uint8_t tda               : 1;
   uint8_t not_used_02       : 3;
   uint8_t tor               : 1;
   uint8_t not_used_03       : 1;
  };
 uint8_t byte; 
} lis2dh12_status_reg_aux_t;

#define LIS2DH12_OUT_TEMP_L            0x0CU
#define LIS2DH12_OUT_TEMP_H            0x0DU
#define LIS2DH12_WHO_AM_I              0x0FU

#define LIS2DH12_CTRL_REG0             0x1EU
typedef union
{
 struct
  {
   uint8_t not_used_01       : 7;
   uint8_t sdo_pu_disc       : 1;
  }; 
 uint8_t byte; 
}lis2dh12_ctrl_reg0_t;

#define LIS2DH12_TEMP_CFG_REG          0x1FU
typedef union
{
 struct
  {
   uint8_t not_used_01       : 6;
   uint8_t temp_en           : 2;
  };
 uint8_t byte; 
} lis2dh12_temp_cfg_reg_t;

#define LIS2DH12_CTRL_REG1             0x20U
typedef union
{
 struct
  {
   uint8_t xen               : 1;
   uint8_t yen               : 1;
   uint8_t zen               : 1;
   uint8_t lpen              : 1;
   uint8_t odr               : 4;
  };  
 uint8_t byte; 
} lis2dh12_ctrl_reg1_t;

#define LIS2DH12_CTRL_REG2             0x21U
typedef union
{
 struct
  {
   uint8_t hp                : 3; /* HPCLICK + HP_IA2 + HP_IA1 -> HP */
   uint8_t fds               : 1;
   uint8_t hpcf              : 2;
   uint8_t hpm               : 2;
  }; 
 uint8_t byte; 
} lis2dh12_ctrl_reg2_t;

#define LIS2DH12_CTRL_REG3             0x22U
typedef union
{
 struct
  {
   uint8_t not_used_01       : 1;
   uint8_t i1_overrun        : 1;
   uint8_t i1_wtm            : 1;
   uint8_t not_used_02       : 1;
   uint8_t i1_zyxda          : 1;
   uint8_t i1_ia2            : 1;
   uint8_t i1_ia1            : 1;
   uint8_t i1_click          : 1;
  };  
 uint8_t byte; 
} lis2dh12_ctrl_reg3_t;

#define LIS2DH12_CTRL_REG4             0x23U
typedef union
{        
 struct
  {
   uint8_t sim               : 1;
   uint8_t st                : 2;
   uint8_t hr                : 1;
   uint8_t fs                : 2;
   uint8_t ble               : 1;
   uint8_t bdu               : 1;
  }; 
 uint8_t byte; 
} lis2dh12_ctrl_reg4_t;

#define LIS2DH12_CTRL_REG5             0x24U
typedef union
{
 struct
  {
   uint8_t d4d_int2          : 1;
   uint8_t lir_int2          : 1;
   uint8_t d4d_int1          : 1;
   uint8_t lir_int1          : 1;
   uint8_t not_used_01       : 2;
   uint8_t fifo_en           : 1;
   uint8_t boot              : 1;
  };  
 uint8_t byte; 
} lis2dh12_ctrl_reg5_t;

#define LIS2DH12_CTRL_REG6            0x25U
typedef union
{
 struct
  {
   uint8_t not_used_01       : 1;
   uint8_t int_polarity      : 1;
   uint8_t not_used_02       : 1;
   uint8_t i2_act            : 1;
   uint8_t i2_boot           : 1;
   uint8_t i2_ia2            : 1;
   uint8_t i2_ia1            : 1;
   uint8_t i2_click          : 1;
  };  
 uint8_t byte; 
} lis2dh12_ctrl_reg6_t;

#define LIS2DH12_REFERENCE            0x26U
#define LIS2DH12_STATUS_REG           0x27U
typedef union
{
 struct
  {
   uint8_t xda               : 1;
   uint8_t yda               : 1;
   uint8_t zda               : 1;
   uint8_t zyxda             : 1;
   uint8_t _xor              : 1;
   uint8_t yor               : 1;
   uint8_t zor               : 1;
   uint8_t zyxor             : 1;
  };  
 uint8_t byte; 
} lis2dh12_status_reg_t;

#define LIS2DH12_OUT_X_L              0x28U
#define LIS2DH12_OUT_X_H              0x29U
#define LIS2DH12_OUT_Y_L              0x2AU
#define LIS2DH12_OUT_Y_H              0x2BU
#define LIS2DH12_OUT_Z_L              0x2CU
#define LIS2DH12_OUT_Z_H              0x2DU
#define LIS2DH12_FIFO_CTRL_REG        0x2EU
typedef union
{
 struct
  {
   uint8_t fth               : 5;
   uint8_t tr                : 1;
   uint8_t fm                : 2;
  };  
 uint8_t byte; 
} lis2dh12_fifo_ctrl_reg_t;

#define LIS2DH12_FIFO_SRC_REG         0x2FU
typedef union
{
 struct
  {
   uint8_t fss               : 5;
   uint8_t empty             : 1;
   uint8_t ovrn_fifo         : 1;
   uint8_t wtm               : 1;
  };  
 uint8_t byte; 
} lis2dh12_fifo_src_reg_t;

#define LIS2DH12_INT1_CFG             0x30U
typedef union
{
 struct
  {
   uint8_t xlie              : 1;
   uint8_t xhie              : 1;
   uint8_t ylie              : 1;
   uint8_t yhie              : 1;
   uint8_t zlie              : 1;
   uint8_t zhie              : 1;
   uint8_t _6d               : 1;
   uint8_t aoi               : 1;
  };  
 uint8_t byte; 
} lis2dh12_int1_cfg_t;

#define LIS2DH12_INT1_SRC             0x31U
typedef union
{
 struct
  {
   uint8_t xl                : 1;
   uint8_t xh                : 1;
   uint8_t yl                : 1;
   uint8_t yh                : 1;
   uint8_t zl                : 1;
   uint8_t zh                : 1;
   uint8_t ia                : 1;
   uint8_t not_used_01       : 1;
  };
 uint8_t byte; 
} lis2dh12_int1_src_t;

#define LIS2DH12_INT1_THS             0x32U
typedef union
{
 struct
  {
   uint8_t ths               : 7;
   uint8_t not_used_01       : 1;
  };  
 uint8_t byte; 
} lis2dh12_int1_ths_t;

#define LIS2DH12_INT1_DURATION        0x33U
typedef union
{
 struct
  {
   uint8_t d                 : 7;
   uint8_t not_used_01       : 1;
  };  
 uint8_t byte; 
} lis2dh12_int1_duration_t;

#define LIS2DH12_INT2_CFG             0x34U
typedef union
{
 struct
  {
   uint8_t xlie              : 1;
   uint8_t xhie              : 1;
   uint8_t ylie              : 1;
   uint8_t yhie              : 1;
   uint8_t zlie              : 1;
   uint8_t zhie              : 1;
   uint8_t _6d               : 1;
   uint8_t aoi               : 1;
  };  
 uint8_t byte; 
} lis2dh12_int2_cfg_t;

#define LIS2DH12_INT2_SRC             0x35U
typedef union
{
 struct
  {
   uint8_t xl                : 1;
   uint8_t xh                : 1;
   uint8_t yl                : 1;
   uint8_t yh                : 1;
   uint8_t zl                : 1;
   uint8_t zh                : 1;
   uint8_t ia                : 1;
   uint8_t not_used_01       : 1;
  };  
 uint8_t byte; 
} lis2dh12_int2_src_t;

#define LIS2DH12_INT2_THS             0x36U
typedef union
{
 struct
  {
   uint8_t ths               : 7;
   uint8_t not_used_01       : 1;
  };  
 uint8_t byte; 
} lis2dh12_int2_ths_t;

#define LIS2DH12_INT2_DURATION        0x37U
typedef union
{
 struct
  {
   uint8_t d                 : 7;
   uint8_t not_used_01       : 1;
  };
 uint8_t byte; 
} lis2dh12_int2_duration_t;

#define LIS2DH12_CLICK_CFG            0x38U
typedef union
{
 struct
  {
   uint8_t xs                : 1;
   uint8_t xd                : 1;
   uint8_t ys                : 1;
   uint8_t yd                : 1;
   uint8_t zs                : 1;
   uint8_t zd                : 1;
   uint8_t not_used_01       : 2;
  };
 uint8_t byte; 
} lis2dh12_click_cfg_t;

#define LIS2DH12_CLICK_SRC            0x39U
typedef union
{
 struct
  {
   uint8_t x                 : 1;
   uint8_t y                 : 1;
   uint8_t z                 : 1;
   uint8_t sign              : 1;
   uint8_t sclick            : 1;
   uint8_t dclick            : 1;
   uint8_t ia                : 1;
   uint8_t not_used_01       : 1;
  };
 uint8_t byte; 
} lis2dh12_click_src_t;

#define LIS2DH12_CLICK_THS            0x3AU
typedef union
{
 struct
  {
   uint8_t ths               : 7;
   uint8_t lir_click         : 1;
  };
 uint8_t byte; 
} lis2dh12_click_ths_t;

#define LIS2DH12_TIME_LIMIT           0x3BU
typedef union
{
 struct
  {
   uint8_t tli               : 7;
   uint8_t not_used_01       : 1;
  };
 uint8_t byte; 
} lis2dh12_time_limit_t;

#define LIS2DH12_TIME_LATENCY         0x3CU
typedef union
{
 struct
 {
  uint8_t tla               : 8;
 };
 uint8_t byte; 
} lis2dh12_time_latency_t;

#define LIS2DH12_TIME_WINDOW          0x3DU
typedef union
{
 struct
 {
  uint8_t tw                : 8;
 };
 uint8_t byte; 
} lis2dh12_time_window_t;

#define LIS2DH12_ACT_THS              0x3EU
typedef union
{
 struct
 {
  uint8_t acth              : 7;
  uint8_t not_used_01       : 1;
 };
 uint8_t byte; 
} lis2dh12_act_ths_t;

#define LIS2DH12_ACT_DUR              0x3FU
typedef union
{
 struct
 {
  uint8_t actd              : 8;
 };
 uint8_t byte; 
} lis2dh12_act_dur_t;


int8_t lis2dh12_read_reg(uint8_t reg, uint8_t *data, uint8_t len);
int8_t lis2dh12_write_reg(uint8_t reg, uint8_t *data, uint8_t len);
uint8_t LIS2DH12_Read1ByteRegister(uint8_t reg);
void LIS2DH12_Write1ByteRegister(uint8_t reg, uint8_t data);

int8_t lis2dh12_temp_status_reg_get(uint8_t *buff);
uint8_t lis2dh12_temp_status_reg_get_o(void);
int8_t lis2dh12_temp_data_ready_get(uint8_t *val);
uint8_t lis2dh12_temp_data_ready_get_o(void);

int8_t lis2dh12_temp_data_ovr_get(uint8_t *val);
uint8_t lis2dh12_temp_data_ovr_get_o(void);

int8_t lis2dh12_temperature_raw_get(int16_t *val);
int16_t lis2dh12_temperature_raw_get_o(void);

typedef enum
{
  LIS2DH12_TEMP_DISABLE  = 0,
  LIS2DH12_TEMP_ENABLE   = 3,
} lis2dh12_temp_en_t;
int8_t lis2dh12_temperature_meas_set(lis2dh12_temp_en_t val);
void lis2dh12_temperature_meas_set_o(lis2dh12_temp_en_t val);
int8_t lis2dh12_temperature_meas_get(lis2dh12_temp_en_t *val);
lis2dh12_temp_en_t lis2dh12_temperature_meas_get_o(void);

typedef enum
{
  LIS2DH12_HR_12bit   = 0,
  LIS2DH12_NM_10bit   = 1,
  LIS2DH12_LP_8bit    = 2,
} lis2dh12_op_md_t;
int8_t lis2dh12_operating_mode_set(lis2dh12_op_md_t val);
void lis2dh12_operating_mode_set_o(lis2dh12_op_md_t val);
int8_t lis2dh12_operating_mode_get(lis2dh12_op_md_t *val);
lis2dh12_op_md_t lis2dh12_operating_mode_get_o(void);

typedef enum
{
  LIS2DH12_POWER_DOWN                      = 0x00,
  LIS2DH12_ODR_1Hz                         = 0x01,
  LIS2DH12_ODR_10Hz                        = 0x02,
  LIS2DH12_ODR_25Hz                        = 0x03,
  LIS2DH12_ODR_50Hz                        = 0x04,
  LIS2DH12_ODR_100Hz                       = 0x05,
  LIS2DH12_ODR_200Hz                       = 0x06,
  LIS2DH12_ODR_400Hz                       = 0x07,
  LIS2DH12_ODR_1kHz620_LP                  = 0x08,
  LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP    = 0x09,
} lis2dh12_odr_t;
#define LIS2DH12_ODR_MASK                  0xF0
int8_t lis2dh12_data_rate_set(lis2dh12_odr_t val);
void lis2dh12_data_rate_set_o(lis2dh12_odr_t val);

int lis2dh12_odr_to_hz(lis2dh12_odr_t sample_rate);
int8_t lis2dh12_data_rate_get(lis2dh12_odr_t *val);
lis2dh12_odr_t lis2dh12_data_rate_get_o(void);

int8_t lis2dh12_high_pass_on_outputs_set(uint8_t val);
void lis2dh12_high_pass_on_outputs_set_o(uint8_t val);
int8_t lis2dh12_high_pass_on_outputs_get(uint8_t *val);
uint8_t lis2dh12_high_pass_on_outputs_get_o(void);

typedef enum
{
  LIS2DH12_AGGRESSIVE  = 0,
  LIS2DH12_STRONG      = 1,
  LIS2DH12_MEDIUM      = 2,
  LIS2DH12_LIGHT       = 3,
} lis2dh12_hpcf_t;
int8_t lis2dh12_high_pass_bandwidth_set(lis2dh12_hpcf_t val);
void lis2dh12_high_pass_bandwidth_set_o(lis2dh12_hpcf_t val);
int8_t lis2dh12_high_pass_bandwidth_get(lis2dh12_hpcf_t *val);
lis2dh12_hpcf_t lis2dh12_high_pass_bandwidth_get_o(void);

typedef enum
{
  LIS2DH12_NORMAL_WITH_RST  = 0,
  LIS2DH12_REFERENCE_MODE   = 1,
  LIS2DH12_NORMAL           = 2,
  LIS2DH12_AUTORST_ON_INT   = 3,
} lis2dh12_hpm_t;
int8_t lis2dh12_high_pass_mode_set(lis2dh12_hpm_t val);
void lis2dh12_high_pass_mode_set_o(lis2dh12_hpm_t val);
int8_t lis2dh12_high_pass_mode_get(lis2dh12_hpm_t *val);
lis2dh12_hpm_t lis2dh12_high_pass_mode_get_o(void);

typedef enum
{
  LIS2DH12_2g   = 0,
  LIS2DH12_4g   = 1,
  LIS2DH12_8g   = 2,
  LIS2DH12_16g  = 3,
} lis2dh12_fs_t;
int8_t lis2dh12_full_scale_set(lis2dh12_fs_t val);
void lis2dh12_full_scale_set_o(lis2dh12_fs_t val);
int lis2dh12_get_full_scale(void);
int8_t lis2dh12_full_scale_get(lis2dh12_fs_t *val);
lis2dh12_fs_t lis2dh12_full_scale_get_o();

int8_t lis2dh12_block_data_update_set(uint8_t val);
void lis2dh12_block_data_update_set_o(uint8_t val);
int8_t lis2dh12_block_data_update_get(uint8_t *val);
uint8_t lis2dh12_block_data_update_get_o(void);

int8_t lis2dh12_filter_reference_set(uint8_t *buff);
void lis2dh12_filter_reference_set_o(uint8_t buff);
int8_t lis2dh12_filter_reference_get(uint8_t *buff);
uint8_t lis2dh12_filter_reference_get_o(void);

int8_t lis2dh12_xl_data_ready_get(uint8_t *val);
uint8_t lis2dh12_xl_data_ready_get_o(void);

int8_t lis2dh12_xl_data_ovr_get(uint8_t *val);
uint8_t lis2dh12_xl_data_ovr_get_o(void);

int8_t lis2dh12_acceleration_raw_get(int16_t *val);

int8_t lis2dh12_device_id_get(uint8_t *buff);
uint8_t lis2dh12_device_id_get_o(void);

typedef enum
{
  LIS2DH12_ST_DISABLE   = 0,
  LIS2DH12_ST_POSITIVE  = 1,
  LIS2DH12_ST_NEGATIVE  = 2,
} lis2dh12_st_t;
int8_t lis2dh12_self_test_set(lis2dh12_st_t val);
void lis2dh12_self_test_set_o(lis2dh12_st_t val);
int8_t lis2dh12_self_test_get(lis2dh12_st_t *val);
lis2dh12_st_t lis2dh12_self_test_get_o(void);

typedef enum
{
  LIS2DH12_LSB_AT_LOW_ADD = 0,
  LIS2DH12_MSB_AT_LOW_ADD = 1,
} lis2dh12_ble_t;
int8_t lis2dh12_data_format_set(lis2dh12_ble_t val);
void lis2dh12_data_format_set_o(lis2dh12_ble_t val);
int8_t lis2dh12_data_format_get(lis2dh12_ble_t *val);
lis2dh12_ble_t lis2dh12_data_format_get_o(void);

int8_t lis2dh12_boot_set(uint8_t val);
void lis2dh12_boot_set_o(uint8_t val);
int8_t lis2dh12_boot_get(uint8_t *val);
uint8_t lis2dh12_boot_get_o(void);

int8_t lis2dh12_status_get(lis2dh12_status_reg_t *val);
lis2dh12_status_reg_t lis2dh12_status_get_o(void);

int8_t lis2dh12_int1_gen_conf_set(lis2dh12_int1_cfg_t *val);
void lis2dh12_int1_gen_conf_set_o(lis2dh12_int1_cfg_t val);
int8_t lis2dh12_int1_gen_conf_get(lis2dh12_int1_cfg_t *val);
lis2dh12_int1_cfg_t lis2dh12_int1_gen_conf_get_o(void);

int8_t lis2dh12_int1_gen_source_get(lis2dh12_int1_src_t *val);
lis2dh12_int1_src_t lis2dh12_int1_gen_source_get_o(void);

int8_t lis2dh12_int1_gen_threshold_set(uint8_t val);
void lis2dh12_int1_gen_threshold_set_o(uint8_t val);
int8_t lis2dh12_int1_gen_threshold_get(uint8_t *val);
uint8_t lis2dh12_int1_gen_threshold_get_o(void);

int8_t lis2dh12_int1_gen_duration_set(uint8_t val);
void lis2dh12_int1_gen_duration_set_o(uint8_t val);
int8_t lis2dh12_int1_gen_duration_get(uint8_t *val);
uint8_t lis2dh12_int1_gen_duration_get_o(void);

int8_t lis2dh12_int2_gen_conf_set(lis2dh12_int2_cfg_t *val);
void lis2dh12_int2_gen_conf_set_o(lis2dh12_int2_cfg_t val);
int8_t lis2dh12_int2_gen_conf_get(lis2dh12_int2_cfg_t *val);
lis2dh12_int2_cfg_t lis2dh12_int2_gen_conf_get_o(void);

int8_t lis2dh12_int2_gen_source_get(lis2dh12_int2_src_t *val);
lis2dh12_int2_src_t lis2dh12_int2_gen_source_get_o(void);

int8_t lis2dh12_int2_gen_threshold_set(uint8_t val);
void lis2dh12_int2_gen_threshold_set_o(uint8_t val);
int8_t lis2dh12_int2_gen_threshold_get(uint8_t *val);
uint8_t lis2dh12_int2_gen_threshold_get_o(void);

int8_t lis2dh12_int2_gen_duration_set(uint8_t val);
void lis2dh12_int2_gen_duration_set_o(uint8_t val);
int8_t lis2dh12_int2_gen_duration_get(uint8_t *val);
uint8_t lis2dh12_int2_gen_duration_get_o(void);

typedef enum
{
  LIS2DH12_DISC_FROM_INT_GENERATOR  = 0,
  LIS2DH12_ON_INT1_GEN              = 1,
  LIS2DH12_ON_INT2_GEN              = 2,
  LIS2DH12_ON_TAP_GEN               = 4,
  LIS2DH12_ON_INT1_INT2_GEN         = 3,
  LIS2DH12_ON_INT1_TAP_GEN          = 5,
  LIS2DH12_ON_INT2_TAP_GEN          = 6,
  LIS2DH12_ON_INT1_INT2_TAP_GEN     = 7,
} lis2dh12_hp_t;
int8_t lis2dh12_high_pass_int_conf_set(lis2dh12_hp_t val);
void lis2dh12_high_pass_int_conf_set_o(lis2dh12_hp_t val);
int8_t lis2dh12_high_pass_int_conf_get(lis2dh12_hp_t *val);
lis2dh12_hp_t lis2dh12_high_pass_int_conf_get_o(void);

int8_t lis2dh12_pin_int1_config_set(lis2dh12_ctrl_reg3_t *val);
void lis2dh12_pin_int1_config_set_o(lis2dh12_ctrl_reg3_t val);
int8_t lis2dh12_pin_int1_config_get(lis2dh12_ctrl_reg3_t *val);
lis2dh12_ctrl_reg3_t lis2dh12_pin_int1_config_get_o(void);


int8_t lis2dh12_int2_pin_detect_4d_set(uint8_t val);
void lis2dh12_int2_pin_detect_4d_set_o(uint8_t val);
int8_t lis2dh12_int2_pin_detect_4d_get(uint8_t *val);
uint8_t lis2dh12_int2_pin_detect_4d_get_o(void);

typedef enum
{
  LIS2DH12_INT2_PULSED   = 0,
  LIS2DH12_INT2_LATCHED  = 1,
} lis2dh12_lir_int2_t;
int8_t lis2dh12_int2_pin_notification_mode_set(lis2dh12_lir_int2_t val);
void lis2dh12_int2_pin_notification_mode_set_o(lis2dh12_lir_int2_t val);
int8_t lis2dh12_int2_pin_notification_mode_get(lis2dh12_lir_int2_t *val);
lis2dh12_lir_int2_t lis2dh12_int2_pin_notification_mode_get_o(void);

int8_t lis2dh12_int1_pin_detect_4d_set(uint8_t val);
void lis2dh12_int1_pin_detect_4d_set_o(uint8_t val);
int8_t lis2dh12_int1_pin_detect_4d_get(uint8_t *val);
uint8_t lis2dh12_int1_pin_detect_4d_get_o(void);

typedef enum
{
  LIS2DH12_INT1_PULSED   = 0,
  LIS2DH12_INT1_LATCHED  = 1,
} lis2dh12_lir_int1_t;
int8_t lis2dh12_int1_pin_notification_mode_set(lis2dh12_lir_int1_t val);
void lis2dh12_int1_pin_notification_mode_set_o(lis2dh12_lir_int1_t val);
int8_t lis2dh12_int1_pin_notification_mode_get(lis2dh12_lir_int1_t *val);
lis2dh12_lir_int1_t lis2dh12_int1_pin_notification_mode_get_o(void);

int8_t lis2dh12_pin_int2_config_set(lis2dh12_ctrl_reg6_t *val);
void lis2dh12_pin_int2_config_set_o(lis2dh12_ctrl_reg6_t val);
int8_t lis2dh12_pin_int2_config_get(lis2dh12_ctrl_reg6_t *val);
lis2dh12_ctrl_reg6_t lis2dh12_pin_int2_config_get_o(void);

int8_t lis2dh12_fifo_set(uint8_t val);
void lis2dh12_fifo_set_o(uint8_t val);
int8_t lis2dh12_fifo_get(uint8_t *val);
uint8_t lis2dh12_fifo_get_o(void);

int8_t lis2dh12_fifo_watermark_set(uint8_t val);
void lis2dh12_fifo_watermark_set_o(uint8_t val);
int8_t lis2dh12_fifo_watermark_get(uint8_t *val);
uint8_t lis2dh12_fifo_watermark_get_o(void);

typedef enum
{
  LIS2DH12_INT1_GEN = 0,
  LIS2DH12_INT2_GEN = 1,
} lis2dh12_tr_t;
int8_t lis2dh12_fifo_trigger_event_set(lis2dh12_tr_t val);
void lis2dh12_fifo_trigger_event_set_o(lis2dh12_tr_t val);
int8_t lis2dh12_fifo_trigger_event_get(lis2dh12_tr_t *val);
lis2dh12_tr_t lis2dh12_fifo_trigger_event_get_o(void);

typedef enum
{
  LIS2DH12_BYPASS_MODE           = 0,
  LIS2DH12_FIFO_MODE             = 1,
  LIS2DH12_DYNAMIC_STREAM_MODE   = 2,
  LIS2DH12_STREAM_TO_FIFO_MODE   = 3,
} lis2dh12_fm_t;
int8_t lis2dh12_fifo_mode_set(lis2dh12_fm_t val);
void lis2dh12_fifo_mode_set_o(lis2dh12_fm_t val);
int8_t lis2dh12_fifo_mode_get(lis2dh12_fm_t *val);
lis2dh12_fm_t lis2dh12_fifo_mode_get_o(void);

int8_t lis2dh12_fifo_status_get(lis2dh12_fifo_src_reg_t *val);
lis2dh12_fifo_src_reg_t lis2dh12_fifo_status_get_o(void);

int8_t lis2dh12_fifo_data_level_get(uint8_t *val);
uint8_t lis2dh12_fifo_data_level_get_o(void);

int8_t lis2dh12_fifo_empty_flag_get(uint8_t *val);
uint8_t lis2dh12_fifo_empty_flag_get_o(void);

int8_t lis2dh12_fifo_ovr_flag_get(uint8_t *val);
uint8_t lis2dh12_fifo_ovr_flag_get_o(void);

int8_t lis2dh12_fifo_fth_flag_get(uint8_t *val);
uint8_t lis2dh12_fifo_fth_flag_get_o(void);

int8_t lis2dh12_tap_conf_set(lis2dh12_click_cfg_t *val);
void lis2dh12_tap_conf_set_o(lis2dh12_click_cfg_t val);
int8_t lis2dh12_tap_conf_get(lis2dh12_click_cfg_t *val);
lis2dh12_click_cfg_t lis2dh12_tap_conf_get_o(void);

int8_t lis2dh12_tap_source_get(lis2dh12_click_src_t *val);
lis2dh12_click_src_t lis2dh12_tap_source_get_o(void);

int8_t lis2dh12_tap_threshold_set(uint8_t val);
void lis2dh12_tap_threshold_set_o(uint8_t val);
int8_t lis2dh12_tap_threshold_get(uint8_t *val);
uint8_t lis2dh12_tap_threshold_get_o(void);

typedef enum
{
  LIS2DH12_TAP_PULSED   = 0,
  LIS2DH12_TAP_LATCHED  = 1,
} lis2dh12_lir_click_t;
int8_t lis2dh12_tap_notification_mode_set(lis2dh12_lir_click_t val);
void lis2dh12_tap_notification_mode_set_o(lis2dh12_lir_click_t val);
int8_t lis2dh12_tap_notification_mode_get(lis2dh12_lir_click_t *val);
lis2dh12_lir_click_t lis2dh12_tap_notification_mode_get_o(void);

int8_t lis2dh12_shock_dur_set(uint8_t val);
void lis2dh12_shock_dur_set_o(uint8_t val);
int8_t lis2dh12_shock_dur_get(uint8_t *val);
uint8_t lis2dh12_shock_dur_get_o(void);

int8_t lis2dh12_quiet_dur_set(uint8_t val);
void lis2dh12_quiet_dur_set_o(uint8_t val);
int8_t lis2dh12_quiet_dur_get(uint8_t *val);
uint8_t lis2dh12_quiet_dur_get_o(void);

int8_t lis2dh12_double_tap_timeout_set(uint8_t val);
void lis2dh12_double_tap_timeout_set_o(uint8_t val);
int8_t lis2dh12_double_tap_timeout_get(uint8_t *val);
uint8_t lis2dh12_double_tap_timeout_get_o(void);

int8_t lis2dh12_act_threshold_set(uint8_t val);
void lis2dh12_act_threshold_set_o(uint8_t val);
int8_t lis2dh12_act_threshold_get(uint8_t *val);
uint8_t lis2dh12_act_threshold_get_o(void);

int8_t lis2dh12_act_timeout_set(uint8_t val);
void lis2dh12_act_timeout_set_o(uint8_t val);
int8_t lis2dh12_act_timeout_get(uint8_t *val);
uint8_t lis2dh12_act_timeout_get_o(void);

typedef enum
{
  LIS2DH12_PULL_UP_CONNECT     = 0,
  LIS2DH12_PULL_UP_DISCONNECT  = 1,
} lis2dh12_sdo_pu_disc_t;
int8_t lis2dh12_pin_sdo_sa0_mode_set(lis2dh12_sdo_pu_disc_t val);
void lis2dh12_pin_sdo_sa0_mode_set_o(lis2dh12_sdo_pu_disc_t val);
int8_t lis2dh12_pin_sdo_sa0_mode_get(lis2dh12_sdo_pu_disc_t *val);
lis2dh12_sdo_pu_disc_t lis2dh12_pin_sdo_sa0_mode_get_o(void);

#ifdef LIS2DH12_FLOAT
float_t lis2dh12_rawToMg(int16_t raw_acceleration);
#else
int16_t lis2dh12_rawToMg(int16_t raw_acceleration);
#endif
int16_t lis2dh12_rawToC10(int16_t data_raw_temperature);

uint8_t lis2dh12_scale_interrupt_threshold(uint16_t threshold_mg);

extern lis2dh12_fs_t state_scale;
extern lis2dh12_op_md_t state_resolution;

#ifdef __cplusplus
}
#endif

#endif /* LIS2DH12_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
