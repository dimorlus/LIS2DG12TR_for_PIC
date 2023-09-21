/*
 * Based on SparkFun LIS2DH12 Arduino library.
 * Modified and optimized for PIC (and other 8-bit MCUs).
 *  
 */

/*
  This is a library written for the SparkFun LIS2DH12
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15420

  Written by Nathan Seidle @ SparkFun Electronics, September 21st, 2019

  The LIS2DH12 is a very low power I2C triple axis accelerometer
  The SparkFun LIS2DH12 library is merely a wrapper for the ST library. Please
  see the lis2dh12_reg files for licensing and portable C functions.

  https://github.com/sparkfun/SparkFun_LIS2DH12_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.9

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <xc.h>
#include <stdbool.h>
#include <string.h>
#include "mcc_generated_files/mcc.h"
#include "LIS2DH12_SF.h"

static bool xIsFresh = false;
static bool yIsFresh = false;
static bool zIsFresh = false;
static bool tempIsFresh = false;

static uint8_t currentScale = 0; //Needed to convert readings to mg
static uint8_t currentMode = 0;  //Needed to convert readings to mg

#ifdef LIS2DH12_FLOAT
static float accelX;
static float accelY;
static float accelZ;
static float temperatureC;
#else
static int16_t accelX;
static int16_t accelY;
static int16_t accelZ;
static int16_t temperatureC;
#endif
static int16_t rawX;
static int16_t rawY;
static int16_t rawZ;

//Begin comm with accel at given I2C address, and given wire port
//Init accel with default settings
bool lis2dh12_init(void)
{
  if (lis2dh12_isConnected() == false) return false;

  //Enable Block Data Update
  lis2dh12_block_data_update_set_o(PROPERTY_ENABLE);

  //Set Output Data Rate to 25Hz
  lis2dh12_setDataRate(LIS2DH12_ODR_25Hz);

  //Set full scale to 2g
  lis2dh12_setScale(LIS2DH12_2g);

  //Enable temperature sensor
  lis2dh12_enableTemperature();

  //Set device in continuous mode with 12 bit resol.
  lis2dh12_setMode(LIS2DH12_HR_12bit);

  return true;
}

//Check to see if IC ack its I2C address. Then check for valid LIS2DH ID.
bool lis2dh12_isConnected(void)
{
 //Something ack'd at this address. Check ID.
 uint8_t whoamI = LIS2DH12_Read1ByteRegister(LIS2DH12_WHO_AM_I);
 if (whoamI == LIS2DH12_ID) return (true);
 return (false);
}

//Returns true if new data is available
bool lis2dh12_available(void)
{
 return lis2dh12_xl_data_ready_get_o();
}

//Blocking wait until new data is available
void lis2dh12_waitForNewData(void)
{
  while (lis2dh12_available() == false) CLRWDT();
}

//Returns true if new temperature data is available
bool lis2dh12_temperatureAvailable(void)
{
  if (lis2dh12_temp_data_ready_get_o()) return true;
  return false;
}

#ifdef LIS2DH12_FLOAT
//Returns X accel of the global accel data
float lis2dh12_getX(void)
{
  if (xIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  xIsFresh = false;
  return (accelX);
}

//Returns Y accel of the global accel data
float lis2dh12_getY(void)
{
  if (yIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  yIsFresh = false;
  return (accelY);
}


//Returns Z accel of the global accel data
float lis2dh12_getZ(void)
{
  if (zIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  zIsFresh = false;
  return (accelZ);
}
//Returns sensor temperature in C
float lis2dh12_getTemperature(void)
{
  if (tempIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_getTempData();
  }
  tempIsFresh = false;
  return (temperatureC);
}
#else
//Returns X accel of the global accel data
int16_t lis2dh12_getX(void)
{
  if (xIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  xIsFresh = false;
  return (accelX);
}

//Returns Y accel of the global accel data
int16_t lis2dh12_getY(void)
{
  if (yIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  yIsFresh = false;
  return (accelY);
}


//Returns Z accel of the global accel data
int16_t lis2dh12_getZ(void)
{
  if (zIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  zIsFresh = false;
  return (accelZ);
}
//Returns sensor temperature in C
int16_t lis2dh12_getTemperature(void)
{
  if (tempIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_getTempData();
  }
  tempIsFresh = false;
  return (temperatureC);
}
#endif
//Returns X of the global accel data
int16_t lis2dh12_getRawX(void)
{
  if (xIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  xIsFresh = false;
  return (rawX);
}

//Returns Y of the global accel data
int16_t lis2dh12_getRawY(void)
{
  if (yIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  yIsFresh = false;
  return (rawY);
}

//Returns Z of the global accel data
int16_t getRawZ(void)
{
  if (zIsFresh == false)
  {
    lis2dh12_waitForNewData(); //Blocking wait until available
    lis2dh12_parseAccelData();
  }
  zIsFresh = false;
  return (rawZ);
}

//Load global vars with latest accel data
//Does not guarantee data is fresh (ie you can read the same accel values multiple times)
void lis2dh12_parseAccelData(void)
{
  // Read accelerometer data
  axis3bit16_t data_raw_acceleration = {0};
  lis2dh12_acceleration_raw_get((int16_t *)data_raw_acceleration.u8bit);

  rawX = data_raw_acceleration.i16bit[0];
  rawY = data_raw_acceleration.i16bit[1];
  rawZ = data_raw_acceleration.i16bit[2];

  state_scale = currentScale;
  state_resolution = currentMode;

  accelX = lis2dh12_rawToMg(rawX);
  accelY = lis2dh12_rawToMg(rawY);
  accelZ = lis2dh12_rawToMg(rawZ);
  xIsFresh = true;
  yIsFresh = true;
  zIsFresh = true;
}

//Load global vars with latest temp data
//Does not guarantee data is fresh (ie you can read the same temp value multiple times)
void lis2dh12_getTempData(void)
{
  //Read temperature data
  axis1bit16_t data_raw_temperature = {0};
  lis2dh12_temperature_raw_get((int16_t *)data_raw_temperature.u8bit);

  #ifdef LIS2DH12_FLOAT
  switch (currentMode)
  {
  case LIS2DH12_HR_12bit: //High resolution
    temperatureC =  ((float_t)data_raw_temperature.i16bit / 256.0f) + 25.0f;
    break;
  case LIS2DH12_NM_10bit: //Normal mode
    temperatureC =  ((float_t)data_raw_temperature.i16bit / 256.0f) + 25.0f;
    break;
  case LIS2DH12_LP_8bit: //Low power mode
    temperatureC =  ((float_t)data_raw_temperature.i16bit / 256.0f) + 25.0f;
    break;
  }
#else
  switch (currentMode)
  {
  case LIS2DH12_HR_12bit: //High resolution
    temperatureC = (100*data_raw_temperature.i16bit / 256) + 250;
    break;
  case LIS2DH12_NM_10bit: //Normal mode
    temperatureC = (100*data_raw_temperature.i16bit / 256) + 250;
    break;
  case LIS2DH12_LP_8bit: //Low power mode
    temperatureC = (100*data_raw_temperature.i16bit / 256) + 250;
    break;
  }
#endif
  tempIsFresh = true;
}

//Enter a self test
void lis2dh12_enableSelfTest(bool direction)
{
  if (direction == true) lis2dh12_self_test_set_o(LIS2DH12_ST_POSITIVE);
  else lis2dh12_self_test_set_o(LIS2DH12_ST_NEGATIVE);
}

//Exit self test
void lis2dh12_disableSelfTest(void)
{
  lis2dh12_self_test_set_o(LIS2DH12_ST_DISABLE);
}

//Set the output data rate of the sensor
void lis2dh12_setDataRate(uint8_t dataRate)
{
  if (dataRate > LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP)
    dataRate = LIS2DH12_ODR_25Hz; //Default to 25Hz
  lis2dh12_data_rate_set_o((lis2dh12_odr_t)dataRate);
}

//Return the output data rate of the sensor
uint8_t lis2dh12_getDataRate(void)
{
  return lis2dh12_data_rate_get_o();
}

//Set full scale of output to +/-2, 4, 8, or 16g
void lis2dh12_setScale(uint8_t scale)
{
  if (scale > LIS2DH12_16g) scale = LIS2DH12_2g; //Default to LIS2DH12_2g

  currentScale = scale; //Used for mg conversion in getX/Y/Z functions

  lis2dh12_full_scale_set_o((lis2dh12_fs_t)scale);
}

//Return the current scale of the sensor
uint8_t lis2dh12_getScale(void)
{
  return lis2dh12_full_scale_get_o();
}

//Enable the onboard temperature sensor
void lis2dh12_enableTemperature(void)
{
  lis2dh12_temperature_meas_set_o(LIS2DH12_TEMP_ENABLE);
}

//Anti-Enable the onboard temperature sensor
void lis2dh12_disableTemperature(void)
{
  lis2dh12_temperature_meas_set_o(LIS2DH12_TEMP_DISABLE);
}

void lis2dh12_setMode(uint8_t mode)
{
  if (mode > LIS2DH12_LP_8bit)  mode = LIS2DH12_HR_12bit; //Default to 12 bit
  currentMode = mode;
  lis2dh12_operating_mode_set_o((lis2dh12_op_md_t)mode);
}

//Return the current mode of the sensor
uint8_t lis2dh12_getMode(void)
{
  return lis2dh12_operating_mode_get_o();
}

void lis2dh12_setInt1Threshold(uint8_t threshold)
{
  lis2dh12_int1_gen_threshold_set_o(threshold);
}

uint8_t lis2dh12_getInt1Threshold(void)
{
  return lis2dh12_int1_gen_threshold_get_o();
}

void lis2dh12_setInt1Duration(uint8_t duration)
{
  lis2dh12_int1_gen_duration_set_o(duration);
}

uint8_t lis2dh12_getInt1Duration(void)
{
  return lis2dh12_int1_gen_duration_get_o();
}

void lis2dh12_setIntPolarity(uint8_t level)
{
  lis2dh12_ctrl_reg6_t val = lis2dh12_pin_int2_config_get_o();
  if (level == HIGH) val.int_polarity = 0; //Clear INT_POLARITY bit for active high
  else val.int_polarity = 1; //Set INT_POLARITY bit for active low
  lis2dh12_pin_int2_config_set_o(val);
}

void lis2dh12_setInt1IA1(bool enable)
{
  lis2dh12_ctrl_reg3_t val = lis2dh12_pin_int1_config_get_o();
  if (enable == true) val.i1_ia1 = 1; //Enable IA1 on INT1
  else val.i1_ia1 = 0; //Disable IA1 on INT1
  lis2dh12_pin_int1_config_set_o(val);
}

bool lis2dh12_getInt1(void)
{
  lis2dh12_int1_src_t val = lis2dh12_int1_gen_source_get_o();
  return (val.ia);
}

void lis2dh12_setInt1Latch(bool enable)
{
  lis2dh12_ctrl_reg5_t ctrl_reg5;
  ctrl_reg5.byte = LIS2DH12_Read1ByteRegister(LIS2DH12_CTRL_REG5);
  if (enable) ctrl_reg5.lir_int1 = 1;
  else ctrl_reg5.lir_int1 = 0;
  LIS2DH12_Write1ByteRegister(LIS2DH12_CTRL_REG5,ctrl_reg5.byte);
}

//Enable X or Y as interrupt sources
void lis2dh12_setInt1(bool enable)
{
  lis2dh12_int1_cfg_t val={0};
  //val.aoi = 0; //Set 'Or' combination of interrupts
  //val._6d = 0; //Set 'Or' combination of interrupts
  if (enable) val.xhie = 1;
  //else val.xhie = 0;

  //val.xlie = 0; //Do not set both low and high

  if (enable) val.yhie = 1;
  //else val.yhie = 0;

  //val.ylie = 0;
  //val.zhie = 0; //Leave out Z otherwise it will always trigger when sitting on table
  //val.zlie = 0;
  lis2dh12_int1_gen_conf_set_o(val);
}

//Enable single tap detection
void lis2dh12_enableTapDetection(void)
{
  lis2dh12_click_cfg_t newBits = lis2dh12_tap_conf_get_o();
  newBits.xs = true;
  newBits.ys = true;
  newBits.zs = true;
  lis2dh12_tap_conf_set_o(newBits);
}

//Disable single tap detection
void lis2dh12_disableTapDetection(void)
{
  lis2dh12_click_cfg_t newBits = lis2dh12_tap_conf_get_o();
  newBits.xs = false;
  newBits.ys = false;
  newBits.zs = false;
  lis2dh12_tap_conf_set_o(newBits);
}

//Set 7 bit threshold value
void lis2dh12_setTapThreshold(uint8_t threshold)
{
  if (threshold > 127) threshold = 127;//Register is 7 bits wide
  lis2dh12_tap_threshold_set_o(threshold);
}

//Returns true if a tap is detected
bool lis2dh12_isTapped(void)
{
  lis2dh12_click_src_t interruptSource = lis2dh12_tap_source_get_o();
  return (interruptSource.x || interruptSource.y || interruptSource.z); //Check if ZYX bits are set
}
