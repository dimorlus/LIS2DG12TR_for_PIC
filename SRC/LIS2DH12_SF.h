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

#ifndef LIS2DH12_SF_H
#define LIS2DH12_SF_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "lis2dh12_reg.h" //This is the ST library

#define ACCEL_DEFAULT_ADR 0x18

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef union {
  int32_t i32bit[3];
  uint8_t u8bit[12];
} axis3bit32_t;

typedef union {
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;


bool lis2dh12_init(void); //Begin comm with accel at given I2C address, and given wire port
bool lis2dh12_isConnected(void);                                                        //Returns true if an accel sensor is detected at library's I2C address
bool lis2dh12_available(void);                                                          //Returns true if new accel data is available
void lis2dh12_waitForNewData(void);                                                     //Block until new data is available
bool lis2dh12_temperatureAvailable(void);                                               //Returns true if new temp data is available

#ifdef LIS2DH12_FLOAT
float lis2dh12_getX(void); //Return latest accel data in milli-g's. If data has already be read, initiate new read.
float lis2dh12_getY(void);
float lis2dh12_getZ(void);
float lis2dh12_getTemperature(void); //Returns latest temp data in C. If data is old, initiate new read.
#else
int16_t lis2dh12_getX(void); //Return latest accel data in milli-g's. If data has already be read, initiate new read.
int16_t lis2dh12_getY(void);
int16_t lis2dh12_getZ(void);
int16_t lis2dh12_getTemperature(void); //Returns latest temp data in C. If data is old, initiate new read.
#endif
int16_t lis2dh12_getRawX(void); //Return raw 16 bit accel reading
int16_t lis2dh12_getRawY(void);
int16_t lis2dh12_getRawZ(void);

void lis2dh12_parseAccelData(void); //Load sensor data into global vars. Call after new data is avaiable.
void lis2dh12_getTempData(void);

void lis2dh12_enableTemperature(void);  //Enable the onboard temp sensor
void lis2dh12_disableTemperature(void); //Disable the onboard temp sensor

void lis2dh12_setDataRate(uint8_t dataRate); //Set the output data rate of sensor. Higher rates consume more current.
uint8_t lis2dh12_getDataRate(void);              //Returns the output data rate of sensor.

void lis2dh12_setScale(uint8_t scale); //Set full scale: +/-2, 4, 8, or 16g
uint8_t lis2dh12_getScale(void);           //Returns current scale of sensor

void lis2dh12_setMode(uint8_t mode); //Set mode to low, normal, or high data rate
uint8_t lis2dh12_getMode(void);          //Get current sensor mode

void lis2dh12_enableSelfTest(bool direction);
void lis2dh12_disableSelfTest(void);

void lis2dh12_enableTapDetection(void); //Enable the single tap interrupt
void lis2dh12_disableTapDetection(void);
void lis2dh12_setTapThreshold(uint8_t threshold); //Set the 7-bit threshold value for tap and double tap
bool lis2dh12_isTapped(void);                         //Returns true if Z, Y, or X tap detection bits are set

void lis2dh12_setInt1Threshold(uint8_t threshold);
uint8_t lis2dh12_getInt1Threshold(void);
void lis2dh12_setInt1Duration(uint8_t duration);
uint8_t lis2dh12_getInt1Duration(void);

void lis2dh12_setIntPolarity(uint8_t level);
void lis2dh12_setInt1IA1(bool enable);
void lis2dh12_setInt1Latch(bool enable);
void lis2dh12_setInt1(bool enable);

bool lis2dh12_getInt1(void);


#endif /* SparkFun_LIS2DH12_H */
