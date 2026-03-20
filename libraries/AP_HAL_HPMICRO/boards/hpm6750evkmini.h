/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This board that does not contain any sensors (but pins are active), it is a great help for a novice user,
 * by flashing an empty board, you can connect via Mavlink (Mission Planner - MP) and gradually add sensors.
 * If you had some sensor configured and it doesn't work then the MP connection does not work and then you may not know what to do next.
*/

#pragma once

#define TRUE  1
#define FALSE 0

//Protocols
// list of protocols/enum:  ardupilot/libraries/AP_SerialManager/AP_SerialManager.h
// default protocols:    ardupilot/libraries/AP_SerialManager/AP_SerialManager.cpp
// HPMicro serials:    AP_HAL_HPMICRO/HAL_HPM_Class.cpp

//Inertial sensors
#define HAL_INS_DEFAULT HAL_INS_NONE
//#define HAL_INS_DEFAULT HAL_INS_MPU9250_I2C
//#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
//#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 0, 0x68, ROTATION_NONE)
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI( Invensense, HAL_INS_ICM20608_NAME, ROTATION_NONE)
//#define HAL_HPM_I2C_BUSES

//RMT pin number
#define HAL_HPM_RMT_RX_PIN_NUMBER 4

//BAROMETER
#define HAL_BARO_ALLOW_INIT_NO_BARO 1

#define HAL_PWM_GROUPS {\
    { .mcpwm_group_id = 0, .base = HPM_PWM1, .clock = clock_mot2, .rc_frequency = 50, .ch_mask = 0xFF, }, \
}

#define NUM_SERVO_CHANNELS    6

#define AP_BATT_MONITOR_MAX_INSTANCES 3
// GPIO36
#define HAL_BATT_VOLT_PIN (0)
#define HAL_BATT_VOLT_SCALE (18.1)
#define HAL_BATT2_VOLT_PIN (1)
#define HAL_BATT2_VOLT_SCALE (18.1)
//GPIO 32
#define HAL_BATT_CURR_PIN (2)
#define HAL_BATT_CURR_SCALE (36)
#define HAL_BATT2_CURR_PIN (3)
#define HAL_BATT2_CURR_SCALE (36)
//ADC
// #define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

//LED
#define DEFAULT_NTF_LED_TYPES Notify_LED_None
// #define AP_FEATURE_BOARD_DETECT 1
#define AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED 0
#define AP_BARO_BACKEND_DEFAULT_ENABLED 0
/* string names for well known SPI devices */
#define HAL_BARO_MS5611_NAME "ms5611"
#ifndef HAL_BARO_MS5611_SPI_INT_NAME
#define HAL_BARO_MS5611_SPI_INT_NAME "ms5611_int"
#endif
#define HAL_BARO_MS5611_SPI_EXT_NAME "ms5611_ext"
#define HAL_BARO_LPS22H_NAME "lps22h"
#define HAL_BARO_BMP280_NAME "bmp280"

#define HAL_INS_MPU60x0_NAME "mpu6000"
#define HAL_INS_MPU60x0_EXT_NAME "mpu6000_ext"

#define HAL_INS_LSM9DS0_G_NAME "lsm9ds0_g"
#define HAL_INS_LSM9DS0_A_NAME "lsm9ds0_am"

#define HAL_INS_LSM9DS0_EXT_G_NAME "lsm9ds0_ext_g"
#define HAL_INS_LSM9DS0_EXT_A_NAME "lsm9ds0_ext_am"

#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_MPU9250_EXT_NAME "mpu9250_ext"

#define HAL_INS_MPU6500_NAME "mpu6500"

#define HAL_INS_ICM20608_NAME "icm20608"
#define HAL_INS_ICM20608_AM_NAME "icm20608-am"
#define HAL_INS_ICM20608_EXT_NAME "icm20608_ext"

// disable all frames for sim on hw except quad to save DRAM .bss
