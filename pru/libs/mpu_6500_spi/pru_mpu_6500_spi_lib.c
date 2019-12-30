/*
 * mpu_6500_spi.c
 *  Created on: 30 dic 2019
 *      Author: Andrea Lambruschini <andrea.lambruschini@gmail.com>
 *
 * Copyright 2019 Andrea Lambruschini
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License.
 */

#include <pru_mpu_6500_spi_lib.h>
#include <pru_spi_lib.h>

#define MPU_6500_SPI_WHOAMI_REGISTER       117
#define MPU_6500_SPI_ACC_OFFSET_REGISTER   119
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER   19
#define MPU_6500_SPI_ACCEL_RAW_REGISTER     59
#define MPU_6500_SPI_TEMP_RAW_REGISTER      65
#define MPU_6500_SPI_GYRO_RAW_REGISTER      67
#define MPU_6500_SPI_CONF_REGISTER          26
#define MPU_6500_SPI_GYRO_CONF_REGISTER     27
#define MPU_6500_SPI_ACC_CONF1_REGISTER     28
#define MPU_6500_SPI_ACC_CONF2_REGISTER     29
#define MPU_6500_SPI_PWR_MGMT1_REGISTER    107
#define MPU_6500_SPI_PWR_MGMT2_REGISTER    108
#define MPU_6500_SPI_INT_ENABLE_REGISTER    56
#define MPU_6500_SPI_INT_STATUS_REGISTER    58

uint16_t mpu_6500_spi_miso_buffer[32] = {};
uint16_t mpu_6500_spi_mosi_buffer[32] = {};

int16_t mpu_6500_spi_acc_axis_offset[3] = {};
int16_t mpu_6500_spi_gyro_axis_offset[3] = {};

uint8_t mpu_6500_spi_conf_reg = 0;
uint8_t mpu_6500_spi_gyro_conf_reg = 0;
uint8_t mpu_6500_spi_acc_conf1_reg = 0;
uint8_t mpu_6500_spi_acc_conf2_reg = 0;
uint8_t mpu_6500_spi_acc_low_power_reg = 0;
uint8_t mpu_6500_spi_fifo_enabled_reg = 0;
uint8_t mpu_6500_spi_interrupt_enabled_reg = 0;
uint8_t mpu_6500_spi_interrupt_status_reg = 0;
uint8_t mpu_6500_spi_acc_int_ctrl_reg = 0;
uint8_t mpu_6500_spi_usr_ctrl_reg = 0;
uint8_t mpu_6500_spi_pwr_mgmt1_reg = 0;
uint8_t mpu_6500_spi_pwr_mgmt2_reg = 0;

int16_t mpu_6500_spi_acc_axis_raw[3] = {};
int16_t mpu_6500_spi_gyro_axis_raw[3] = {};

int8_t mpu_6500_spi_init() {
    // set Configuration Register
    pru_spi_write8(MPU_6500_SPI_CONF_REGISTER, 0x01); // gyro bandwidth 184Hz

    // set Gyro Configuration Register
    pru_spi_write8(MPU_6500_SPI_GYRO_CONF_REGISTER, 0x0); // +-250deg/sec

    // set Acc Conf1 Register
    pru_spi_write8(MPU_6500_SPI_ACC_CONF1_REGISTER, 0x10); // 8g

    // set Acc Conf2 Register
    pru_spi_write8(MPU_6500_SPI_ACC_CONF2_REGISTER, 0x02); // bandwidth 92Hz

    // set PwrMgmt2 Register
    pru_spi_write8(MPU_6500_SPI_PWR_MGMT2_REGISTER, 0x00);

    // set PwrMgmt1 Register
    pru_spi_write8(MPU_6500_SPI_PWR_MGMT1_REGISTER, 0x01); // clock, cycle

    // set Int Enable Register
    pru_spi_write8(MPU_6500_SPI_INT_ENABLE_REGISTER, 0x01); // data ready int

    // set Int Status Register
    pru_spi_write8(MPU_6500_SPI_INT_STATUS_REGISTER, 0x00); // reset all interrupts?

    return mpu_6500_spi_loadConfigurations();
}

int8_t mpu_6500_spi_get_data(int16_t* acc, int16_t* gyro, int16_t* temp) {

    int8_t result = 0;
    int32_t appTemp = 0;
    mpu_6500_spi_mosi_buffer[0] = (MPU_6500_SPI_ACCEL_RAW_REGISTER -1) << 8 ; // result start at position 1 on miso buffer
    result = pru_spi_readData(mpu_6500_spi_mosi_buffer, mpu_6500_spi_miso_buffer, 4);
    // load acc raw data
    acc[0] = mpu_6500_spi_miso_buffer[1];
    acc[1] = mpu_6500_spi_miso_buffer[2];
    acc[2] = mpu_6500_spi_miso_buffer[3];

    // read temperature raw
    /*
     * TEMP_degC = ((TEMP_OUT –RoomTemp_Offset)/Temp_Sensitivity)+ 21degC
     * es. (1961 - 0)/333.87 + 21 = 26,87
     *
     * 333,87*2^8 = 85470
     * (21 + 0,55) * 2^8 = 5516
     */
    appTemp = pru_spi_read16(MPU_6500_SPI_TEMP_RAW_REGISTER -1) << 16;
    *temp  = (((appTemp / 85470) + 5516) >> 8);

    mpu_6500_spi_mosi_buffer[0] = (MPU_6500_SPI_GYRO_RAW_REGISTER -1) << 8 ; // result start at position 1 on miso buffer
    result = pru_spi_readData(mpu_6500_spi_mosi_buffer, mpu_6500_spi_miso_buffer, 4);
    // load gyro raw data
    gyro[0] = mpu_6500_spi_miso_buffer[1];
    gyro[1] = mpu_6500_spi_miso_buffer[2];
    gyro[2] = mpu_6500_spi_miso_buffer[3];
    return result;
}

int8_t mpu_6500_spi_loadConfigurations() {
    // read configuration
    mpu_6500_spi_conf_reg = pru_spi_read8(MPU_6500_SPI_CONF_REGISTER);
    __delay_cycles(10);

    // read gyro configuration
    mpu_6500_spi_gyro_conf_reg = pru_spi_read8(MPU_6500_SPI_GYRO_CONF_REGISTER);
    __delay_cycles(10);

    // read acc configuration 1
    mpu_6500_spi_acc_conf1_reg = pru_spi_read8(MPU_6500_SPI_ACC_CONF1_REGISTER);
    __delay_cycles(10);

    // read acc configuration 2
    mpu_6500_spi_acc_conf2_reg = pru_spi_read8(MPU_6500_SPI_ACC_CONF2_REGISTER);
    __delay_cycles(10);

    // read interrupt enabled conf
    mpu_6500_spi_interrupt_enabled_reg = pru_spi_read8(MPU_6500_SPI_INT_ENABLE_REGISTER);
    __delay_cycles(10);

    // read interrupt status
    mpu_6500_spi_interrupt_status_reg = pru_spi_read8(MPU_6500_SPI_INT_STATUS_REGISTER);
    __delay_cycles(10);

    // read power management 1
    mpu_6500_spi_pwr_mgmt1_reg = pru_spi_read8(MPU_6500_SPI_PWR_MGMT1_REGISTER);
    __delay_cycles(10);

    // read power management 2
    mpu_6500_spi_pwr_mgmt2_reg = pru_spi_read8(MPU_6500_SPI_PWR_MGMT2_REGISTER);
    __delay_cycles(10);

    return mpu_6500_spi_loadOffsets();
}

int8_t mpu_6500_spi_loadOffsets() {
    // read accel offsets
    mpu_6500_spi_mosi_buffer[0] = (MPU_6500_SPI_ACC_OFFSET_REGISTER -1) << 8 ; // result start at position 1 on miso buffer
    if(pru_spi_readData(mpu_6500_spi_mosi_buffer, mpu_6500_spi_miso_buffer, 4)) {
        return -1;
    }
    mpu_6500_spi_acc_axis_offset[X_AXIS] = mpu_6500_spi_miso_buffer[1];
    mpu_6500_spi_acc_axis_offset[Y_AXIS] = mpu_6500_spi_miso_buffer[2];
    mpu_6500_spi_acc_axis_offset[Z_AXIS] = mpu_6500_spi_miso_buffer[3];

    mpu_6500_spi_mosi_buffer[0] = (MPU_6500_SPI_GYRO_OFFSET_REGISTER -1) << 8 ; // result start at position 1 on miso buffer
    if(pru_spi_readData(mpu_6500_spi_mosi_buffer, mpu_6500_spi_miso_buffer, 4)) {
        return -1;
    }
    // load gyro offsets
    mpu_6500_spi_gyro_axis_offset[X_AXIS] = mpu_6500_spi_miso_buffer[1];
    mpu_6500_spi_gyro_axis_offset[Y_AXIS] = mpu_6500_spi_miso_buffer[2];
    mpu_6500_spi_gyro_axis_offset[Z_AXIS] = mpu_6500_spi_miso_buffer[3];
    return 0;
}

int8_t mpu_6500_spi_testConnection(){
    return ((pru_spi_read8(MPU_6500_SPI_WHOAMI_REGISTER) == 0x70) ? 0 : -1);
}

int8_t mpu_6500_spi_read_register(uint8_t address) {
    return pru_spi_read8(address);
}

int8_t mpu_6500_spi_start() {
    pru_spi_write8(MPU_6500_SPI_PWR_MGMT1_REGISTER, 0x21); // Cycle, clock enabled
    return 0;
}

int8_t mpu_6500_spi_stop() {
    pru_spi_write8(MPU_6500_SPI_PWR_MGMT1_REGISTER, 0x01); // No Cycle, clock enabled
    return 0;
}

