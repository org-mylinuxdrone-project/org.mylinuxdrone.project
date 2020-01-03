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

#define MPU_6500_SPI_WHOAMI_REGISTER              117
#define MPU_6500_SPI_ACC_OFFSET_REGISTER          119
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_X_H      19
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_X_L      20
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_Y_H      21
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_Y_L      22
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_Z_H      23
#define MPU_6500_SPI_GYRO_OFFSET_REGISTER_Z_L      24
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_X_H      119
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_X_L      120
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_Y_H      122
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_Y_L      123
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_Z_H      125
#define MPU_6500_SPI_ACC_OFFSET_REGISTER_Z_L      126
#define MPU_6500_SPI_ACCEL_RAW_REGISTER            59
#define MPU_6500_SPI_TEMP_RAW_REGISTER             65
#define MPU_6500_SPI_GYRO_RAW_REGISTER             67
#define MPU_6500_SPI_CONF_REGISTER                 26
#define MPU_6500_SPI_GYRO_CONF_REGISTER            27
#define MPU_6500_SPI_ACC_CONF1_REGISTER            28
#define MPU_6500_SPI_ACC_CONF2_REGISTER            29
#define MPU_6500_SPI_PWR_MGMT1_REGISTER           107
#define MPU_6500_SPI_PWR_MGMT2_REGISTER           108
#define MPU_6500_SPI_INT_ENABLE_REGISTER           56
#define MPU_6500_SPI_INT_STATUS_REGISTER           58

uint16_t mpu_6500_spi_miso_buffer[6] = {};
uint16_t mpu_6500_spi_mosi_buffer[6] = {};

int32_t mpu_6500_spi_gyro_accumulate[3] = {};
int32_t mpu_6500_spi_acc_accumulate[3] = {};

int32_t mpu_6500_spi_gyro_offset[3] = {};
int32_t mpu_6500_spi_acc_offset[3] = {};

int32_t mpu_6500_spi_gyro_offset_reload[3] = {};
int32_t mpu_6500_spi_acc_offset_reload[3] = {};

uint16_t mpu_6500_spi_calib_counter = 2048;
uint16_t mpu_6500_spi_discard_counter = 2048;

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

    mpu_6500_spi_gyro_accumulate[0] = 0;
    mpu_6500_spi_gyro_accumulate[1] = 0;
    mpu_6500_spi_gyro_accumulate[2] = 0;
    mpu_6500_spi_acc_accumulate[0] = 0;
    mpu_6500_spi_acc_accumulate[1] = 0;
    mpu_6500_spi_acc_accumulate[2] = 0;

    return 0;
}

int8_t mpu_6500_spi_get_data(int16_t* acc, int16_t* gyro, int16_t* temp) {

    int8_t result = 0;

    // load acc raw data
    mpu_6500_spi_mosi_buffer[0] = (MPU_6500_SPI_ACCEL_RAW_REGISTER -1) << 8 ; // result start at position 1 on miso buffer
    result = pru_spi_readData(mpu_6500_spi_mosi_buffer, mpu_6500_spi_miso_buffer, 4);
    acc[0] = mpu_6500_spi_miso_buffer[1];
    acc[1] = mpu_6500_spi_miso_buffer[2];
    acc[2] = mpu_6500_spi_miso_buffer[3];

    // read temperature raw
    /*
     * TEMP_degC = ((TEMP_OUT –RoomTemp_Offset)/Temp_Sensitivity)+ 21degC
     * es. (1961 - 0)/333.87 + 21 = 26,87
     *
     * Se si vuole restituire in gradi centigradi, questo è il modo:
     *
     * Considerare che 333,87*2^8 = 85470 e che (21 + 0,55) * 2^8 = 5516, si effettua il conto in virgola fissa (8 bit)
     *     int32_t appTemp = 0;
     *     appTemp = pru_spi_read16(MPU_6500_SPI_TEMP_RAW_REGISTER -1) << 16;
     *     *temp  = (((appTemp / 85470) + 5516) >> 8);
     */
      *temp = pru_spi_read16(MPU_6500_SPI_TEMP_RAW_REGISTER -1);

    // load gyro raw data
    mpu_6500_spi_mosi_buffer[0] = (MPU_6500_SPI_GYRO_RAW_REGISTER -1) << 8 ; // result start at position 1 on miso buffer
    result = pru_spi_readData(mpu_6500_spi_mosi_buffer, mpu_6500_spi_miso_buffer, 4);
    gyro[0] = mpu_6500_spi_miso_buffer[1];
    gyro[1] = mpu_6500_spi_miso_buffer[2];
    gyro[2] = mpu_6500_spi_miso_buffer[3];

    if(mpu_6500_spi_calib_counter == 0) {
        return result;
    } else
    if(mpu_6500_spi_discard_counter == 0) {
            mpu_6500_spi_calib_counter--;
            mpu_6500_spi_gyro_accumulate[0] += gyro[0];
            mpu_6500_spi_gyro_accumulate[1] += gyro[1];
            mpu_6500_spi_gyro_accumulate[2] += gyro[2];
            mpu_6500_spi_acc_accumulate[0] += acc[0];
            mpu_6500_spi_acc_accumulate[1] += acc[1];
            mpu_6500_spi_acc_accumulate[2] += acc[2];
            if(mpu_6500_spi_calib_counter == 0) {
                mpu_6500_spi_gyro_offset[0] = mpu_6500_spi_gyro_accumulate[0] >> 11;
                mpu_6500_spi_gyro_offset[1] = mpu_6500_spi_gyro_accumulate[1] >> 11;
                mpu_6500_spi_gyro_offset[2] = mpu_6500_spi_gyro_accumulate[2] >> 11;
                mpu_6500_spi_acc_offset[0] = mpu_6500_spi_acc_accumulate[0] >> 11;
                mpu_6500_spi_acc_offset[1] = mpu_6500_spi_acc_accumulate[1] >> 11;
                mpu_6500_spi_acc_offset[2] = mpu_6500_spi_acc_accumulate[2] >> 11;

                // Assegno offsets gyro
                mpu_6500_spi_mosi_buffer[0] = (MPU_6500_SPI_GYRO_OFFSET_REGISTER_X_H -1) << 8 ; // result start at position 1 on miso buffer
                result = pru_spi_readData(mpu_6500_spi_mosi_buffer, mpu_6500_spi_miso_buffer, 4);
                mpu_6500_spi_gyro_offset[0] += mpu_6500_spi_miso_buffer[1];
                mpu_6500_spi_gyro_offset[1] += mpu_6500_spi_miso_buffer[2];
                mpu_6500_spi_gyro_offset[2] += mpu_6500_spi_miso_buffer[3];

                // Assegno offsets accel
                mpu_6500_spi_acc_offset[0] += pru_spi_read16(MPU_6500_SPI_ACC_OFFSET_REGISTER_X_H -1);
                mpu_6500_spi_acc_offset[1] += pru_spi_read16(MPU_6500_SPI_ACC_OFFSET_REGISTER_Y_H -1);
                mpu_6500_spi_acc_offset[2] += pru_spi_read16(MPU_6500_SPI_ACC_OFFSET_REGISTER_Z_H -1);

                // Assegno registri offset gyro
                pru_spi_write16(MPU_6500_SPI_GYRO_OFFSET_REGISTER_X_H, mpu_6500_spi_gyro_offset[0]);
                pru_spi_write16(MPU_6500_SPI_GYRO_OFFSET_REGISTER_Y_H, mpu_6500_spi_gyro_offset[1]);
                pru_spi_write16(MPU_6500_SPI_GYRO_OFFSET_REGISTER_Z_H, mpu_6500_spi_gyro_offset[2]);

                // Assegno registri offset gyro
                pru_spi_write16(MPU_6500_SPI_ACC_OFFSET_REGISTER_X_H, mpu_6500_spi_acc_offset[0]);
                pru_spi_write16(MPU_6500_SPI_ACC_OFFSET_REGISTER_Y_H, mpu_6500_spi_acc_offset[1]);
                pru_spi_write16(MPU_6500_SPI_ACC_OFFSET_REGISTER_Z_H, mpu_6500_spi_acc_offset[2]);

                // Solo per test ricarico i valori (da togliere)
                // Assegno offsets reload gyro
                mpu_6500_spi_mosi_buffer[0] = (MPU_6500_SPI_GYRO_OFFSET_REGISTER_X_H -1) << 8 ; // result start at position 1 on miso buffer
                result = pru_spi_readData(mpu_6500_spi_mosi_buffer, mpu_6500_spi_miso_buffer, 4);
                mpu_6500_spi_gyro_offset_reload[0] = mpu_6500_spi_miso_buffer[1];
                mpu_6500_spi_gyro_offset_reload[1] = mpu_6500_spi_miso_buffer[2];
                mpu_6500_spi_gyro_offset_reload[2] = mpu_6500_spi_miso_buffer[3];

                // Assegno offsets reload accel
                mpu_6500_spi_acc_offset_reload[0] = pru_spi_read16(MPU_6500_SPI_ACC_OFFSET_REGISTER_X_H -1);
                mpu_6500_spi_acc_offset_reload[1] = pru_spi_read16(MPU_6500_SPI_ACC_OFFSET_REGISTER_Y_H -1);
                mpu_6500_spi_acc_offset_reload[2] = pru_spi_read16(MPU_6500_SPI_ACC_OFFSET_REGISTER_Z_H -1);
            }
    } else {
        mpu_6500_spi_discard_counter--;
    }
    return result;
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

