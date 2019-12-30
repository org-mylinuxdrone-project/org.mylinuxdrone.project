/*
 * main.c
 *
 *  Created on: 27 dic 2019
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

#include <resource_table.h>
#include <pru_mpu_6500_spi_lib.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>

#ifdef PRU1_CTRL
#define PRU_CTRL PRU1_CTRL
#else
#define PRU_CTRL PRU0_CTRL
#endif

#define MPU_INT 9       //P8_29

uint8_t RESULT = 0x00;
uint16_t CHECK = 0x00;
uint16_t ERROR = 0x00;
uint32_t COUNTER = 0x00;
uint16_t mosiData[32] = { };
uint16_t misoData[32] = { };
uint32_t RESULT32 = 0x00;
uint32_t ADDRESS = 0xF5000000;

int16_t temp_raw = 0;
int16_t acc_axis_raw[3] = {};
int16_t gyro_axis_raw[3] = {};

volatile register uint32_t __R31;

/**
 * main.c
 */
int main(void)
{
    /*
     * CT_CFG.SYSCFG_bit.STANDBY_INIT : the object is used to write data to
     * the SYSCFG register. Writing 0 to the STANDBY_INIT section of the
     * register opens up the OCP master port that is used for PRU to ARM
     * communication.
     */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    CHECK = 0;
    ERROR = 0;
    while(mpu_6500_spi_testConnection())
    {
        CHECK++;
        ERROR = 1;
    }
    while(mpu_6500_spi_init())
    {
        CHECK++;
        ERROR = 2;
    }
    __delay_cycles(1000);
//    while(mpu_6500_spi_start()) {
//        CHECK++;
//        ERROR = 3;
//    }
    while (1)
    {
        if(__R31 & (1 << MPU_INT)) {
            if(mpu_6500_spi_get_data(acc_axis_raw, gyro_axis_raw, &temp_raw)) {
                CHECK++;
                ERROR = 3;
                continue;
            }
        }
//        __delay_cycles(50);
//        RESULT = mpu_6500_spi_read_register(26);
//        if(RESULT != 7) {
//            CHECK++;
//            ERROR = 4;
//            continue;
//        }
    }
}

