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
#include <pru_spi_lib.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>

#ifdef PRU1_CTRL
#define PRU_CTRL PRU1_CTRL
#else
#define PRU_CTRL PRU0_CTRL
#endif

#define MOSI    7       //P8_40
#define CLK     4       //P8_41
#define MISO    8       //P8_27
#define CS      10      //P8_28
#define MPU_INT 9       //P8_29

uint16_t RESULT = 0x00;
uint16_t CHECK = 0x00;
uint16_t ERROR = 0x00;
uint32_t COUNTER = 0x00;
uint16_t mosiData[32] = { };
uint16_t misoData[32] = { };
uint32_t RESULT32 = 0x00;
uint32_t ADDRESS = 0xF5000000;

/**
 * main.c
 */
int main(void)
{
    uint32_t i = 0;
    /*
     * CT_CFG.SYSCFG_bit.STANDBY_INIT : the object is used to write data to
     * the SYSCFG register. Writing 0 to the STANDBY_INIT section of the
     * register opens up the OCP master port that is used for PRU to ARM
     * communication.
     */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    CHECK = 0;
    ERROR = 0;
    while (1)
    {
//        for (i = 0x80000000; i != 0; i = i >> 1){
//            RESULT = pru_spi_read16(0xF500);
//            if(RESULT != 0x70) {
//                CHECK++;
//                ERROR = RESULT;
//            }
//        }
//        for (i = 0x80000000; i != 0; i = i >> 1){
//            RESULT32 = pru_spi_read32(ADDRESS);
//            RESULT = (RESULT32 >> 16) & 0x0000FFFF;
//            if(RESULT != 0x70) {
//                CHECK++;
//                ERROR = RESULT;
//            }
//        }

        mosiData[0] = 0xF500;
        mosiData[1] = 0x0000;

        for (i = 0x80000000; i != 0; i = i >> 1)
        {
            pru_spi_transferData(mosiData, misoData, 8);
            RESULT = misoData[0];
            if (RESULT != 0x70)
            {
                CHECK++;
                ERROR = RESULT;
            }
        }
    }
}

