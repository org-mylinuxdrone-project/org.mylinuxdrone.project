/*
 * pru_spi_lib.c
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
#include <pru_spi_lib.h>
#include <pru_ctrl.h>
#include <pru_cfg.h>

#ifdef PRU1_CTRL
#define PRU_CTRL PRU1_CTRL
#else
#define PRU_CTRL PRU0_CTRL
#endif

volatile register uint32_t __R30;
volatile register uint32_t __R31;

inline void pru_spi_DelayCycles(uint8_t cycles)
{
    /* Enable counter */
    PRU_CTRL.CYCLE = 0;
    PRU_CTRL.CTRL_bit.CTR_EN = 1;
    while ( PRU_CTRL.CYCLE < cycles - 3)
        ;
    // disable counter
    PRU_CTRL.CTRL_bit.CTR_EN = 0;
}

void pru_spi_transferData(PruSpiStatus* status, uint16_t hwords)
{
    //Set the CFG Register to direct output instead of serial output
    CT_CFG.GPCFG1 = 0;

    // enable counter
//    PRU_CTRL.CYCLE = 0;
//    PRU_CTRL.CTRL_bit.CTR_EN = 1;

    uint16_t pos = 0;
    uint16_t miso = 0;
    uint16_t mosi = 0;
    uint16_t counter = 0;
    uint16_t* mosiPtr  = status->mosiData;
    uint16_t* misoPtr  = status->misoData;

    // reset clock and select device
    __R30 |= (0x10);
    __R30 &= ~(1 << 10);

    // transfer bytes (MSB first, SPI Mode 3 (cpha=1, cpol=1))
    for (pos = 1 << (hwords -1); pos != 0; pos = pos >> 1)
    {
        mosi = *(mosiPtr);
        mosiPtr++;

        // transfer byte
        for (counter = 0x8000; counter != 0; counter = counter >> 1)
        {

            // clock down
            __R30 &= ~(0x10);

            // transfer mosi bit
            if ((mosi & counter)) {
                __R30 |= 0x80;
            }
            else {
                __R30 &= ~(0x80);
            }
            __delay_cycles(2);

            // read miso bit
            if (__R31 & 0x100) {
                miso |= counter;
            }
            else {
                miso &= ~(counter);
            }

            // clock up
            __R30 |= 0x10;

            __delay_cycles(2);
        }
        *(misoPtr) = miso;
        misoPtr++;
    }

    // deselect device
    __R30 |= (1 << 10);
    __delay_cycles(20);
}
