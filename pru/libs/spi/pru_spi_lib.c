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
    CT_CFG.GPCFG0 = 0;

    // enable counter
    PRU_CTRL.CYCLE = 0;
    PRU_CTRL.CTRL_bit.CTR_EN = 1;

    uint16_t pos = 0;
    uint16_t miso = 0;
    uint16_t mosi = 0;
    uint8_t bit = 0;
    uint32_t mosiMask = (1 << status->pins.MOSI);
    uint32_t mosiMaskNeg = ~(1 << status->pins.MOSI);
    uint32_t misoMask = (1 << status->pins.MISO);
    uint32_t clkMask = (1 << status->pins.CLK);
    uint32_t clkMaskNeg = ~(1 << status->pins.CLK);
    uint32_t csMask = (1 << status->pins.CS);
    uint32_t csMaskNeg = ~(1 << status->pins.CS);
    uint8_t clockDelayCycles = status->clockDelayCycles;
    uint16_t setMisoBit = 0x01;
    uint16_t unsetMisoBit = ~(0x01);

    // reset clock and select device
    __R30 |= (1 << status->pins.CLK);
    __R30 &= ~(1 << status->pins.CS);

    // transfer bytes (MSB first, SPI Mode 3 (cpha=1, cpol=1))
    for (pos = 0; pos < hwords; pos++)
    {
        mosi = *(status->mosiData + pos);

        // transfer byte
        for (bit = 0; bit < 16; bit++)
        {
            while ( PRU_CTRL.CYCLE < (clockDelayCycles))
                ;
            miso = miso << 1;

            // clock down
            __R30 &= clkMaskNeg;
            // count from down
            PRU_CTRL.CYCLE = 0;

            // transfer mosi bit
            if ((mosi << bit) & 0x8000) {
                __R30 |= mosiMask;
            }
            else {
                __R30 &= mosiMaskNeg;
            }

            // delay 400ns before up clock
            while ( PRU_CTRL.CYCLE < clockDelayCycles)
                ;

            // clock up
            __R30 |= clkMask;
            // count from up
            PRU_CTRL.CYCLE = 0;

            // read miso bit
            if (__R31 & misoMask) {
                miso |= (uint16_t)(0x01);
            }
            else {
                miso &= ~((uint16_t)(0x01));
            }
        }
        *(status->misoData + pos) = miso;
    }

    // deselect device
    __R30 |= (1 << status->pins.CS);

}
