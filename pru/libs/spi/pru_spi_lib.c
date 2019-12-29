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
#include <pru_cfg.h>

volatile register uint32_t __R30;
volatile register uint32_t __R31;

uint16_t pru_spi_read16(uint16_t mosi)
{
    //Set the CFG Register to direct output instead of serial output
    CT_CFG.GPCFG1 = 0;


    uint16_t miso = 0;
    uint16_t counter = 0;

    // reset clock and select device
    __R30 |= (0x10); // 0x10 = 1 << CLK (bit 4)
    __R30 &= ~(1 << 10); // 1 << CS (bit 10)

    // transfer byte
    for (counter = 0x8000; counter != 0; counter = counter >> 1)
    {

        // clock down
        __R30 &= ~(0x10); // 0x10 = 1 << CLK (bit 4)

        // transfer mosi bit
        if ((mosi & counter))
        {
            __R30 |= 0x80;
        }
        else
        {
            __R30 &= ~(0x80);
        }
        __delay_cycles(5);

        // read miso bit
        if (__R31 & 0x100) // 0x100 = 1 << MISO (bit 7)
        {
            miso |= counter;
        }
        else
        {
            miso &= ~(counter);
        }

        // clock up
        __R30 |= 0x10; // 0x10 = 1 << CLK (bit 4)

        __delay_cycles(5);
    }
    // deselect device
    __R30 |= (1 << 10); // 1 << CS (bit 10)
    __delay_cycles(50);
    return miso;
}
