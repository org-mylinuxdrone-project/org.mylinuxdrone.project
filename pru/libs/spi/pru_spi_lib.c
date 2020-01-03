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

#define MOSI    7       //P8_40
#define CLK     4       //P8_41
#define MISO    8       //P8_27
#define CS      10      //P8_28

volatile register uint32_t __R30;
volatile register uint32_t __R31;

uint8_t pru_spi_read8(uint8_t address)
{
    // alzo il bit 15 per 'lettura'
    uint16_t mosi = ((address | 0x80) << 8);
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
    __delay_cycles(60);
    return miso; // scarto primo byte
}

void pru_spi_write8(uint8_t address, uint8_t value) {
    uint16_t mosi = ((address & ~(0x80)) << 8 | value);
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
        __delay_cycles(100);

        // clock up
        __R30 |= 0x10; // 0x10 = 1 << CLK (bit 4)

        __delay_cycles(100);
    }
    // deselect device
    __R30 |= (1 << 10); // 1 << CS (bit 10)
    __delay_cycles(60);
}

void pru_spi_write16(uint8_t address, uint16_t value) {
    uint32_t mosi = (((address & ~(0x80)) << 24) | (value << 8));
    uint32_t counter = 0;

    // reset clock and select device
    __R30 |= (0x10); // 0x10 = 1 << CLK (bit 4)
    __R30 &= ~(1 << 10); // 1 << CS (bit 10)

    // transfer 24 bits
    for (counter = 0x80000000; counter != 0x80; counter = counter >> 1)
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
        __delay_cycles(100);

        // clock up
        __R30 |= 0x10; // 0x10 = 1 << CLK (bit 4)

        __delay_cycles(100);
    }
    // deselect device
    __R30 |= (1 << 10); // 1 << CS (bit 10)
    __delay_cycles(60);
}

uint16_t pru_spi_read16(uint8_t address)
{
    // alzo il bit 31 per 'lettura'
    uint32_t mosi = (address | 0x80);
    mosi = (mosi << 24) & 0xFF000000;

    //Set the CFG Register to direct output instead of serial output
    CT_CFG.GPCFG1 = 0;


    uint32_t miso = 0;
    uint32_t counter = 0;

    // reset clock and select device
    __R30 |= (0x10); // 0x10 = 1 << CLK (bit 4)
    __R30 &= ~(1 << 10); // 1 << CS (bit 10)

    // transfer byte
    for (counter = 0x80000000; counter != 0; counter = counter >> 1)
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
    __delay_cycles(60);
    return miso; // scarto primi due byte
}

int8_t pru_spi_readData(uint16_t* mosiData, uint16_t* misoData, uint16_t length)
{
    // spi read operation: alzo il bit 15 del primo mosiData (address)
    mosiData[0] = (mosiData[0]|0x8000);

    //Set the CFG Register to direct output instead of serial output
    CT_CFG.GPCFG1 = 0;

    uint16_t pos = 0;
    uint8_t bit = 0;
    uint16_t mosi = 0;
    uint16_t miso = 0;

    // reset clock and select device
    __R30 |= (0x10); // 0x10 = 1 << CLK (bit 4)
    __R30 &= ~(1 << 10); // 1 << CS (bit 10)

    // transfer bytes (MSB first, SPI Mode 3 (cpha=1, cpol=1))
    for (pos = 0; pos < length; pos++)
    {
        mosi = *(mosiData + pos);

        // transfer byte
        for (bit = 0; bit < 16; bit++)
        {
            if(bit != 0) __delay_cycles(15);

            miso = miso << 1;

            // clock down
            __R30 &= ~(0x10); // 0x10 = 1 << CLK (bit 4)

            // transfer mosi bit
            if ((mosi << bit) & 0x8000) {
                __R30 |= 0x80;
            }
            else {
                __R30 &= ~(0x80);
            }

            // delay 400ns before up clock
            __delay_cycles(15);

            // read miso bit
            if (__R31 & 0x100) {
                miso |= (uint16_t)(0x01);
            }
            else {
                miso &= ~((uint16_t)(0x01));
            }

            // clock up
            __R30 |= 0x10; // 0x10 = 1 << CLK (bit 4)

        }
        *(misoData + pos) = (pos == 0 ? (miso & 0x00FF) : miso); // scarto il primo byte deiprimi 16 bits.
    }

    // deselect device
    __R30 |= (1 << 10); // 1 << CS (bit 10)
    __delay_cycles(60);
    return 0;
}

