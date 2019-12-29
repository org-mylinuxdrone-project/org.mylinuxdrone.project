/*
 * pru_spi_lib.h
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

#ifndef PRU_SPI_LIB_H_
#define PRU_SPI_LIB_H_

#include <stdint.h>
#define PRU_1MHZ_CPU_CYCLES 90
#define PRU_20MHZ_CPU_CYCLES 5

uint16_t pru_spi_read16(uint16_t status);
uint32_t pru_spi_read32(uint32_t status);
void pru_spi_transferData(uint16_t* mosiData, uint16_t* misoData, uint16_t length);


#endif /* PRU_SPI_LIB_H_ */
