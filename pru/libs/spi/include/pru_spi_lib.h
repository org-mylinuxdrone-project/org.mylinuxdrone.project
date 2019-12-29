/*
 * pru_spi_lib.h
 *
 *  Created on: 27 dic 2019
 *      Author: andrea
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
