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

typedef struct  {
    uint16_t mosiData;
    uint16_t misoData;
} PruSpiStatus;

uint16_t pru_spi_read16(uint16_t status);



#endif /* PRU_SPI_LIB_H_ */
