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
    volatile struct {
        uint8_t MOSI;
        uint8_t MISO;
        uint8_t CLK;
        uint8_t CS;
    } pins;
    uint8_t clockDelayCycles;
    uint8_t* mosiData;
    uint8_t* misoData;
} PruSpiStatus;

void pru_spi_transferData(PruSpiStatus* status, uint16_t bytes);
inline void pru_spi_DelayCycles(uint8_t cycles);



#endif /* PRU_SPI_LIB_H_ */
