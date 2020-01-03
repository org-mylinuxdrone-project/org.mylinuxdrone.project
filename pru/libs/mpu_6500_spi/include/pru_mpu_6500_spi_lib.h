/*
 * mpu_6500_spi.h
 *  Created on: 30 dic 2019
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
#ifndef PRU_MPU_6500_SPI_LIB_H_
#define PRU_MPU_6500_SPI_LIB_H_
#include <stdint.h>

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

/*
 * Initialize Mpu
 * return 0 if ok, else -1
 */
int8_t mpu_6500_spi_init();

/*
 * Read Whoami register and verify the value at 0x70
 * return -1 if check fail, 0 if ok
 */
int8_t mpu_6500_spi_testConnection();
int8_t mpu_6500_spi_read_register(uint8_t address);
int8_t mpu_6500_spi_loadOffsets();
int8_t mpu_6500_spi_loadConfigurations();

int8_t mpu_6500_spi_get_data(int16_t* acc, int16_t* gyro, int16_t* temp);
int8_t mpu_6500_spi_calc_offsets(uint8_t interruptPin);

int8_t mpu_6500_spi_start();
int8_t mpu_6500_spi_stop();



#endif /* PRU_MPU_6500_SPI_LIB_H_ */
