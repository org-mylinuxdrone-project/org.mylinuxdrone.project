/*
 * pru_scratchpad.h
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
#ifndef PRU_SCRATCHPAD_H_
#define PRU_SCRATCHPAD_H_

#define INT_P1_TO_P0            18
#define INT_P0_TO_P1            19

/*
 * Register R31 values to generate interrupt
 * R31_P0_TO_P1 : The value that when written to R31 register will
 *              generate INT_P1_TO_P0 interrupt
 */
#define R31_P0_TO_P1    (1<<5) | (INT_P0_TO_P1 - 16) // TODO: Togliere. non usato
#define R31_P1_TO_P0    (1<<5) | (INT_P1_TO_P0 - 16) // TODO: Togliere. non usato

/*
 * HOST_PRU0_TO_PRU1_CB : The check bit to check HOST_PRU0_TO_PRU1 interrupt
 *              Since the value of HOST_PRU0_TO_PRU1 is 1, HOST_PRU0_TO_PRU1_CB
 *              has the value 31
 */
#define HOST_PRU0_TO_PRU1_CB    31 // TODO: Togliere. non usato. host channel 1 mapped on bit 31 of R31

/* Address of the external peripherals
 * SHARED_MEM_ADDR : Absolute local address of the 12 KB shared RAM that will be
 *              used to communicate sampling configuration data between the two
 *              PRUs
 */
#define SHARED_MEM_ADDR 0x00010000

/*
 * Scratch Pad Bank IDs
 * The scratch pad inside the PRU-ICSS has 3 banks. Each bank has different
 * ID numbers.
 * SP_BANK_0 : ID number for scratch pad bank 0
 * SP_BANK_1 : ID number for scratch pad bank 1
 * SP_BANK_2 : ID number for scratch pad bank 2
 */
#define SP_BANK_0       10
#define SP_BANK_1       11
#define SP_BANK_2       12

enum message_types {
    MPU_DATA_MSG_TYPE = 0,
    COMPASS_DATA_MSG_TYPE,
    BAROMETER_DATA_MSG_TYPE,
    MPU_CONFIG_MSG_TYPE,
    MOTORS_DATA_MSG_TYPE,
    RC_DATA_MSG_TYPE,
    MPU_ENABLE_MSG_TYPE,
    COMPASS_ENABLE_MSG_TYPE,
    BAROMETER_ENABLE_MSG_TYPE,
    MOTORS_ENABLE_MSG_TYPE,
    MPU_DISABLE_MSG_TYPE,
    COMPASS_DISABLE_MSG_TYPE,
    BAROMETER_DISABLE_MSG_TYPE,
    MOTORS_DISABLE_MSG_TYPE,
    RC_ENABLE_MSG_TYPE,
    RC_DISABLE_MSG_TYPE,
    MPU_CREATE_CHANNEL_MSG_TYPE,
    MPU_DESTROY_CHANNEL_MSG_TYPE,
    RC_CREATE_CHANNEL_MSG_TYPE,
    RC_DESTROY_CHANNEL_MSG_TYPE
};

typedef struct
{
    uint32_t message_type;
    union
    {
        struct {
            uint16_t throttle;
            int16_t yaw;
            int16_t pitch;
            int16_t roll;
            int16_t aux1;
            int16_t aux2;
            int16_t aux3;
            int16_t aux4;
        } rc;
        struct {
            uint16_t m1;
            uint16_t m2;
            uint16_t m3;
            uint16_t m4;
        } motors;
        struct
        {
            uint16_t m[4];
        } motors_vect;
        struct
        {
            int16_t accel[3];
            int16_t gyro[3];
        } mpu_accel_gyro_vect;
        struct
        {
            int16_t value[6];
        } mpu_accel_gyro_single_vect;
        struct
        {
            int16_t ax;
            int16_t ay;
            int16_t az;
            int16_t gx;
            int16_t gy;
            int16_t gz;
        } mpu_accel_gyro;
        struct
        {
            uint8_t gyro_scale;
            uint8_t accel_scale;
            uint16_t frequency_hz;
            uint16_t gyro_offset[3];
            uint16_t accel_offset[3];
        } MpuConfMessage;
    };
} PrbMessageType;

#endif /* PRU_SCRATCHPAD_H_ */
