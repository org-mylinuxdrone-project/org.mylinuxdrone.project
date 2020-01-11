/*
 * main.c
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

#include <resource_table.h>
#include <pru_mpu_6500_spi_lib.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_intc.h>
#include <pru_scratchpad.h>
#include <rcReceiver.h>

#ifdef PRU1_CTRL
#define PRU_CTRL PRU1_CTRL
#else
#define PRU_CTRL PRU0_CTRL
#endif

#define MPU_INT 9 //P8_29
#define IS_MPU_DATA_RDY()       (__R31 & (1 << MPU_INT))
#define MPU_SENSOR_NUM          0
#define COMPASS_SENSOR_NUM      1
#define BAROMETER_SENSOR_NUM    2
#define RC_SENSOR_NUM           3
#define MPU_SENSOR_MASK         (1 << MPU_SENSOR_NUM)
#define COMPASS_SENSOR_MASK     (1 << COMPASS_SENSOR_NUM)
#define BAROMETER_SENSOR_MASK   (1 << BAROMETER_SENSOR_NUM)
#define RC_SENSOR_MASK          (1 << RC_SENSOR_NUM)

#define INT_P0_TO_P1_MASK       (1 << INT_P0_TO_P1)
#define INT_P1_TO_P0_MASK       (1 << INT_P1_TO_P0)

#define IS_ACTIVE(MSK)          (active_sensors & MSK)
#define IS_INT_P0_TO_P1()       (CT_INTC.SECR0_bit.ENA_STS_31_0 & INT_P0_TO_P1_MASK)
#define RECEIVE_MSG_FROM_P0()   {\
                                  (CT_INTC.SICR_bit.STS_CLR_IDX = INT_P0_TO_P1);\
                                  __xin(SP_BANK_0, 3, 0, pru0_data);\
                                }
#define SEND_DATA_TO_P0(MSG_TYPE) {\
                                    pru0_data_struct->message_type = MSG_TYPE;\
                                    __xout(SP_BANK_1, 6, 0, pru0_data);\
                                    (CT_INTC.SRSR0_bit.RAW_STS_31_0 |= (1 << INT_P1_TO_P0));\
                                  }

/*
 * buffer for pru0 communications
 */
unsigned char pru0_data[sizeof(PrbMessageType)] = { '\0' };
PrbMessageType* pru0_data_struct = (PrbMessageType*) pru0_data;

uint32_t active_sensors = 0;

uint16_t CHECK = 0x00;
uint16_t ERROR = 0x00;
uint8_t counter8 = 0;
uint16_t mosiData[32] = { };
uint16_t misoData[32] = { };

int16_t temp_raw = 0;
uint8_t rc_receiver_newData = 0;
int32_t RC_BUFFER[9] = { 0 };

volatile register uint32_t __R31;

inline uint8_t is_enabled_RC()
{
    return active_sensors & (1 << RC_SENSOR_NUM);
}

void enable_RC()
{
    if (!is_enabled_RC())
    {
        rc_receiver_Start();
        active_sensors |= (1 << RC_SENSOR_NUM);
    }
}

void disable_RC()
{
    if (is_enabled_RC())
    {
        rc_receiver_Stop();
        active_sensors &= ~(1 << RC_SENSOR_NUM);
    }
}

inline uint8_t is_enabled_Barometer()
{
    return active_sensors & (1 << BAROMETER_SENSOR_NUM);
}

inline uint8_t is_enabled_Compass()
{
    return active_sensors & (1 << COMPASS_SENSOR_NUM);
}

inline uint8_t is_enabled_MPU()
{
    return active_sensors & (1 << MPU_SENSOR_NUM);
}
void enable_MPU()
{
    if (!is_enabled_MPU())
    {
        active_sensors |= (1 << MPU_SENSOR_NUM);
    }
}

void disable_MPU()
{
    active_sensors &= ~(1 << MPU_SENSOR_NUM);
}

inline void clean_rc_buffer()
{
    for (counter8 = 0; counter8 < 9; counter8++)
    {
        RC_BUFFER[counter8] = 0;
    }
}

uint8_t rc_receiver_Init_result = 0;
/**
 * main.c
 */
int main(void)
{
    // clear interrupt to pru0
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_P1_TO_P0;

    /*
     * Init RC
     * Attenzione! richiede livello di ottimizzazione 'off' per la build.
     * Abilitando anche solo il livello 1, 'edma' non parte.
     * Probabilmente devo inserire qualche sleep nel processo di inizializzazione
     * TODO: verificare
     */
    rc_receiver_Init_result = rc_receiver_Init();

    /*
     * CT_CFG.SYSCFG_bit.STANDBY_INIT : the object is used to write data to
     * the SYSCFG register. Writing 0 to the STANDBY_INIT section of the
     * register opens up the OCP master port that is used for PRU to ARM
     * communication.
     */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    CHECK = 0;
    ERROR = 0;
    while (mpu_6500_spi_testConnection())
    {
        CHECK++;
        ERROR = 1;
    }
    while (mpu_6500_spi_init())
    {
        CHECK++;
        ERROR = 2;
    }
    __delay_cycles(1000);

    if (mpu_6500_spi_calc_gyro_bias(MPU_INT))
    {
        CHECK++;
        ERROR = 3;
    }
    while (1)
    {
        // Rules: ordered by priority
        /* Check for New Data from RC
         * TX not connected <=> rc_receiver_newData & RC_RECEIVER_TX_NOT_PRESENT
         * New Data <=> rc_receiver_newData & RC_RECEIVER_TX_COMPLETE
         * KO <=> rc_receiver_newData = 0
         */
        if (IS_ACTIVE(RC_SENSOR_MASK) && (rc_receiver_newData =
                rc_receiver_PulseNewData()))
        {
            // TODO: inviare l'intero buffer a pru0
            if (rc_receiver_newData & RC_RECEIVER_TX_NOT_PRESENT)
            {
                clean_rc_buffer();
            }
            else if (rc_receiver_newData & RC_RECEIVER_TX_COMPLETE)
            {
                if (rc_receiver_extract_Data(RC_BUFFER))
                {
                    pru0_data_struct->rc.throttle = (RC_BUFFER[2] >> 1)+16383; // range from 0 to 32767
                    pru0_data_struct->rc.yaw = RC_BUFFER[4];
                    pru0_data_struct->rc.pitch = RC_BUFFER[3];
                    pru0_data_struct->rc.roll = RC_BUFFER[1];
                    pru0_data_struct->rc.aux1 = RC_BUFFER[6];
                    pru0_data_struct->rc.aux2 = RC_BUFFER[5];
                    pru0_data_struct->rc.aux3 = RC_BUFFER[7];
                    pru0_data_struct->rc.aux4 = RC_BUFFER[8];
                    SEND_DATA_TO_P0(RC_DATA_MSG_TYPE);
                }
            }
        }
        else if (IS_ACTIVE(MPU_SENSOR_MASK) && IS_MPU_DATA_RDY())
        {
            //TODO: aggiungere temperature nella struttura dati insieme a gyro e accel
            if (mpu_6500_spi_get_data(pru0_data_struct->mpu_accel_gyro_vect.accel, pru0_data_struct->mpu_accel_gyro_vect.gyro, &temp_raw))
            {
                CHECK++;
                ERROR = 4;
                continue;
            }
            SEND_DATA_TO_P0(MPU_DATA_MSG_TYPE);
        }
        else if (IS_INT_P0_TO_P1())
        {
            RECEIVE_MSG_FROM_P0();

            switch (pru0_data_struct->message_type)
            {
            case MPU_ENABLE_MSG_TYPE:
            {
                enable_MPU();
                break;
            }
            case MPU_DISABLE_MSG_TYPE:
            {
                disable_MPU();
                break;
            }
            case RC_ENABLE_MSG_TYPE:
            {
                enable_RC();
                break;
            }
            case RC_DISABLE_MSG_TYPE:
            {
                disable_RC();
                break;
            }
            }
        }
        else if (IS_ACTIVE(BAROMETER_SENSOR_MASK))
        {
            // TODO: read data from baro
            // TODO: send data to pru0
        }
        else if (IS_ACTIVE(COMPASS_SENSOR_MASK))
        {
            // TODO: read data from compass
            // TODO: send data to pru0
        }

    }
}

