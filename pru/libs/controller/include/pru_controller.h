/*
 * pru_controller.h
 *  Created on: 06 gen 2020
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
#ifndef PRU_CONTROLLER_H_
#define PRU_CONTROLLER_H_
#include <stdint.h>

#ifndef POS_YAW
#define POS_YAW      0
#endif
#ifndef POS_PITCH
#define POS_PITCH    1
#endif
#ifndef POS_ROLL
#define POS_ROLL     2
#endif
#ifndef POS_THROTTLE
#define POS_THROTTLE 3
#endif
#ifndef POS_AUX1
#define POS_AUX1 4
#endif
#ifndef POS_AUX2
#define POS_AUX2 5
#endif
#ifndef POS_AUX3
#define POS_AUX3 6
#endif
#ifndef POS_AUX4
#define POS_AUX4 7
#endif


#ifndef MAX
#define MAX(A,B)                              (A > B ? A : B)
#endif
#ifndef MIN
#define MIN(A,B)                              (A < B ? A : B)
#endif
#ifndef LIMIT
#define LIMIT(V,MX,MN)                        (MAX((MN),MIN((V),(MX))))
#endif

struct pru_controller_status {
    int16_t F[4];      // LSB motori
    int16_t MErr[4];   // FixPoint 10 bit
    int16_t MIErr[4];  // FixPoint 10 bit
    int16_t MDErr[4];  // FixPoint 10 bit
    int16_t M[4];      // Momenti [Y,P,R,T]
};
struct pru_controller_config {
    uint16_t ke;
    uint16_t ki;
    uint16_t kd;
    uint16_t yke;
    uint16_t yki;
    uint16_t ykd;
    uint8_t kgyro; // filtro passa alto gyro: gyro[i] = kgyro*gyro[i-1] + (1-kgyro)*gyro[i];
    uint8_t mas;   // numero di campioni media accel
};


void pru_controller_apply(int16_t* rc, int16_t* accel, int16_t* gyro);
struct pru_controller_status* pru_controller_get_status();
struct pru_controller_config* pru_controller_get_config();
void pru_controller_init();
void pru_controller_enable();
void pru_controller_disable();
uint8_t pru_controller_is_enabled();
#endif /* PRU_CONTROLLER_H_ */
