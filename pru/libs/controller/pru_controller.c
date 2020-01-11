/*
 * pru_controller.c
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

#include <pru_controller.h>


/*
 * +------------+
 * |Axis        |
 * +------------+
 * |   Z  Y     |
 * |   | /      |
 * |   |/       |
 * |   /____ X  |
 * +------------+
*  +--------------+
*  | Motors       |
*  +--------------+
*  | 3(cw) 1(ccw) |
*  |   \  /       |
*  |    \/        |
*  |    /\        |
*  |   /  \       |
*  | 2(ccw)4(cw)  |
*  +--------------+
*
*    +-----------+
*    |Dir.Accel  |
*    +--+--+--+--+
*    | 1| 2| 3| 4|
*    +--+--+--+--+--+
*    |-1|-1| 1| 1| Y|
* A= |-1| 1|-1| 1| P|
*    |-1| 1| 1|-1| R|
*    | 1| 1| 1| 1| T|
*    +--+--+--+--+--+
*/

/* TODO:
 * Input: RC(i) in gradi, Accel, Gyro
 *        I dati devono essere forniti in scala e già rispetto agli assi definiti (matrici direzione già applicate)
 * Stato: F, M, MErr, MIErr, MDErr, Throttle (F, MErr* inizializzati a zero, M inizializzato a [Gyro, Throttle])
 * Parametri: ke, ki, kd
 * Calcolo:
 * - MErr(i)=M(i-1)-[Gyro(i), Throttle(i-1)]
 * - MIErr(i)=MIErr(i-1) + MIErr(i)
 * - MDErr(i)=(MErr(i) - MErr(i-1))*Freq (1KHz)
 * - M(i)=ke(RC(i)-[Gyro(i), Throttle(i-1)]+ke*MErr(i)) + ki*MIErr(i)+ kd*MDErr(i)
 * - F(i)=F(i-i) + inv(A)*M(i)
 * Output:F(i)
 *   l'output è inviato direttamente al controller motori.
 */
int16_t M[4] = {0}; // sostituito
int16_t F[4] = {0}; // accumulato
int16_t MErr[4] = {0};    // sostituito
int16_t MIErr[4] = {0};   // accumulato
int16_t MDErr[4] = {0};   // sostituito
uint16_t ke = 0x0000;    // fix point 8 bits (0x0100 corrisponde a 1)
uint16_t ki = 0x0000;    // fix point 8 bits
uint16_t kd = 0x0000;    // fix point 8 bits
int16_t throttlePrev = 0;
int8_t invA[4][4] = {
                    {-1, -1, -1, 1 },
                    {-1,  1,  1, 1 },
                    { 1, -1,  1, 1 },
                    { 1,  1, -1, 1 },
};
int16_t F1 = 0;
int16_t F2 = 0;
int16_t F3 = 0;
int16_t F4 = 0;

int16_t M1 = 0;
int16_t M2 = 0;
int16_t M3 = 0;
int16_t M4 = 0;
int16_t gyroPrev[3] = {0};

void pru_controller_apply(int16_t* rc, int16_t* accel, int16_t* gyro) {
    uint8_t i = 0;
    uint8_t j = 0;
    for(i = 0; i < 3; i++) {
        int16_t prevErr = MErr[i];
        MErr[i] = LIMIT(M[i] - (gyro[i] - gyroPrev[i]), 1000, -1000);
        MDErr[i] = MErr[i] - prevErr;
        MIErr[i] += MErr[i]; // <---TODO: Gestire limiti max, min
        M[i] = (rc[i] - gyro[i]) + ((ke*MErr[i]) >> 8)+((ki*MIErr[i]) >> 8) + ((kd*MDErr[i]) >> 8);
        gyroPrev[i] = gyro[i];
    }

    /* TODO:
     * Per ora non verifico l'accelerazione verticale
     * sarà implementazione successiva
     */
    MErr[POS_THROTTLE] = 0;
    MDErr[POS_THROTTLE] = 0;
    MIErr[POS_THROTTLE] = 0;
    M[POS_THROTTLE] = rc[POS_THROTTLE];
//    M[POS_THROTTLE] = rc[POS_THROTTLE] - throttlePrev;
//    throttlePrev = rc[POS_THROTTLE];

    for(i = 0; i < 4; i++) {
        F[i] = 0;
        for(j = 0; j < 4; j++) {
            // invA corrisponde a 4*A^(-1) per questo >> 2
            F[i] += ((invA[i][j]*M[j]) >> 2);
        }
    }
    F1 = F[0];
    F2 = F[1];
    F3 = F[2];
    F4 = F[3];

    M1 = M[0];
    M2 = M[1];
    M3 = M[2];
    M4 = M[3];

    /* TODO:
     * Inviare F al controller motori
     */
}
