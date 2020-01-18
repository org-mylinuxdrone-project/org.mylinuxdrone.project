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
 * Input: RC(i), Accel, Gyro tutti in LSB
 *        I dati devono essere forniti in scala e già rispetto agli assi definiti (matrici direzione già applicate)
 * Stato: F, M, MErr, MIErr, MDErr, Throttle
 * Parametri: ke, ki, kd
 * Calcolo:
 * - MErr(i)=Throttle(i)- Gyro(i)
 * - MIErr(i)=MIErr(i-1) + MIErr(i)
 * - MDErr(i)=(MErr(i) - MErr(i-1))*Freq (1KHz)
 * - M(i)=+ke*MErr(i)) + ki*MIErr(i)+ kd*MDErr(i)
 * - F(i)=inv(A)*M(i)
 * Output:F(i)
 *   l'output è inviato direttamente al controller motori.
 */
struct pru_controller_status pru_controller_status_val = {
                                                          {0, 0, 0, 0}, // F
                                                          {0, 0, 0, 0}, // MErr
                                                          {0, 0, 0, 0}, // MIErr
                                                          {0, 0, 0, 0}, // MDErr
                                                          {0, 0, 0, 0}, // M
};
uint16_t ke = 0x0080;    // fix point 8 bits (0x0100 corrisponde a 1)
uint16_t ki = 0x0000;    // fix point 8 bits
uint16_t kd = 0x0000;    // fix point 8 bits
uint16_t yke = 0x0080;    // fix point 8 bits (0x0100 corrisponde a 1)
uint16_t yki = 0x0000;    // fix point 8 bits
uint16_t ykd = 0x0000;    // fix point 8 bits

int8_t invA[4][4] = {
                    {-1, -1, -1, 1 },
                    {-1,  1,  1, 1 },
                    { 1, -1,  1, 1 },
                    { 1,  1, -1, 1 }
};
struct pru_controller_status* pru_controller_get_status() {
    return &pru_controller_status_val;
}
void pru_controller_apply(int16_t* rc, int16_t* accel, int16_t* gyro) {
    uint8_t i = 0;
    uint8_t j = 0;
    int16_t prevErr = 0;

    prevErr = pru_controller_status_val.MErr[POS_YAW];
    pru_controller_status_val.MErr[POS_YAW] = rc[POS_YAW] - gyro[POS_YAW];
    pru_controller_status_val.MDErr[POS_YAW] = pru_controller_status_val.MErr[POS_YAW] - prevErr;
    pru_controller_status_val.MIErr[POS_YAW] += pru_controller_status_val.MErr[POS_YAW]; // <---TODO: Gestire limiti max, min
    pru_controller_status_val.M[POS_YAW] = ((yke*pru_controller_status_val.MErr[POS_YAW]) >> 8)+
                                           ((yki*pru_controller_status_val.MIErr[POS_YAW]) >> 8) +
                                           ((ykd*pru_controller_status_val.MDErr[POS_YAW]) >> 8);

    prevErr = pru_controller_status_val.MErr[POS_PITCH];
    pru_controller_status_val.MErr[POS_PITCH] = rc[POS_PITCH] - gyro[POS_PITCH];
    pru_controller_status_val.MDErr[POS_PITCH] = pru_controller_status_val.MErr[POS_PITCH] - prevErr;
    pru_controller_status_val.MIErr[POS_PITCH] += pru_controller_status_val.MErr[POS_PITCH]; // <---TODO: Gestire limiti max, min
    pru_controller_status_val.M[POS_PITCH] = ((ke*pru_controller_status_val.MErr[POS_PITCH]) >> 8)+
                                             ((ki*pru_controller_status_val.MIErr[POS_PITCH]) >> 8) +
                                             ((kd*pru_controller_status_val.MDErr[POS_PITCH]) >> 8);

    prevErr = pru_controller_status_val.MErr[POS_ROLL];
    pru_controller_status_val.MErr[POS_ROLL] = rc[POS_ROLL] - gyro[POS_ROLL];
    pru_controller_status_val.MDErr[POS_ROLL] = pru_controller_status_val.MErr[POS_ROLL] - prevErr;
    pru_controller_status_val.MIErr[POS_ROLL] += pru_controller_status_val.MErr[POS_ROLL]; // <---TODO: Gestire limiti max, min
    pru_controller_status_val.M[POS_ROLL] = ((ke*pru_controller_status_val.MErr[POS_ROLL]) >> 8)+
                                            ((ki*pru_controller_status_val.MIErr[POS_ROLL]) >> 8) +
                                            ((kd*pru_controller_status_val.MDErr[POS_ROLL]) >> 8);

    /* TODO:
     * Per ora non verifico l'accelerazione verticale
     * sarà implementazione successiva
     */
    pru_controller_status_val.MErr[POS_THROTTLE] = 0;
    pru_controller_status_val.MDErr[POS_THROTTLE] = 0;
    pru_controller_status_val.MIErr[POS_THROTTLE] = 0;
    pru_controller_status_val.M[POS_THROTTLE] = rc[POS_THROTTLE];
    for(i = 0; i < 4; i++) {
        pru_controller_status_val.F[i] = 0;
        for(j = 0; j < 4; j++) {
            // invA corrisponde a 4*A^(-1) per questo >> 2
            pru_controller_status_val.F[i] += ((invA[i][j]*pru_controller_status_val.M[j]) >> 2);
        }
        pru_controller_status_val.F[i] = MAX(0, pru_controller_status_val.F[i]); // non possono essere negative
    }

    /* TODO:
     * Inviare F al controller motori
     */
}
