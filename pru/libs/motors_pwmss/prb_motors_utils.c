/*
 * prb_motors_utils.c
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */

#include <prb_motors_utils.h>

uint16_t pru_rpmsg_acc_motors_target[4] = { 0 };

/*
 * param: required_accels ha dimensione 4 e rappresenta la richieste di accelerazione per ogni motore.
 *        accelerazione espressa in (m/s^2)*LSB/r
 */
void prb_motors_calculate(uint16_t* required_accels) {
    // TODO: calcolare pru_rpmsg_acc_motors_target
}

uint16_t* prb_motors_get_motors_target() {
    return pru_rpmsg_acc_motors_target;
}
