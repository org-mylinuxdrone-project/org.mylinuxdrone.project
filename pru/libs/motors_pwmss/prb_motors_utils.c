/*
 * prb_motors_utils.c
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */

#include <prb_motors_utils.h>

/*
 * TODO:
 * Input F(i), Ftot(i)
 * Calcolo:
 * - Err(i) = F(i-i) - (Ftot(i)-Ftot(i-1))
 * - MIn0(i) = F(i) + Err(i)
 * - Resp(i) = MOut(i-1)/calcMOut(Ftot(i))   <-- da verificare
 * - MIn(i) = MIn0(i) * Resp(i)
 * - MOut = MOut(i-1) + calcMOut(MIn(i))
 * Invarianti:
 * - |MOut[1](i)-MOut[3](i)| <= 300
 * - |MOut[3](i)-MOut[4](i)| <= 300
 * - |(MOut[3](i)+MOut[4](i)) - (MOut[1](i)-MOut[3](i))| <= 300
 * - |(MOut[3](i)+MOut[4](i)) + (MOut[1](i)+MOut[3](i))| == calcMOut(Throttle(i))
 * Output: MOut(i) nel range Pwm
 *   l'output è inviato direttamente su pwm
 *
 *
 */

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
