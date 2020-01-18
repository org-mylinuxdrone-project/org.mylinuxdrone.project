/*
 * mld_pru_pid_channel.h
 *  Created on: 12 gen 2020
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
#ifndef MLD_PRU_PID_CHANNEL_H_
#define MLD_PRU_PID_CHANNEL_H_

#include "mylinuxdrone.h"

enum mld_pid_device_status_enum {
    MLD_PID_DEVICE_STATUS_ENABLED,
    MLD_PID_DEVICE_STATUS_STARTED,
    MLD_PID_DEVICE_STATUS_STOPPED,
    MLD_PID_DEVICE_STATUS_CALIBRATION_STARTING,
    MLD_PID_DEVICE_STATUS_CALIBRATING,
    MLD_PID_DEVICE_STATUS_CALIBRATION_STOPPING,
    MLD_PID_DEVICE_STATUS_ERROR
};

struct mld_pid_device_chan_config {
    uint16_t ke;
    uint16_t ki;
    uint16_t kd;
    uint16_t yke;
    uint16_t yki;
    uint16_t ykd;
    uint8_t kgyro; // filtro passa alto gyro: gyro[i] = k*gyro[i-1] + (1-k)*gyro[i];
    uint8_t mas;   // numero di campioni media accel
};

struct mld_pid_device_data {
    int16_t F[4];      // LSB motori
    int16_t MErr[4];   // FixPoint 10 bit
    int16_t MIErr[4];  // FixPoint 10 bit
    int16_t MDErr[4];  // FixPoint 10 bit
    int16_t M[4];      // Momenti [Y,P,R,T]
};

struct mld_pid_device_status {
    enum mld_pid_device_status_enum status;
    struct mld_pid_device_chan_config config;
    struct mld_pid_device_data data;
};

struct mld_pid_device {
    struct mylinuxdrone_device dev;
    struct mld_pid_device_status status;
};
#define to_mld_pid_driver(d) container_of(d, struct pru_pid_driver, driver)
#define to_mld_pid_device(d) container_of(d, struct mld_pid_device, dev)

#endif /* MLD_PRU_PID_CHANNEL_H_ */
