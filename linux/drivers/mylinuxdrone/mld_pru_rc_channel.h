/*
 * mld_pru_rc_channel.h
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
#ifndef MLD_PRU_RC_CHANNEL_H_
#define MLD_PRU_RC_CHANNEL_H_

#include "mylinuxdrone.h"

enum mld_rc_device_status_enum {
    MLD_RC_DEVICE_STATUS_ENABLED,
    MLD_RC_DEVICE_STATUS_STARTED,
    MLD_RC_DEVICE_STATUS_STOPPED,
    MLD_RC_DEVICE_STATUS_CALIBRATION_STARTING,
    MLD_RC_DEVICE_STATUS_CALIBRATING,
    MLD_RC_DEVICE_STATUS_CALIBRATION_STOPPING,
    MLD_RC_DEVICE_STATUS_ERROR
};

struct mld_rc_device_chan_config {
    int16_t rawMin;
    int16_t rawCenter;
    int16_t rawMax;
    int16_t min;
    int16_t max;
    uint16_t radius;
    uint16_t factor; // fixpoint 10 bit
};

struct mld_rc_device_data {
    int16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t aux1;
    int16_t aux2;
    int16_t aux3;
    int16_t aux4;
};

struct mld_rc_device_status {
    enum mld_rc_device_status_enum status;
    struct mld_rc_device_chan_config config[8];
    struct mld_rc_device_data data;
};

struct mld_rc_device {
    struct mylinuxdrone_device dev;
    struct mld_rc_device_status status;
};
#define to_mld_rc_driver(d) container_of(d, struct pru_rc_driver, driver)
#define to_mld_rc_device(d) container_of(d, struct mld_rc_device, dev)

#endif /* MLD_PRU_RC_CHANNEL_H_ */
