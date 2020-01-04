/*
 * mld_pru_cntrl_channel.h
 *
 *  Created on: 08 mag 2019
 *      Author: andrea
 */

#ifndef MLD_PRU_CNTRL_CHANNEL_H_
#define MLD_PRU_CNTRL_CHANNEL_H_


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
    MPU_DESTROY_CHANNEL_MSG_TYPE
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




#endif /* MLD_PRU_CNTRL_CHANNEL_H_ */
