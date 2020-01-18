#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <resource_table.h>
#include <pru_rpmsg.h>
#include <prb_pwmss.h>
#include <prb_motors_utils.h>
#include <pru_controller.h>

#define VIRTIO_CONFIG_S_DRIVER_OK  4
#define PWMSS_DEVICE_FRONT 0
#define PWMSS_DEVICE_REAR 2

volatile register uint32_t __R30;
volatile register uint32_t __R31;

struct pru_rpmsg_transport transport;
unsigned short src, dst, len, src_mpu_channel, src_rc_channel;
unsigned char received_arm_data[sizeof(PrbMessageType)] = { '\0' };
unsigned char received_pru1_data[sizeof(PrbMessageType)] = { '\0' };
PrbMessageType* received_pru1_data_struct = (PrbMessageType*) received_pru1_data;
PrbMessageType* received_arm_data_struct = (PrbMessageType*) received_arm_data;
unsigned char pru_rpmsg_config_data[sizeof(PrbConfigMessageType)] = { '\0' };
PrbConfigMessageType* pru_rpmsg_config_data_struct = (PrbConfigMessageType*) pru_rpmsg_config_data;
struct pru_controller_status* pru_rpmsg_pid_values;

int counter32 = 0;
uint8_t pru_rpmsg_imu_data_changed_flag = 0;
uint8_t pru_rpmsg_rc_data_changed_flag = 0;

int16_t pru_rpmsg_accel[3] = { 0 };
int16_t pru_rpmsg_gyro[3] = { 0 };
int16_t pru_rpmsg_rc[8] = { 0 };

/*******************************************************************************
 * RC Definitions
 *******************************************************************************/
typedef struct
{
    int16_t rawMin;
    int16_t rawCenter;
    int16_t rawMax;
    int16_t min;
    int16_t max;
    int16_t radius;
    uint16_t factor;
} rc_receiver_chan_def_struct;

uint8_t pru_rpmsg_rc_status_calibration = 0;
uint8_t pru_rpmsg_counter8 = 0;
int16_t pru_rpmsg_rc_chan_temp = 0;

rc_receiver_chan_def_struct rc_receiver_chan_def[8] = {
// TODO: configurazione di default (riprendere dai dati rilevati in bbb)
        { 1131, 1878, 2630, 1131, 2625, 747, 44918 }, // roll
        { 1084, 1886, 2631, 1141, 2631, 745, 45038 }, // throttle
        { 1128, 1842, 2519, 1165, 2519, 677, 49562 }, // pitch
        { 1082, 1844, 2598, 1090, 2598, 754, 44501 }, // yaw
        { 1081, 1855, 2629, 1081, 2629, 774, 43351 }, // aux2
        { 1081, 1855, 2629, 1081, 2629, 774, 43351 }, // aux1
        { 1081, 1855, 2629, 1081, 2629, 774, 43351 }, // aux3
        { 1081, 1855, 2629, 1081, 2629, 774, 43351 }  // aux4
};
/*******************************************************************************
 * END of RC Definitions
 *******************************************************************************/

/*******************************************************************************
 * PID Definitions
 *******************************************************************************/

static void prb_init_buffers()
{
    for (counter32 = 0; counter32 < sizeof(PrbMessageType); counter32++)
    {
        received_arm_data[counter32] = '\0';
        received_pru1_data[counter32] = '\0';
    }
}

static uint8_t prb_init_rpmsg()
{
    volatile unsigned char *status;

    /* Clear the status of the registers that will be used in this programs
     * As they have been un-serviced in the last software tun
     */
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_ARM_TO_P0;
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_P1_TO_P0;

    /*
     * Register initialization
     *      Writing to an INTC register or CGF register can be done with
     *      the help of predefined structures. These structures are :
     *      CT_INTC and CT_CFG. Writing to these registers basically
     *      accesse the peripheral through constant-table and loads the
     *      indicated register with appropriate value.
     *
     * CT_CFG.SYSCFG_bit.STANDBY_INIT : the object is used to write data to
     * the SYSCFG register. Writing 0 to the STANDBY_INIT section of the
     * register opens up the OCP master port that is used for PRU to ARM
     * communication.
     *
     * CT_INTC.EISR_bit.EN_SET_IDX : the object is used to write data to the
     * EISR register. Writing an index number to the EN_SET_IDX section of the
     * register results in enabling of that interrupt.
     */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
    CT_INTC.EISR_bit.EN_SET_IDX = INT_P1_TO_P0;
    CT_INTC.EISR_bit.EN_SET_IDX = INT_P0_TO_P1;

    /* Make sure the Linux drivers are ready for RPMsg communication */
    /* this is another place where a hang could occur */
    status = &resourceTable.rpmsg_vdev.status;
    while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK))
        ;

    /* Initialize the RPMsg transport structure */
    /* this function is defined in rpmsg_pru.c.  It's sole purpose is to call pru_virtqueue_init twice (once for
     vring0 and once for vring1).  pru_virtqueue_init is defined in pru_virtqueue.c.  It's sole purpose is to
     call vring_init.  Not sure yet where that's defined, but it appears to be part of the pru_rpmsg iface.*/
    /* should probably test for RPMSG_SUCCESS.  If not, then the interface is not working and code should halt */
    pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0,
                   &resourceTable.rpmsg_vring1, INT_P0_TO_ARM, INT_ARM_TO_P0);

    /* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
    // In a real-time environment, rather than waiting forever, this can probably be run loop-after-loop
    // until success is achieved.  At that point, set a flag and then enable the send/receive functionality
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, RPMSG_CHAN_NAME,
    RPMSG_CHAN_DESC,
                             RPMSG_CHAN_PORT) != PRU_RPMSG_SUCCESS)
        ;
    return 0;
}

/**
 * main.c
 */
int main(void)
{
    /*
     * Clear interrupt to pru1
     */
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_P0_TO_P1;

    /*
     * fill payload for received messages to null
     */
    prb_init_buffers();

    /*
     * Init rpmsg and create pru-mylinuxdrone channel
     */
    prb_init_rpmsg();

    pru_pwmss_lib_Init(PWMSS_DEVICE_FRONT);
    pru_pwmss_lib_Init(PWMSS_DEVICE_REAR);

    pru_pwmss_lib_Start(PWMSS_DEVICE_FRONT);
    pru_pwmss_lib_Start(PWMSS_DEVICE_REAR);

    src_mpu_channel = 0;
    src_rc_channel = 0;

    while (1)
    {
        // receive data from PRU1
        if (CT_INTC.SECR0_bit.ENA_STS_31_0 & (1 << INT_P1_TO_P0))
        {
            CT_INTC.SICR_bit.STS_CLR_IDX = INT_P1_TO_P0;
            __xin(SP_BANK_1, 6, 0, received_pru1_data);
            switch (received_pru1_data_struct->message_type)
            {
            case MPU_DATA_MSG_TYPE:
            {
                /* TODO:
                 * 1) Calcolare inv(A)*M*k=F (requested_accels)
                 * 2) Calcolare risposta motori (implementare in prb_motors fornendo le accelerazioni rilevate)
                 * 3) Calcolare uscita Pwmss (con prb_motors_calculate(required_accels detected_accels))
                 * 4) Inviare messaggio con RC, IMU, Motors, etc?
                 *    Questo permette di avere un unico driver su linux che riceve tutti i dati del controller su un unico
                 *    channel a frequenza 1KHz
                 */

                /*
                 * Esempio di calcolo per motori
                 * da implementare in prb_motors_utils::prb_motors_calculate
                 */
                // TODO: calcolare effetto sui motori dai dati mpu ed rc
//                received_pru1_data_struct->message_type = MOTORS_DATA_MSG_TYPE;
//                uint16_t* pru_rpmsg_motors_ptr = prb_motors_get_motors_target();
//
//                pru_pwmss_lib_SetDuty(
//                        PWMSS_DEVICE_FRONT,
//                        pru_rpmsg_motors_ptr[PRB_MOTORS_FRONT_LEFT - 1],
//                        pru_rpmsg_motors_ptr[PRB_MOTORS_FRONT_RIGHT - 1]);
//                pru_pwmss_lib_SetDuty(
//                        PWMSS_DEVICE_REAR,
//                        pru_rpmsg_motors_ptr[PRB_MOTORS_REAR_LEFT - 1],
//                        pru_rpmsg_motors_ptr[PRB_MOTORS_REAR_RIGHT - 1]);
                /*
                 * Map accel/gyro sensor data to accel/gyro pid data
                 * The accel/gyro sensor data have an inverted order respect to pid data.
                 * from (x, y, z) to (yaw, pitch, roll)
                 */
                for (counter32 = 0; counter32 < 3; counter32++)
                {
                    pru_rpmsg_accel[2 - counter32] =
                            received_pru1_data_struct->mpu_accel_gyro_vect.accel[counter32];
                    pru_rpmsg_gyro[2 - counter32] =
                            received_pru1_data_struct->mpu_accel_gyro_vect.gyro[counter32];
                }

                pru_controller_apply(pru_rpmsg_rc, pru_rpmsg_accel,
                                     pru_rpmsg_gyro);

                /*TODO:
                 * - Quando inviare i dati Pid? (con quale frequenza?)
                 * - Gestire l'attivazione/disattivazione del PID
                 */
                pru_rpmsg_pid_values = pru_controller_get_status();

                // send data from PRU1 to ARM
                if (src_mpu_channel != 0)
                {
                    pru_rpmsg_send(&transport, RPMSG_MPU_CHAN_PORT,
                                   src_mpu_channel, received_pru1_data,
                                   sizeof(PrbMessageType));
                }
                break;
            } // end case MPU_DATA_MSG_TYPE
            case RC_DATA_MSG_TYPE:
            {
                /*
                 * TODO: mettere in scala [-32768, +32767]
                 * - In configurazione:
                 *   - Ogni channel è definito da:
                 *    - rawMin,
                 *    - center,
                 *    - rawMax,
                 *    - min = center - radius,
                 *    - max = rawMax,
                 *    - radius = min(|center - rawMin|, |rawMax - center|),
                 *    - factor = (65535*2^10)/(2*radius[chan]) )
                 * - Il valore rilevato viene trasformato in:
                 *   - scaledValue = (factor*CENTER(LIMIT(min, max, rawValue), center)) >> 10
                 */
                if (pru_rpmsg_rc_status_calibration == 0)
                {
                    // scale rc values
                    for (pru_rpmsg_counter8 = 0; pru_rpmsg_counter8 < 8;
                            pru_rpmsg_counter8++)
                    {
                        received_pru1_data_struct->rc_array.chan[pru_rpmsg_counter8] =
                                (rc_receiver_chan_def[pru_rpmsg_counter8].factor
                                        * (LIMIT(
                                                (received_pru1_data_struct->rc_array.chan[pru_rpmsg_counter8]),
                                                rc_receiver_chan_def[pru_rpmsg_counter8].max,
                                                rc_receiver_chan_def[pru_rpmsg_counter8].min)
                                                - rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter))
                                        >> 10;
                    }
                    // scale throttle from [-32768, 32767] to [0, 32767]
                    received_pru1_data_struct->rc.throttle = MAX(
                            0,
                            (received_pru1_data_struct->rc.throttle >> 1)
                                    + 16384);
                }
                else
                {
                    // calc rawMin, rawMax, rawCenter values
                    // NOTA: calibration must be started with stick on the center position
                    for (pru_rpmsg_counter8 = 0; pru_rpmsg_counter8 < 8;
                            pru_rpmsg_counter8++)
                    {
                        pru_rpmsg_rc_chan_temp =
                                received_pru1_data_struct->rc_array.chan[pru_rpmsg_counter8];
                        if (pru_rpmsg_rc_chan_temp
                                < rc_receiver_chan_def[pru_rpmsg_counter8].rawMin)
                        {
                            rc_receiver_chan_def[pru_rpmsg_counter8].rawMin =
                                    pru_rpmsg_rc_chan_temp;
                        }
                        else if (pru_rpmsg_rc_chan_temp
                                > rc_receiver_chan_def[pru_rpmsg_counter8].rawMax)
                        {
                            rc_receiver_chan_def[pru_rpmsg_counter8].rawMax =
                                    pru_rpmsg_rc_chan_temp;
                        }
                        else if (rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter
                                == 0)
                        {
                            rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter =
                                    pru_rpmsg_rc_chan_temp;
                        }
                    }
                }

                // Map rc channels from sensor to pid order
                pru_rpmsg_rc[POS_YAW] = received_pru1_data_struct->rc.yaw;
                pru_rpmsg_rc[POS_PITCH] = received_pru1_data_struct->rc.pitch;
                pru_rpmsg_rc[POS_ROLL] = received_pru1_data_struct->rc.roll;
                pru_rpmsg_rc[POS_THROTTLE] =
                        received_pru1_data_struct->rc.throttle;
                pru_rpmsg_rc[POS_AUX1] = received_pru1_data_struct->rc.aux1;
                pru_rpmsg_rc[POS_AUX2] = received_pru1_data_struct->rc.aux2;
                pru_rpmsg_rc[POS_AUX3] = received_pru1_data_struct->rc.aux3;
                pru_rpmsg_rc[POS_AUX4] = received_pru1_data_struct->rc.aux4;

                if (src_rc_channel != 0)
                {
                    // send data from PRU1 to ARM
                    pru_rpmsg_send(&transport, RPMSG_RC_CHAN_PORT,
                                   src_rc_channel, received_pru1_data,
                                   sizeof(PrbMessageType));
                }
                break;
            }
                // end case RC_DATA_MSG_TYPE
            } // end switch

        } // end if received message from P1
          // received message from ARM
        else if (CT_INTC.SECR0_bit.ENA_STS_31_0 & (1 << INT_ARM_TO_P0))
        {
            CT_INTC.SICR_bit.STS_CLR_IDX = INT_ARM_TO_P0;
            if (pru_rpmsg_receive(&transport, &src, &dst, received_arm_data,
                                  &len) == PRU_RPMSG_SUCCESS)
            {
                // send data from ARM to PRU1
                __xout(SP_BANK_0, 3, 0, received_arm_data);
                // send interrupt to P1
                CT_INTC.SRSR0_bit.RAW_STS_31_0 |= (1 << INT_P0_TO_P1);

                switch (received_arm_data_struct->message_type)
                {
                // TODO: aggiungere i case per ogni canale
                // disabilitare prima il sensore relativo al canale
                case MPU_ENABLE_MSG_TYPE:
                {
                    src_mpu_channel = src;
                    break;
                }
                case MPU_DISABLE_MSG_TYPE:
                {
                    src_mpu_channel = src;
                    break;
                }
                case MPU_CREATE_CHANNEL_MSG_TYPE:
                {
                    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport,
                    RPMSG_MPU_CHAN_NAME,
                                             RPMSG_MPU_CHAN_DESC,
                                             RPMSG_MPU_CHAN_PORT)
                            != PRU_RPMSG_SUCCESS)
                        ;
                    break;
                } // end case MPU_CREATE_CHANNEL
                case MPU_DESTROY_CHANNEL_MSG_TYPE:
                {
                    while (pru_rpmsg_channel(RPMSG_NS_DESTROY, &transport,
                    RPMSG_MPU_CHAN_NAME,
                                             RPMSG_MPU_CHAN_DESC,
                                             RPMSG_MPU_CHAN_PORT)
                            != PRU_RPMSG_SUCCESS)
                        ;
                    break;
                } // end case MPU_DESTROY_CHANNEL_MSG_TYPE
                case RC_ENABLE_MSG_TYPE:
                {
                    src_rc_channel = src;
                    break;
                }
                case RC_DISABLE_MSG_TYPE:
                {
                    src_rc_channel = src;
                    break;
                }
                case RC_CALIBRATION_ENABLE_MSG_TYPE:
                {
                    src_rc_channel = src;
                    pru_rpmsg_rc_status_calibration = 1;

                    // send data from PRU0 to ARM
                    received_pru1_data_struct->message_type =
                            RC_CALIBRATION_ENABLED_MSG_TYPE;
                    pru_rpmsg_send(&transport, RPMSG_RC_CHAN_PORT,
                                   src_rc_channel, received_pru1_data,
                                   sizeof(PrbMessageType));

                    // reset rc chan configuration
                    for (pru_rpmsg_counter8 = 0; pru_rpmsg_counter8 < 8;
                            pru_rpmsg_counter8++)
                    {
                        pru_rpmsg_rc_chan_temp =
                                (received_pru1_data_struct->rc_array.chan[pru_rpmsg_counter8]
                                        >> 10);
                        rc_receiver_chan_def[pru_rpmsg_counter8].rawMin = 32767;
                        rc_receiver_chan_def[pru_rpmsg_counter8].rawMax =
                                -32768;
                        rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter = 0;
                    }
                    break;
                }
                case RC_CALIBRATION_DISABLE_MSG_TYPE:
                {
                    src_rc_channel = src;
                    // send data from PRU0 to ARM
                    received_pru1_data_struct->message_type =
                            RC_CALIBRATION_DISABLED_MSG_TYPE;
                    pru_rpmsg_send(&transport, RPMSG_RC_CHAN_PORT,
                                   src_rc_channel, received_pru1_data,
                                   sizeof(PrbMessageType));

                    if (pru_rpmsg_rc_status_calibration)
                    {
                        // calc rc radius, min, max
                        for (pru_rpmsg_counter8 = 0; pru_rpmsg_counter8 < 8;
                                pru_rpmsg_counter8++)
                        {
                            rc_receiver_chan_def[pru_rpmsg_counter8].radius =
                                    MAX(0,
                                        MIN(rc_receiver_chan_def[pru_rpmsg_counter8].rawMax - rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter, rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter - rc_receiver_chan_def[pru_rpmsg_counter8].rawMin));
                            rc_receiver_chan_def[pru_rpmsg_counter8].min =
                                    rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter
                                            - rc_receiver_chan_def[pru_rpmsg_counter8].radius;
                            rc_receiver_chan_def[pru_rpmsg_counter8].max =
                                    rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter
                                            + rc_receiver_chan_def[pru_rpmsg_counter8].radius;
                            rc_receiver_chan_def[pru_rpmsg_counter8].factor =
                                    (65535 << 10)
                                            / (rc_receiver_chan_def[pru_rpmsg_counter8].radius
                                                    << 1);
                        }

                    }
                    pru_rpmsg_rc_status_calibration = 0;
                    break;
                }
                case RC_GET_CONFIG_MSG_TYPE: {
                    src_rc_channel = src;
                    // send rc config data from PRU0 to ARM
                    pru_rpmsg_config_data_struct->message_type =
                            RC_CONFIG_DATA_MSG_TYPE;
                    for(pru_rpmsg_counter8 = 0; pru_rpmsg_counter8 < 8; pru_rpmsg_counter8++) {
                        pru_rpmsg_config_data_struct->rc_config_chan[pru_rpmsg_counter8].rawMin = rc_receiver_chan_def[pru_rpmsg_counter8].rawMin;
                        pru_rpmsg_config_data_struct->rc_config_chan[pru_rpmsg_counter8].rawMax = rc_receiver_chan_def[pru_rpmsg_counter8].rawMax;
                        pru_rpmsg_config_data_struct->rc_config_chan[pru_rpmsg_counter8].rawCenter = rc_receiver_chan_def[pru_rpmsg_counter8].rawCenter;
                        pru_rpmsg_config_data_struct->rc_config_chan[pru_rpmsg_counter8].min = rc_receiver_chan_def[pru_rpmsg_counter8].min;
                        pru_rpmsg_config_data_struct->rc_config_chan[pru_rpmsg_counter8].max = rc_receiver_chan_def[pru_rpmsg_counter8].max;
                        pru_rpmsg_config_data_struct->rc_config_chan[pru_rpmsg_counter8].radius = rc_receiver_chan_def[pru_rpmsg_counter8].radius;
                        pru_rpmsg_config_data_struct->rc_config_chan[pru_rpmsg_counter8].factor = rc_receiver_chan_def[pru_rpmsg_counter8].factor;
                    }
                    pru_rpmsg_send(&transport, RPMSG_RC_CHAN_PORT,
                                   src_rc_channel, pru_rpmsg_config_data_struct,
                                   sizeof(PrbConfigMessageType));
                    break;
                }
                case RC_CREATE_CHANNEL_MSG_TYPE:
                {
                    while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport,
                    RPMSG_RC_CHAN_NAME,
                                             RPMSG_RC_CHAN_DESC,
                                             RPMSG_RC_CHAN_PORT)
                            != PRU_RPMSG_SUCCESS)
                        ;
                    break;
                } // end case RC_CREATE_CHANNEL
                case RC_DESTROY_CHANNEL_MSG_TYPE:
                {
                    while (pru_rpmsg_channel(RPMSG_NS_DESTROY, &transport,
                    RPMSG_RC_CHAN_NAME,
                                             RPMSG_RC_CHAN_DESC,
                                             RPMSG_RC_CHAN_PORT)
                            != PRU_RPMSG_SUCCESS)
                        ;
                    break;
                } // end case RC_DESTROY_CHANNEL_MSG_TYPE
                } // end switch
            }
        } // end if received message from ARM
    }
}

