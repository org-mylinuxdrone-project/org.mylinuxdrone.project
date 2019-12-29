#include <resource_table.h>
#include <pru_spi_lib.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>

#ifdef PRU1_CTRL
#define PRU_CTRL PRU1_CTRL
#else
#define PRU_CTRL PRU0_CTRL
#endif

#define MOSI    7       //P8_40
#define CLK     4       //P8_41
#define MISO    8       //P8_27
#define CS      10      //P8_28
#define MPU_INT 9       //P8_29

uint16_t RESULT = 0x00;
uint16_t CHECK = 0x00;
uint16_t ERROR = 0x00;
uint32_t COUNTER = 0x00;
uint16_t mosiData = 0xF500;
uint16_t misoData = 0x0;


/**
 * main.c
 */
int main(void)
{
    uint32_t i = 0;
    /*
     * CT_CFG.SYSCFG_bit.STANDBY_INIT : the object is used to write data to
     * the SYSCFG register. Writing 0 to the STANDBY_INIT section of the
     * register opens up the OCP master port that is used for PRU to ARM
     * communication.
     */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    PruSpiStatus spiStatus = {mosiData, misoData };

    CHECK = 0;
    ERROR = 0;
    while (1)
    {
        // enable counter
        PRU_CTRL.CYCLE = 0;
        PRU_CTRL.CTRL_bit.CTR_EN = 1;
        for (i = 0x80000000; i != 0; i = i >> 1){
            pru_spi_transferData(&spiStatus);
            RESULT = spiStatus.misoData;
            if(RESULT != 0x70) {
                CHECK++;
                ERROR = RESULT;
            }
        }
        PRU_CTRL.CTRL_bit.CTR_EN = 0;
        COUNTER = PRU_CTRL.CYCLE;
    }
}

