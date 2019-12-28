#include <resource_table.h>
#include <pru_spi_lib.h>
#include <pru_cfg.h>

#define MOSI    7       //P8_40
#define CLK     4       //P8_41
#define MISO    8       //P8_27
#define CS      10      //P8_28
#define MPU_INT 9       //P8_29

uint16_t RESULT = 0x00;
uint16_t RESULT1 = 0x00;
uint16_t mosiData[256] = { };
uint16_t misoData[256] = { };


/**
 * main.c
 */
int main(void)
{
    /*
     * CT_CFG.SYSCFG_bit.STANDBY_INIT : the object is used to write data to
     * the SYSCFG register. Writing 0 to the STANDBY_INIT section of the
     * register opens up the OCP master port that is used for PRU to ARM
     * communication.
     */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    PruSpiStatus spiStatus = { .pins = {
                                         MOSI,
                                         MISO,
                                         CLK,
                                         CS },
                               PRU_1MHZ_CPU_CYCLES, mosiData, misoData };

    mosiData[0] = 0xF500;

    while (1)
    {
        pru_spi_transferData(&spiStatus, 1);
    }
}

