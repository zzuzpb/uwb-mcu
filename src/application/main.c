/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "compiler.h"
#include "port.h"

#include "scheduler.h"
#include "instance.h"

#include "deca_types.h"

#include "deca_spi.h"

uint8 s1switch = 0;
int instance_anchaddr = 0;
#define tagaddr (instance_anchaddr)
#define ancaddr (instance_anchaddr)

int instance_mode = ANCHOR;
//int instance_mode = TAG;
//int instance_mode = LISTENER;

typedef struct
{
    uint8 channel ;
    uint8 prf ;
    uint8 datarate ;
    uint8 preambleCode ;
    uint8 preambleLength ;
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO ;
} chConfig_t ;


//Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
chConfig_t chConfig =
#if 0
                    //mode 1 - S1: 2 off, 3 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        4,              // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2 - S1: 2 on, 3 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        4,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3 - S1: 2 off, 3 on
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,              // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    }
#endif
                    //mode 4 - S1: 2 on, 3 on
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    };


// ======================================================
//
//  Configure instance tag/anchor/etc... addresses
//
void addressconfigure(void)
{
    instanceAddressConfig_t ipc ;

   	ipc.anchorAddress = ANCHOR_BASE_ADDR | instance_anchaddr;
    ipc.tagAddress = instance_anchaddr;

    instancesetaddresses(&ipc);
}

uint32 inittestapplication(void)
{
    uint32 devID ;
    instanceConfig_t instConfig;
    int result;

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);  //max SPI before PLLs configured is ~4M

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_SPIx_clear_chip_select();  //CS low
        Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
        port_SPIx_set_chip_select();  //CS high
        Sleep(7);
        devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    addressconfigure() ;                            // set up initial payload configuration

    instancesetrole(instance_mode) ;     // Set this instance role

    instConfig.channelNumber = chConfig.channel ;
    instConfig.preambleCode = chConfig.preambleCode ;
    instConfig.pulseRepFreq = chConfig.prf ;
    instConfig.pacSize = chConfig.pacSize ;
    instConfig.nsSFD = chConfig.nsSFD ;
    instConfig.sfdTO = chConfig.sfdTO ;
    instConfig.dataRate = chConfig.datarate ;
    instConfig.preambleLen = chConfig.preambleLength ;

    instance_config(&instConfig) ;                  // Set operating channel etc

    return devID;
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
void process_dwRSTn_irq(void)
{
    instance_notify_DW1000_inIDLE(1);
}

void process_deca_irq(void)
{
    do{

        instance_process_irq(0);

    }while(port_CheckEXT_IRQ() == 1); //while IRS line active (ARM can only do edge sensitive interrupts)

}

static struct part_configuration_t {
	uint32 part_low;
	uint32 mode;
	uint32 no;
} part_configuration_tab[] = {
	{ 0x1AA7U, ANCHOR  , 0 },
	{ 0x2415U, ANCHOR  , 1 },
	{ 0x0b5fU, ANCHOR  , 2 },
	{ 0x0242U, ANCHOR  , 3 },
	{ 0x101eU, LISTENER, 0 },
	{ 0x1024U, LISTENER, 1 },
	{ 0x104aU, LISTENER, 2 },
};
static void setup_modem_paramters_according_part_no(void)
{
	size_t i;
	uint32 get_part(void);
	uint32 part = get_part();
	for (i = 0; i < sizeof(part_configuration_tab)/ sizeof(part_configuration_tab[0]); i++){
		if ((part & 0xFFFFU) == part_configuration_tab[i].part_low) {
			const struct part_configuration_t *p = &part_configuration_tab[i];
			instance_anchaddr = p->no;
			instance_mode     = p->mode;
			break;
		}
	}
}

/*
 * @fn      main()
 * @brief   main entry point
**/
#pragma GCC optimize ("O3")
int main(void)
{
    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();
    UartSend("Hello RTLS System(v0.9a) by X-Lab, Huaqin\r\n");

    spi_peripheral_init();

    setup_modem_paramters_according_part_no();

    port_DisableEXT_IRQ(); //disable ScenSor IRQ until we configure the device

    {

        led_off(LED_ALL);

        if(inittestapplication() == (uint32)-1)
        {
            led_on(LED_ALL); //to display error....
            return 0; //error
        }
    }

    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting

    SchedulerInit(tagaddr);

    // main loop
    while(1)
    {
        instance_run();

        if(instancenewrange())
        {
#if 0
        	int n = 0, l = 0, r= 0, aaddr, taddr;
        	int rangeTime, correction;
        	int rres, rres_raw;
            char buf[100];

            range_result = instancegetidist();
            range_raw = instancegetidistraw();
            aaddr = instancenewrangeancadd() & 0xf;
            taddr = instancenewrangetagadd() & 0xf;
            rangeTime = instancenewrangetim() & 0xffffffff;
            sprintf(buf, "%x %x %f\r\n", aaddr, taddr, range_result);
            UartSend(buf);
#endif
        }
    }


    return 0;
}



