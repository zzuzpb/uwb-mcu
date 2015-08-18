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

#include "instance.h"

#include "deca_types.h"

#include "deca_spi.h"

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*);
extern void send_usbmessage(uint8*, int);

                             //"1234567812345678"
#define SOFTWARE_VER_STRING    "Ver. 1.06ru TREK" //16 bytes!

uint8 s1switch = 0;
int instance_anchaddr = 0;
#define tagaddr (instance_anchaddr)
#define ancaddr (instance_anchaddr)

int instance_mode = ANCHOR;
//int instance_mode = TAG;
//int instance_mode = LISTENER;

#define LCD_BUFF_LEN (80)
uint8 dataseq[LCD_BUFF_LEN];
uint8 dataseq1[LCD_BUFF_LEN];

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

    if(instance_anchaddr > 3)
    {
    	ipc.anchorAddress = GATEWAY_ANCHOR_ADDR | 0x4 ; //listener
    }
    else
    {
    	ipc.anchorAddress = GATEWAY_ANCHOR_ADDR | instance_anchaddr;
    }
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

void initLCD(void)
{
    uint8 initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38 /*0x3C*/, 0x0C, 0x01, 0x06 };
    uint8 command = 0x0;
    int j = 100000;

    writetoLCD( 9, 0,  initseq); //init seq
    while(j--);

    command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
    command = 0x1 ;  //clear screen
    writetoLCD( 1, 0,  &command);
}

static struct part_configuration_t {
	uint32 part_low;
	uint32 mode;
	uint32 no;
} part_configuration_tab[] = {
	{ 0x2415U, TAG    , 1 },
	{ 0x0b5fU, ANCHOR , 0 },
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
    int i = 0;
    //int toggle = 1;
    uint8 command = 0x0;
    uint16  tagusbqidx = 0;
    uint8 usbVCOMout[LCD_BUFF_LEN*4];
    double range_result = 0, range_raw = 0;

    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();

    spi_peripheral_init();

    Sleep(1000); //wait for LCD to power on

    initLCD();

    memset(dataseq, 0, LCD_BUFF_LEN);
    memcpy(dataseq, (const uint8 *) "DECAWAVE        ", 16);
    writetoLCD( 40, 1, dataseq); //send some data
    memcpy(dataseq, (const uint8 *) SOFTWARE_VER_STRING, 16); // Also set at line #26 (Should make this from single value !!!)
    writetoLCD( 16, 1, dataseq); //send some data

    Sleep(1000);
#ifdef USB_SUPPORT
    // enable the USB functionality
    usb_init();
    Sleep(1000);
#endif

    setup_modem_paramters_according_part_no();

    port_DisableEXT_IRQ(); //disable ScenSor IRQ until we configure the device

    {

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
        memset(dataseq, ' ', LCD_BUFF_LEN);
        memcpy(dataseq, (const uint8 *) "DECAWAVE   TREK ", 16);
        writetoLCD( 16, 1, dataseq); //send some data

        led_off(LED_ALL);

        if(inittestapplication() == (uint32)-1)
        {
            led_on(LED_ALL); //to display error....
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  &dataseq[0]);
            memset(dataseq, ' ', LCD_BUFF_LEN);
            memcpy(dataseq, (const uint8 *) "ERROR   ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(dataseq, (const uint8 *) "  INIT FAIL ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            return 0; //error
        }

#ifdef USB_SUPPORT //this is defined in the port.h file
        // Configure USB for output, (i.e. not USB to SPI)
        usb_printconfig(16, (uint8 *)SOFTWARE_VER_STRING);
#endif

        //sleep for 5 seconds displaying last LCD message and flashing LEDs
        i=30;
        while(i--)
        {
            if (i & 1) led_off(LED_ALL);
            else    led_on(LED_ALL);

            Sleep(200);
        }
        i = 0;
        led_off(LED_ALL);
        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);

        memset(dataseq, ' ', LCD_BUFF_LEN);
        memset(dataseq1, ' ', LCD_BUFF_LEN);

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
    }

    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting

    //memset(dataseq, ' ', LCD_BUFF_LEN);
    memset(dataseq1, ' ', LCD_BUFF_LEN);

    // main loop
    while(1)
    {
        instance_run();

        //if there is a new ranging report received or a new range has been calculated, then prepare data
        //to output over USB - Virtual COM port, and update the LCD
        if(instancenewrange())
        {
        	int n = 0, l = 0, r= 0, aaddr, taddr;
        	int rangeTime, correction;
        	int rres, rres_raw;
            uint16 txa, rxa;

            //send the new range information to LCD and/or USB
            range_result = instancegetidist();
            range_raw = instancegetidistraw();
            aaddr = instancenewrangeancadd() & 0xf;
            taddr = instancenewrangetagadd() & 0xf;
            rangeTime = instancenewrangetim() & 0xffffffff;
#if (LCD_UPDATE_ON == 1)
            //led_on(LED_PC9);
            command = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  &command);

            memset(dataseq1, ' ', LCD_BUFF_LEN);
            {
            	void get_part_lot_id(uint32 *part, uint32 *lot);
            	uint32 part, lot;
            	get_part_lot_id(&part, &lot);
            	sprintf(dataseq, "%08X%08x", part,lot);
            }

            writetoLCD( 40, 1, dataseq); //send some data

            if(instance_mode == ANCHOR)
            {
            	sprintf((char*)&dataseq1[0], "A%d  A%dT%d: %3.2f m", ancaddr, aaddr, taddr, range_result);
            }
            else if(instance_mode == TAG)
            {
                sprintf((char*)&dataseq1[0], "T%d  A%dT%d: %3.2f m", tagaddr, aaddr, taddr, range_result);
            }
            else
            {
            	sprintf((char*)&dataseq1[0], "LS  A%dT%d: %3.2f m", aaddr, taddr, range_result);
            }
            writetoLCD( 16, 1, dataseq1); //send some data
            //led_off(LED_PC9);
#endif
#ifdef USB_SUPPORT //this is set in the port.h file
            //led_on(LED_PC9);
            l = instancegetlcount() & 0xFFFF;
            r = instancegetrnum();
            txa =  instancetxantdly();
            rxa =  instancerxantdly();
            rres = ((int)(range_result*1000));
            rres_raw = ((int)(range_raw*1000));
            //correction = instance_data[0].tagSleepCorrection2;
            // anchorID tagID range rangeraw countofranges rangenum rangetime txantdly rxantdly address
            if(instance_mode == TAG)
            {
            	if((tagusbqidx + 60) < (LCD_BUFF_LEN*4)) //assume we need 60 length at most (this is the report length below)
            	{

            		n = sprintf((char*)&usbVCOMout[tagusbqidx], "ma%02x t%02x %08x %08x %04x %02x %08x %04x %04x t%d\r\n", aaddr, taddr, rres, rres_raw, l, r, rangeTime, txa, rxa, tagaddr);
            		//n = sprintf((char*)&usbVCOMout[tagusbqidx], "ma%02x t%02x %08x %08x %08x %08x %04x %04x t%d\r\n", aaddr, taddr, rres, instance_data[0].systime, instance_data[0].delayedReplyTime, rangeTime, instance_data[0].lateTX, rxa, tagaddr);

            		tagusbqidx+=n;
            	}
            }
            else if (instance_mode == ANCHOR)
            {
            	n = sprintf((char*)&usbVCOMout[0], "ma%02x t%02x %08x %08x %04x %02x %08x %04x %04x a%d", aaddr, taddr, rres, rres_raw, l, r, rangeTime, txa, rxa, ancaddr);
                send_usbmessage(&usbVCOMout[0], n);
            }
            else
            {
            	n = sprintf((char*)&usbVCOMout[0], "ma%02x t%02x %08x %08x %04x %02x %08x %04x %04x l%d", aaddr, taddr, rres, rres_raw, l, r, rangeTime, txa, rxa, ancaddr);
                send_usbmessage(&usbVCOMout[0], n);
            }
            //led_off(LED_PC9);
#endif
        }

#ifdef USB_SUPPORT //this is set in the port.h file
        if((tagusbqidx != 0) && (instance_data[0].testAppState == TA_SLEEP_DONE)) //only TX over USB when Tag sleeping
        {
        	send_usbmessage(&usbVCOMout[0], tagusbqidx - 2); //so we don't add another new line
        	tagusbqidx = 0;
        }

        //led_on(LED_PC7);
        usb_run();
        //led_off(LED_PC7);
#endif
    }


    return 0;
}



