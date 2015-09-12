/*! ----------------------------------------------------------------------------
 *  @file    instance_common.c
 *  @brief   DecaWave application level common instance functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "compiler.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_spi.h"

#include "instance.h"


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------

double inst_idist = 0;
double inst_idistraw = 0;
instance_data_t instance_data[NUM_INST] ;

//int eventOutcount = 0;
//int eventIncount = 0;

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint64 convertmicrosectodevicetimeu (double microsecu)
{
    uint64 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64) (dtime) ;

    return dt;
}

double convertdevicetimetosec(int32 dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}


void reportTOF(instance_data_t *inst)
{
        double distance ;
        double distance_to_correct;
        double tof ;
        int64 tofi ;

        // check for negative results and accept them making them proper negative integers
        tofi = (int64) inst->tof ;                          // make it signed
        if (tofi > 0x007FFFFFFFFF)                          // MP counter is 40 bits, close up TOF may be negative
        {
            tofi -= 0x010000000000 ;                       // subtract fill 40 bit range to make it negative
        }

        // convert to seconds (as floating point)
        tof = convertdevicetimetosec(tofi) * 0.25;          //this is divided by 4 to get single time of flight
        inst_idistraw = distance = tof * SPEED_OF_LIGHT;

#if (CORRECT_RANGE_BIAS == 1)
        //for the 6.81Mb data rate we assume gating gain of 6dB is used,
        //thus a different range bias needs to be applied
        //if(inst->configData.dataRate == DWT_BR_6M8)
        if(inst->configData.smartPowerEn)
        {
        	//1.31 for channel 2 and 1.51 for channel 5
        	if(inst->configData.chan == 5)
        	{
        		distance_to_correct = distance/1.51;
        	}
        	else //channel 2
        	{
        		distance_to_correct = distance/1.31;
			}
        }
        else
        {
        	distance_to_correct = distance;
        }

        distance = distance - dwt_getrangebias(inst->configData.chan, (float) distance_to_correct, inst->configData.prf);
#endif

        if ((distance < 0) || (distance > 20000.000))    // discard any results less than <0 cm or >20km
            return;

        inst_idist = distance;

        inst->longTermRangeCount++ ;                          // for computing a long term average

    return ;
}// end of reportTOF


// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else


// -------------------------------------------------------------------------------------------------------------------
// Set this instance role as the Tag, Anchor or Listener
void instancesetrole(int inst_mode)
{
    // assume instance 0, for this
    instance_data[0].mode =  inst_mode;                   // set the role
}

int instancegetrole(void)
{
    return instance_data[0].mode;
}

int instancenewrange(void)
{
    if(instance_data[0].newrange)
    {
        instance_data[0].newrange = 0;
        return 1;
    }

    return 0;
}

int instancenewrangeancadd(void)
{
    return instance_data[0].newrangeancaddress;
}

int instancenewrangetagadd(void)
{
    return instance_data[0].newrangetagaddress;
}

int instancenewrangetim(void)
{
    return instance_data[0].newrangetime;
}

// -------------------------------------------------------------------------------------------------------------------
// function to clear counts/averages/range values
//
void instanceclearcounts(void)
{
    int instance = 0 ;

    instance_data[instance].rxTimeouts = 0 ;

    dwt_configeventcounters(1); //enable and clear - NOTE: the counters are not preserved when in DEEP SLEEP

    instance_data[instance].frame_sn = 0;
    instance_data[instance].lastReportSN = 0xff;

    instance_data[instance].longTermRangeCount  = 0;

} // end instanceclearcounts()


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int instance_init(void)
{
    int instance = 0 ;
    int result;

    instance_data[instance].mode =  ANCHOR;                                // assume listener,
    instance_data[instance].testAppState = TA_INIT ;
    instance_data[instance].instToSleep = FALSE;


    // Reset the IC (might be needed if not getting here from POWER ON)
    // ARM code: Remove soft reset here as using hard reset in the inittestapplication() in the main.c file
    //dwt_softreset();

	//this initialises DW1000 and uses specified configurations from OTP/ROM
    result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM) ;

    //this is platform dependent - only program if DW EVK/EVB
    dwt_setleds(3) ; //configure the GPIOs which control the leds on EVBs

    if (DWT_SUCCESS != result)
    {
        return (-1) ;   // device initialise has failed
    }


    instanceclearcounts() ;

    instance_data[instance].panid = 0xdeca ;

    instance_data[instance].wait4ack = 0;

    instance_clearevents();

    //dwt_geteui(instance_data[instance].eui64);
    memset(instance_data[instance].eui64, 0, ADDR_BYTE_SIZE_L);

    instance_data[instance].anchorListIndex = 0 ;

    instance_data[instance].rxautoreenable = 0;
    dwt_setautorxreenable(0); //disable auto RX re-enable
    dwt_setdblrxbuffmode(0); //disable double RX buffer

    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_SFDT | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);

    dwt_setcallbacks(instance_txcallback, instance_rxcallback);

    return 0 ;
}

// -------------------------------------------------------------------------------------------------------------------
//
// Return the Device ID register value, enables higher level validation of physical device presence
//

uint32 instancereaddeviceid(void)
{
    return dwt_readdevid() ;
}


// -------------------------------------------------------------------------------------------------------------------
//
// function to allow application configuration be passed into instance and affect underlying device operation
//
void instance_config(instanceConfig_t *config)
{
    int instance = 0 ;
    uint32 power = 0;
    uint8 otprev ;

    instance_data[instance].configData.chan = config->channelNumber ;
    instance_data[instance].configData.rxCode =  config->preambleCode ;
    instance_data[instance].configData.txCode = config->preambleCode ;
    instance_data[instance].configData.prf = config->pulseRepFreq ;
    instance_data[instance].configData.dataRate = config->dataRate ;
    instance_data[instance].configData.txPreambLength = config->preambleLen ;
    instance_data[instance].configData.rxPAC = config->pacSize ;
    instance_data[instance].configData.nsSFD = config->nsSFD ;
    instance_data[instance].configData.phrMode = DWT_PHRMODE_STD ;
    instance_data[instance].configData.sfdTO = config->sfdTO;

    //enable gating gain for 6.81Mbps data rate
#if 1
    if(instance_data[instance].configData.dataRate == DWT_BR_6M8)
    {
        instance_data[instance].configData.smartPowerEn = 1;
    }
    else
#endif
    {
        instance_data[instance].configData.smartPowerEn = 0;
    }

    //configure the channel parameters
    dwt_configure(&instance_data[instance].configData, DWT_LOADXTALTRIM) ;

    instance_data[instance].configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

    //firstly check if there are calibrated TX power value in the DW1000 OTP
    power = dwt_getotptxpower(config->pulseRepFreq, instance_data[instance].configData.chan);

    if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
    }

    //Configure TX power
    //if smart power is used then the value as read from OTP is used directly
    //if smart power is used the user needs to make sure to transmit only one frame per 1ms or TX spectrum power will be violated
    if(instance_data[instance].configData.smartPowerEn == 1)
    {
        instance_data[instance].configTX.power = power;
    }
	else //if the smart power is not used, then the low byte value (repeated) is used for the whole TX power register
    {
        uint8 pow = power & 0xFF ;
        instance_data[instance].configTX.power = (pow | (pow << 8) | (pow << 16) | (pow << 24));
    }
    dwt_setsmarttxpower(instance_data[instance].configData.smartPowerEn);

    //configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&instance_data[instance].configTX);

    otprev = dwt_otprevision() ;  // this revision tells us how OTP is programmed.

	if ((2 == otprev) || (3 == otprev))  // board is calibrated with TREK1000 with antenna delays set for each use case)
	{
		uint8 mode = (instance_data[instance].mode == ANCHOR ? 1 : 0);
		uint8 chanindex = 0;

		instance_data[instance].txantennaDelay
										= dwt_getTREKOTPantennadelay(mode,
												instance_data[instance].configData.chan,
												instance_data[instance].configData.dataRate) ;

		// if nothing was actually programmed then set a reasonable value anyway
		if ((instance_data[instance].txantennaDelay == 0)
				|| (instance_data[instance].txantennaDelay == 0xffff))
		{
			if(instance_data[instance].configData.chan == 5)
			{
				chanindex = 1;
			}

			instance_data[instance].txantennaDelay = rfDelaysTREK[chanindex];
		}

	}
	else // assume it is older EVK1000 programming.
	{
		//get the antenna delay that was read from the OTP calibration area
		instance_data[instance].txantennaDelay = dwt_readantennadelay(config->pulseRepFreq) >> 1;

		// if nothing was actually programmed then set a reasonable value anyway
		if ((instance_data[instance].txantennaDelay == 0)
				|| (instance_data[instance].txantennaDelay == 0xffff))
		{
			instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
		}
	}

	// -------------------------------------------------------------------------------------------------------------------
	// set the antenna delay, we assume that the RX is the same as TX.
	dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
	dwt_settxantennadelay(instance_data[instance].txantennaDelay);

    instance_data[instance].rxantennaDelay = instance_data[instance].txantennaDelay;

    if(config->preambleLen == DWT_PLEN_64) //if preamble length is 64
	{
    	SPI_ConfigFastRate(SPI_BaudRatePrescaler_32); //reduce SPI to < 3MHz

		dwt_loadopsettabfromotp(0);

		SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    }

    if(config->dataRate == DWT_BR_110K)
    {
        //set the default response delays
        instancesetreplydelay(FIXED_REPLY_DELAY_110K);
    }
    else
    {
        //set the default response delays
        instancesetreplydelay(FIXED_REPLY_DELAY_6M81);
	}
}

// -------------------------------------------------------------------------------------------------------------------
// function to set the tag sleep time (in ms)
//

int instancegetrnum(void) //get ranging number
{
	return instance_data[0].rangeNum;
}

int instancegetlcount(void) //get count of ranges used for calculation of lt avg
{
    int x = instance_data[0].longTermRangeCount;

    return (x);
}

double instancegetidist(void) //get instantaneous range
{
    double x = inst_idist;

    return (x);
}

double instancegetidistraw(void) //get instantaneous range (uncorrected)
{
    double x = inst_idistraw;

    return (x);
}

void inst_processrxtimeout(instance_data_t *inst)
{

	//inst->responseTimeouts ++ ;
    inst->rxTimeouts ++ ;
    inst->done = INST_NOT_DONE_YET;

    if(inst->mode == ANCHOR || inst->mode == LISTENER) //we did not receive the final - wait for next poll
    {
		//only enable receiver when not using double buffering
		inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
		dwt_setrxtimeout(0);

    }
	else //if(inst->mode == TAG)
    {
		// will continue next range, unlike origin version.
		inst->testAppState = TA_TXE_WAIT ;
		inst->nextState = TA_TXPOLL_WAIT_SEND ;
		return;
    }

    //timeout - disable the radio (if using SW timeout the rx will not be off)
    dwt_forcetrxoff() ;
}

//
// NB: This function is called from the (TX) interrupt handler
//
void instance_txcallback(const dwt_callback_data_t *txd)
{
	int instance = 0;
	uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};

	uint8 txevent = txd->event;
	event_data_t dw_event;

	if(instance_data[instance].ackreq) //the ACK has been requested in the last RX frame - we got TX event, this means the ACK has been sent
	{
		txevent = DWT_SIG_TX_AA_DONE;
		instance_data[instance].ackreq = 0;
        //add the pending event to the event queue, so that the received frame can be processed
        instance_putevent(instance_getsavedevent());
	}

	if(txevent == DWT_SIG_TX_DONE)
	{
		//uint64 txtimestamp = 0;

		//NOTE - we can only get TX good (done) while here
		//dwt_readtxtimestamp((uint8*) &instance_data[instance].txu.txTimeStamp);

		dwt_readtxtimestamp(txTimeStamp) ;
		dw_event.timeStamp32l = (uint32)txTimeStamp[0] + ((uint32)txTimeStamp[1] << 8) + ((uint32)txTimeStamp[2] << 16) + ((uint32)txTimeStamp[3] << 24);
		dw_event.timeStamp = txTimeStamp[4];
	    dw_event.timeStamp <<= 32;
		dw_event.timeStamp += dw_event.timeStamp32l;
		dw_event.timeStamp32h = ((uint32)txTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

		dw_event.rxLength = 0;
		dw_event.type2 = dw_event.type = DWT_SIG_TX_DONE ;

		instance_putevent(dw_event);

		//printf("TX time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
		//printf("TX Timestamp: %4.15e\n", convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp));
#if (DECA_SUPPORT_SOUNDING==1)
	#if DECA_ACCUM_LOG_SUPPORT==1
		if ((instance_data[instance].dwlogdata.accumLogging == LOG_ALL_ACCUM ) || (instance_data[instance].dwlogdata.accumLogging == LOG_ALL_NOACCUM ))
		{
			uint32 hi32 = dw_event.timeStamp >> 32;
			uint32 low32 = dw_event.timeStamp & 0xffffffff;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"\nTX Frame TimeStamp Raw  = %02X %02X%02X%02X%02X\n",txTimeStamp[4], txTimeStamp[3], txTimeStamp[2], txTimeStamp[1], txTimeStamp[0]) ;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"   Adding Antenna Delay = %04X %08X\n", hi32, low32 ) ;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"%02X Tx time = %4.15e\n", instance_data[instance].msg.seqNum, convertdevicetimetosecu(dw_event.timeStamp)) ;
		}
    #endif
#endif
	}
	else if(txevent == DWT_SIG_TX_AA_DONE)
	{
		//auto ACK confirmation
		dw_event.rxLength = 0;
		dw_event.type2 = dw_event.type = DWT_SIG_TX_AA_DONE ;

		instance_putevent(dw_event);

		//printf("TX AA time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
	}
}

//
// NB: This function is called from the (RX) interrupt handler
//
void instance_rxcallback(const dwt_callback_data_t *rxd)
{
	int instance = 0;
	uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};

    uint8 rxd_event = 0;
	//uint8 fcode_index  = 0;
	event_data_t dw_event;

	//if we got a frame with a good CRC - RX OK
    if(rxd->event == DWT_SIG_RX_OKAY)
	{
		//ACK request bit?
		instance_data[instance].ackreq = rxd->aatset;

        //if(instance_data[instance].ackreq) //this indicates that the ACK request has been set
        //{
            //need to hold onto the received frame until the ACK has been sent
            //configuration and sending of ACK frame can also be done here when not using auto-ACK feature.
			//	printf("ACK request\n");
        //}

        rxd_event = DWT_SIG_RX_OKAY;

		dw_event.rxLength = rxd->datalength;

		//need to process the frame control bytes to figure out what type of frame we have received
        switch(rxd->fctrl[0])
	    {
			//blink type frame
		    case 0xC5:
					rxd_event = SIG_RX_UNKNOWN;
					break;

			//ACK type frame
			case 0x02:
				if(rxd->datalength == 5)
					rxd_event = SIG_RX_ACK;
			    break;

			//data type frames (with/without ACK request) - assume PIDC is on.
			case 0x41:
			case 0x61:
				//read the frame
				if(rxd->datalength > STANDARD_FRAME_SIZE)
					rxd_event = SIG_RX_UNKNOWN;
/*
				//need to check the destination/source address mode
				if((rxd->fctrl[1] & 0xCC) == 0x88) //dest & src short (16 bits)
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
				}
				else if((rxd->fctrl[1] & 0xCC) == 0xCC) //dest & src long (64 bits)
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_L; //function code is in first byte after source address
				}
				else //using one short/one long
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_LS; //function code is in first byte after source address
				}
*/
				break;

			//any other frame types are not supported by this application
			default:
				rxd_event = SIG_RX_UNKNOWN;
				break;
		}


		//read rx timestamp
		if((rxd_event == SIG_RX_ACK) || (rxd_event == DWT_SIG_RX_OKAY))
		{
			dwt_readrxtimestamp(rxTimeStamp) ;
			dw_event.timeStamp32l =  (uint32)rxTimeStamp[0] + ((uint32)rxTimeStamp[1] << 8) + ((uint32)rxTimeStamp[2] << 16) + ((uint32)rxTimeStamp[3] << 24);
			dw_event.timeStamp = rxTimeStamp[4];
			dw_event.timeStamp <<= 32;
			dw_event.timeStamp += dw_event.timeStamp32l;
			dw_event.timeStamp32h = ((uint32)rxTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

			dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
			//dwt_readdignostics(&instance_data[instance].dwlogdata.diag);

			if(dwt_checkoverrun()) //the overrun has occured while we were reading the data - dump the frame/data
			{
				rxd_event = DWT_SIG_RX_ERROR ;
			}
		}

		dw_event.type2 = dw_event.type = rxd_event;

		//printf("rx call back %d %d FI:%08X %02x%02x event%d\n", rxd_event, instance_data[instance].ackreq, dwt_read32bitreg(0x10), rxd->fctrl[0], rxd->fctrl[1], instance_data[instance].dweventCnt);

		//Process good frames
		if(rxd_event == DWT_SIG_RX_OKAY)
		{
			//if(rxd->dblbuff == 0)  instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window
			//instance_calculatepower();

            if(instance_data[instance].ackreq == 0) //only notify there is event if no ACK pending
	    		instance_putevent(dw_event);
			else
				instance_saveevent(dw_event);

			#if DECA_LOG_ENABLE==1
			#if DECA_KEEP_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = DWT_SIG_RX_NOERR ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(DWT_SIG_RX_NOERR, dw_event.msgu.frame[fcode_index], dw_event.msgu.frame[2], &dw_event);
			#endif

			//printf("RX OK %d %x\n",instance_data[instance].testAppState, instance_data[instance].rxmsg.messageData[FCODE]);
			//printf("RX OK %d ", instance_data[instance].testAppState);
			//printf("RX time %f ecount %d\n",convertdevicetimetosecu(dw_event.timeStamp), instance_data[instance].dweventCnt);

		}
		else if (rxd_event == SIG_RX_ACK)
		{
			//printf("RX ACK %d (count %d) \n", instance_data[instance].testAppState, instance_data[instance].dweventCnt);
			instance_putevent(dw_event);
			//if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display
		}

		/*if(instance_data[instance].mode == LISTENER) //print out the message bytes when in Listener mode
		{
			int i;
			uint8 buffer[1024];
			dwt_readrxdata(buffer, rxd->datalength, 0);  // Read Data Frame
			buffer[1023] = 0;
			instancelogrxdata(&instance_data[instance], buffer, rxd->datalength);


			printf("RX data(%d): ", rxd->datalength);
			for(i=0; i<rxd->datalength; i++)
			{
				printf("%02x", buffer[i]);
			}
			printf("\n");
		}*/

		if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx
		{
			//if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display

			//dwt_readdignostics(&instance_data[instance].dwlogdata.diag);

			//instance_calculatepower();

			#if DECA_LOG_ENABLE==1
			#if DECA_KEEP_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = DWT_SIG_RX_NOERR ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(DWT_SIG_RX_NOERR, dw_event.msgu.frame[fcode_index], dw_event.msgu.frame[2], &dw_event);
			#endif

			dwt_rxenable(0) ;
		}
	}
	else if (rxd->event == DWT_SIG_RX_TIMEOUT)
	{
		dw_event.type2 = dw_event.type = DWT_SIG_RX_TIMEOUT;
		dw_event.rxLength = 0;
		dw_event.timeStamp = 0;
		dw_event.timeStamp32l = 0;
		dw_event.timeStamp32h = 0;

		instance_putevent(dw_event);
		//printf("RX timeout while in %d\n", instance_data[instance].testAppState);
	}
	else //assume other events are errors
	{
		//printf("RX error %d \n", instance_data[instance].testAppState);
		if(instance_data[instance].rxautoreenable == 0)
		{
			//re-enable the receiver
#if (DECA_BADF_ACCUMULATOR == 1)
			instance_readaccumulatordata();

			dwt_readdignostics(&instance_data[instance].dwlogdata.diag);

			instance_calculatepower();
#endif
#if (DECA_ERROR_LOGGING == 1)
			#if DECA_LOG_ENABLE==1
			#if DECA_BADF_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = rxd->event ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(rxd->event, 0, 0, &dw_event);
			#endif
#endif

			//for ranging application rx error frame is same as TO - as we are not going to get the expected frame
			if(instance_data[instance].mode == TAG)
			{
				dw_event.type = DWT_SIG_RX_TIMEOUT;
				dw_event.type2 = 0x40 | DWT_SIG_RX_TIMEOUT;
				dw_event.rxLength = 0;

				instance_putevent(dw_event);
			}
			else
			{
				dwt_rxenable(0) ;
			}

		}
	}
}


int instance_peekevent(void)
{
	int instance = 0;
    return instance_data[instance].dwevent[instance_data[instance].dweventPeek].type; //return the type of event that is in front of the queue
}

void instance_saveevent(event_data_t newevent)
{
	int instance = 0;

	instance_data[instance].saved_dwevent = newevent;
}

event_data_t instance_getsavedevent(void)
{
	int instance = 0;

	return instance_data[instance].saved_dwevent;
}

void instance_putevent(event_data_t newevent)
{
	int instance = 0;
	uint8 etype = newevent.type;

	newevent.type = 0;
	newevent.gotit = 0 ; //newevent.eventtimeclr = 0;

	//copy event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn] = newevent;

	//set type - this makes it a new event (making sure the event data is copied before event is set as new)
	//to make sure that the get event function does not get an incomplete event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn].type = etype;

	instance_data[instance].dweventIdxIn++;

	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxIn)
		instance_data[instance].dweventIdxIn = 0;

	//eventIncount++;

	//printf("put %d - in %d out %d @ %d\n", newevent.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut, ptime);
}

event_data_t dw_event_g;

#pragma GCC optimize ("O0")
event_data_t* instance_getevent(int x)
{
	int instance = 0;
	int indexOut = instance_data[instance].dweventIdxOut;

	//dw_event_g = instance_data[instance].dwevent[instance_data[instance].dweventCntOut]; //this holds any TX/RX events

	//memcpy(&dw_event_g, &instance_data[instance].dwevent[instance_data[instance].dweventCntOut], sizeof(event_data_t));

	if(instance_data[instance].dwevent[indexOut].type == 0) //exit with "no event"
	{
		dw_event_g.type = 0;
		dw_event_g.type2 = 0;
		return &dw_event_g;
	}

	//copy the event
	dw_event_g.type2 = instance_data[instance].dwevent[indexOut].type2 ;
	dw_event_g.rxLength = instance_data[instance].dwevent[indexOut].rxLength ;
	dw_event_g.timeStamp = instance_data[instance].dwevent[indexOut].timeStamp ;
	dw_event_g.timeStamp32l = instance_data[instance].dwevent[indexOut].timeStamp32l ;
	dw_event_g.timeStamp32h = instance_data[instance].dwevent[indexOut].timeStamp32h ;

	memcpy(&dw_event_g.msgu, &instance_data[instance].dwevent[indexOut].msgu, sizeof(instance_data[instance].dwevent[indexOut].msgu));

	dw_event_g.type = instance_data[instance].dwevent[indexOut].type ;


	instance_data[instance].dwevent[indexOut].gotit = x;

	instance_data[instance].dwevent[indexOut].type = 0; //clear the event

	instance_data[instance].dweventIdxOut++;
	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxOut) //wrap the counter
		instance_data[instance].dweventIdxOut = 0;

	instance_data[instance].dweventPeek = instance_data[instance].dweventIdxOut; //set the new peek value

	//if(dw_event.type) printf("get %d - in %d out %d @ %d\n", dw_event.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut, ptime);

	//eventOutcount++;


	return &dw_event_g;
}

void instance_clearevents(void)
{
	int i = 0;
	int instance = 0;

	for(i=0; i<MAX_EVENT_NUMBER; i++)
	{
        memset(&instance_data[instance].dwevent[i], 0, sizeof(event_data_t));
	}

	instance_data[instance].dweventIdxIn = 0;
	instance_data[instance].dweventIdxOut = 0;
	instance_data[instance].dweventPeek = 0;

	//eventOutcount = 0;
	//eventIncount = 0;
}

// -------------------------------------------------------------------------------------------------------------------
int instance_run(void)
{
    int instance = 0 ;
    int done = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR


        while(done == INST_NOT_DONE_YET)
        {
            //int state = instance_data[instance].testAppState;
            done = testapprun(&instance_data[instance], message) ;                                               // run the communications application

            //we've processed message
            message = 0;
        }
    return 0 ;
}


void instance_close(void)
{
    //wake up device from low power mode
    //NOTE - in the ARM  code just drop chip select for 200us
    port_SPIx_clear_chip_select();  //CS low
    Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
    port_SPIx_set_chip_select();  //CS high
    Sleep(5);
    dwt_entersleepaftertx(0); // clear the "enter deep sleep after tx" bit

    dwt_setinterrupt(0xFFFFFFFF, 0); //don't allow any interrupts

}


void instance_notify_DW1000_inIDLE(int idle)
{
	instance_data[0].dwIDLE = idle;
}

void instanceconfigtxpower(uint32 txpower)
{
	instance_data[0].txpower = txpower ;

	instance_data[0].txpowerChanged = 1;

}

void instancesettxpower(void)
{
	if(instance_data[0].txpowerChanged == 1)
	{
	    //Configure TX power
	    dwt_write32bitreg(0x1E, instance_data[0].txpower);

		instance_data[0].txpowerChanged = 0;
	}
}

void instanceconfigantennadelays(uint16 tx, uint16 rx)
{
	instance_data[0].txantennaDelay = tx ;
	instance_data[0].rxantennaDelay = rx ;

	instance_data[0].antennaDelayChanged = 1;
}

void instancesetantennadelays(void)
{
	if(instance_data[0].antennaDelayChanged == 1)
	{
		dwt_setrxantennadelay(instance_data[0].rxantennaDelay);
		dwt_settxantennadelay(instance_data[0].txantennaDelay);

		instance_data[0].antennaDelayChanged = 0;
	}
}


uint16 instancetxantdly(void)
{
	return instance_data[0].txantennaDelay;
}

uint16 instancerxantdly(void)
{
	return instance_data[0].rxantennaDelay;
}
#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
