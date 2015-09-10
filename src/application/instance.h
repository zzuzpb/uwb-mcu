/*! ----------------------------------------------------------------------------
 *  @file    instance.h
 *  @brief   DecaWave header for application level instance
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "deca_types.h"
#include "deca_device_api.h"

/******************************************************************************************************************
********************* NOTES on DW (MP) features/options ***********************************************************
*******************************************************************************************************************/

#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define NUM_INST            1
#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

#define SIG_RX_ACK				5		// Frame Received is an ACK (length 5 bytes)
#define SIG_RX_UNKNOWN			99		// Received an unknown frame

//Fast 2WR function codes
#define RTLS_DEMO_MSG_TAG_POLL              (0x81)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x70)          // Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINAL             (0x82)          // Tag final massage back to Anchor
#define RTLS_DEMO_MSG_ANCH_TOFR             (0x71)          // Anchor TOF Report message


//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    3				// FunctionCode(1), Range Num (1), Poll Num (1)
#define ANCH_RESPONSE_MSG_LEN               4               // FunctionCode(1), Gateway (1), Sleep Correction Time (2)
#define TAG_FINAL_MSG_LEN                   16              // FunctionCode(1), Poll_TxTime(5), Resp_RxTime(5), Final_TxTime(5)
#define TOF_REPORT_MSG_LEN                  7               // FunctionCode(1), Measured_TOF_Time(5), Range number(1)

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL


#define ANCHOR_BASE_ADDR				(0x8000)

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define INST_DONE_WAIT_FOR_NEXT_EVENT   	1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_NOT_DONE_YET               	0   //this signifies that the instance is still processing the current event

//application data message byte offsets
#define FCODE                               0               // Function code is 1st byte of messageData
#define PTXT                                1
#define RRXT                                6
#define FTXT                                11
#define TOFR                                1
#define TOFRN								6				// range number
#define RES_R1                              1               // Response tag sleep correction LSB
#define RES_R2                              2               // Response tag sleep correction MSB
#define RES_T1                              3               // Ranging request response delay low byte
#define RES_T2                              4               // Ranging request response delay high byte
#define POLL_RNUM                           1               // Poll message range number
#define POLL_PNUM                           2               // Poll message poll number

#define DELAYRX_WAIT4REPORT		(210)   //this is the time in us the RX turn on is delayed (after Final transmission and before Report reception starts)
//response delay time (Tag or Anchor when sending Final/Response messages respectively)
#define FIXED_REPLY_DELAY_110K				5000 //us response time
#define FIXED_REPLY_DELAY_6M81				375 //us response time


typedef enum instanceModes{LISTENER, TAG, ANCHOR, NUM_MODES} INST_MODE;

//Listener = in this mode, the instance only receives frames, does not respond
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above

typedef enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1
    TA_TXPOLL_WAIT_SEND,        //2
    TA_TXFINAL_WAIT_SEND,       //3
    TA_TXRESPONSE_WAIT_SEND,    //4
    TA_TXREPORT_WAIT_SEND,      //5
    TA_TX_WAIT_CONF,            //6

    TA_RXE_WAIT,                //7
    TA_RX_WAIT_DATA,            //8

    TA_SLEEP_DONE               //9

} INST_STATES;


// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 fcs[2] ;                              	//  03-04  CRC
} ack_msg ;

typedef struct
{
    uint8 channelNumber ;       // valid range is 1 to 11
    uint8 preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8 pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 preambleLen ;         // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO;  //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t ;

typedef struct
{
    uint16 anchorAddress;
    uint16 tagAddress;
} instanceAddressConfig_t ;

typedef struct
{
	uint32 icid;

	dwt_rxdiag_t diag;

#if DECA_LOG_ENABLE==1
    int         accumLogging ;                                // log data to a file, used to indicate that we are currenty logging (file is open)
	FILE        *accumLogFile ;                               // file
#endif

} devicelogdata_t ;

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define MAX_EVENT_NUMBER (10)
//NOTE: Accumulators don't need to be stored as part of the event structure as when reading them only one RX event can happen...
//the receiver is singly buffered and will stop after a frame is received

typedef struct
{
	uint8  type;			// event type
	uint8  type2;			// holds the event type - does not clear (not used to show if event has been processed)
	//uint8  broadcastmsg;	// specifies if the rx message is broadcast message
	uint16 rxLength ;

	uint64 timeStamp ;		// last timestamp (Tx or Rx)

	uint32 timeStamp32l ;		   // last tx/rx timestamp - low 32 bits
	uint32 timeStamp32h ;		   // last tx/rx timestamp - high 32 bits

	union {
			//holds received frame (after a good RX frame event)
			uint8   frame[STANDARD_FRAME_SIZE];
    		srd_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_msg_dssl rxmsg_sl ;
			srd_msg_dlss rxmsg_ls ;
			srd_msg_dsss rxmsg_ss ; //16 bit addresses
			ack_msg rxackmsg ; //holds received ACK frame
	}msgu;

	//uint32 eventtime ;
	//uint32 eventtimeclr ;
	uint8 gotit;
}event_data_t ;

#define RTD_MED_SZ          8      // buffer size for mean of 8

typedef struct{
	uint32 diffRmP;
	uint32 diffFmR;
} rtd_t;

typedef struct {
                uint8 PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; //
}tx_struct;

typedef struct {
				uint32 evbID;
                uint32 dwID32bit;
                uint16 delay[4][2]; // 4 modes (tag, anchor)
}antennaCal_t;

//Calibrated antenna values for 10 test EVB1000
//Table contains EVB number, IC ID (32 bit), Mode 1 Tag, Mode 1 Anchor, Mode 2 Tag etc...
extern const antennaCal_t rfDelaysCalTest[12] ;

typedef struct
{
    INST_MODE mode;				//instance mode (tag or anchor)

    INST_STATES testAppState ;			//state machine - current state
    INST_STATES nextState ;				//state machine - next state
    INST_STATES previousState ;			//state machine - previous state
    int done ;					//done with the current event/wait for next event to arrive

	//configuration structures
	dwt_config_t    configData ;	//DW1000 channel configuration
	dwt_txconfig_t  configTX ;		//DW1000 TX power configuration
	uint16			txantennaDelay ; //DW1000 TX antenna delay
	uint16			rxantennaDelay ; //DW1000 RX antenna delay
	uint32 			txpower ;		 //DW1000 TX power
	uint8 txpowerChanged ;
	uint8 antennaDelayChanged;
	// "MAC" features
	uint8 rxautoreenable;			//auto RX re-enable (the receiver will re-enable on an errored frame)

	//user payload and address structure for non-discovery mode
    instanceAddressConfig_t payload ;

	//timeouts and delays
	//this is the delay used for the delayed transmit (when sending the ranging init, response, and final messages)
	uint64 fixedReplyDelay ;

	// xx_sy the units are 1.0256 us
	int fixedReplyDelay_sy ;    // this is the delay used after sending a poll or response and turning on the receiver to receive response or final

	int fwtoTime_sy ;	//this is final message duration (longest out of ranging messages)

	uint64 delayedReplyTime;		// delayed reply time of ranging-init/response/final message

    uint32 rxTimeouts ;

    //message structures used for transmitted messages
    srd_msg_dlsl msg ;			// simple 802.15.4 frame structure (used for tx message) - using long addresses

//messages used in "fast" ranging ...
	srd_msg_dsss msg_f ; // ranging message with 16-bit addresses - used for "fast" ranging

	//Tag function address/message configuration
	uint8   eui64[8];				// devices EUI 64-bit address
	uint16  psduLength ;			// used for storing the frame length
    uint8   frame_sn;				// modulo 256 frame sequence number - it is incremented for each new frame transmission
	uint16  panid ;					// panid used in the frames

    uint8 relpyAddress[8] ;

	//64 bit timestamps
	//union of TX timestamps
	union {
		uint64 txTimeStamp ;		   // last tx timestamp
		uint64 tagPollTxTime ;		   // tag's poll tx timestamp
	    uint64 anchorRespTxTime ;	   // anchor's reponse tx timestamp
	}txu;
	uint64 anchorRespRxTime ;	    // receive time of response message
	uint64 tagPollRxTime ;          // receive time of poll message


	//application control parameters
	uint8	ackreq;					// set if the last RX message had ACK request bit set - it is cleared when the ACK tx confirmation is processed
									// the received frame with the ACK request bit set will only be processed once the ACK has been sent
    uint8	wait4ack ;				// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion

	uint8   instToSleep;			// if set the instance will go to sleep before sending the blink/poll message

	uint8	gotTO;					// got timeout event

	uint8   responseRxNum;			// response number
	uint8	rangeNum;				// incremented for each sequence of ranges

    //diagnostic counters/data, results and logging
    uint32 tof32 ;
    uint64 tof ;

    int longTermRangeCount ;
    uint8 lastReportSN ;
    int last_update ;           // detect changes to status report

    double idistance ; // instantaneous distance
    int newrange;
    int newrangeancaddress; //last 4 bytes of anchor address
    int newrangetagaddress; //last 4 bytes of tag address

    int newrangetime;

    uint8 anchorListIndex ;

	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
	event_data_t saved_dwevent; //holds an RX event while the ACK is being sent
    uint8 dweventIdxOut;
    uint8 dweventIdxIn;
	uint8 dweventPeek;

	uint32 listen_begin_time;

	int dwIDLE;

} instance_data_t ;

//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to calculate and report the Time of Flight to the GUI/display
void reportTOF(instance_data_t *inst);

// clear the status/ranging data
void instanceclearcounts(void) ;

void instance_readaccumulatordata(void);
//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------

void instance_close(void);
// Call init, then call config, then call run. call close when finished
// initialise the instance (application) structures and DW1000 device
int instance_init(void);
// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config) ;

// configure the payload and MAC address
void instancesetaddresses(instanceAddressConfig_t *plconfig) ;

void inst_processrxtimeout(instance_data_t *inst);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int instance_run(void) ;       // returns indication of status report change
int testapprun(instance_data_t *inst, int message);

// calls the DW1000 interrupt handler
#define instance_process_irq(x) 	dwt_isr()  //call device interrupt handler
// configure TX/RX callback functions that are called from DW1000 ISR
void instance_rxcallback(const dwt_callback_data_t *rxd);
void instance_txcallback(const dwt_callback_data_t *txd);

// sets the Tag sleep delay time (the time Tag "sleeps" between each ranging attempt)
void instancesetreplydelay(int delayms);

// set/get the instance roles e.g. Tag/Anchor/Listener
void instancesetrole(int mode) ;                //
int instancegetrole(void) ;
// get the DW1000 device ID (e.g. 0xDECA0130 for DW1000)
uint32 instancereaddeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence

double instancegetidist(void);
double instancegetidistraw(void);

int instancegetrnum(void);
int instancegetlcount(void);

int instancenewrangeancadd(void);
int instancenewrangetagadd(void);
int instancenewrangepolltim(void);
int instancenewrange(void);
int instancenewrangetim(void);

uint64 convertmicrosectodevicetimeu (double microsecu);
double convertdevicetimetosec(int32 dt);

#define DWT_PRF_64M_RFDLY   (514.462f)
#define DWT_PRF_16M_RFDLY   (513.9067f)
const uint16 rfDelays[2];
const uint16 rfDelaysTREK[2];
const tx_struct txSpectrumConfig[8];

instance_data_t instance_data[NUM_INST] ;


int instance_peekevent(void);

void instance_saveevent(event_data_t newevent);

event_data_t instance_getsavedevent(void);

void instance_putevent(event_data_t newevent);

event_data_t* instance_getevent(int x);

void instance_clearevents(void);

void instance_notify_DW1000_inIDLE(int idle);

// configure TX power
void instanceconfigtxpower(uint32 txpower);
void instancesettxpower(void);

// configure the antenna delays
void instanceconfigantennadelays(uint16 tx, uint16 rx);
void instancesetantennadelays(void);
uint16 instancetxantdly(void);
uint16 instancerxantdly(void);

int instance_starttxtest(int framePeriod);
#ifdef __cplusplus
}
#endif

#endif
