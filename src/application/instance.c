/*! ----------------------------------------------------------------------------
 *  @file    instance.c
 *  @brief   DecaWave application level message exchange for ranging demo
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
#include "deca_regs.h"

#include "instance.h"
#include "scheduler.h"
// -------------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
//
// function to construct the message/frame header bytes
//
// -------------------------------------------------------------------------------------------------------------------
//
void instanceconfigframeheader16(instance_data_t *inst)
{
    //set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
    inst->msg_f.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;

	//source/dest addressing modes and frame version
	inst->msg_f.frameCtrl[1] = 0x8 /*dest extended address (16bits)*/ | 0x80 /*src extended address (16bits)*/;

	inst->msg_f.panID[0] = (inst->panid) & 0xff;
	inst->msg_f.panID[1] = inst->panid >> 8;
    inst->msg_f.seqNum = inst->frame_sn++;

}

int destaddress(instance_data_t *inst)
{
 	inst->msg_f.destAddr[0] = inst->anchorListIndex & 0xff;
	inst->msg_f.destAddr[1] = (ANCHOR_BASE_ADDR >> 8);
	inst->anchorListIndex++ ;
    if(inst->anchorListIndex >= MAX_ANCHOR) {
        inst->instToSleep = TRUE ; //we'll sleep after this poll
        inst->anchorListIndex = 0; //start from the first anchor in the list after sleep finishes
    }
    return 0;
}

// -------------------------------------------------------------------------------------------------------------------
//
// function to configure the frame data, prior to writing the frame to the TX buffer
//
// -------------------------------------------------------------------------------------------------------------------
//
void setupmacframedata(instance_data_t *inst, int len, int framectrllen, int fcode)
{
	inst->msg_f.messageData[FCODE] = fcode; //message function code (specifies if message is a poll, response or other...)
    inst->psduLength = len + framectrllen;

	instanceconfigframeheader16(inst);
}


int instancesendpacket(instance_data_t *inst, int delayedTx)
{
    int result = 0;

    dwt_writetxfctrl(inst->psduLength, 0);
    if(delayedTx)
    {
        uint32 dtime;
        dtime = (uint32) (inst->delayedReplyTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

    //begin delayed TX of frame
    if (dwt_starttx(delayedTx | inst->wait4ack))  // delayed start was too late
    {
        result = 1; //late/error
    }


    return result;                                              // state changes
    // after sending we should return to TX ON STATE ?
}

// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine (all the instance modes Tag, Anchor or Listener use the same statemachine....)
//
// -------------------------------------------------------------------------------------------------------------------
//
int testapprun(instance_data_t *inst, int message)
{
    switch (inst->testAppState)
    {
        case TA_INIT :
            // printf("TA_INIT") ;
            switch (inst->mode)
            {
                case TAG:
                {
                	uint16 mode = 0;
                	uint16 shortadd = 0;

                    inst->gotTO = 0;

                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;
                    dwt_setpanid(inst->panid);

                    memcpy(inst->eui64, &inst->payload.tagAddress, ADDR_BYTE_SIZE_S);
                    dwt_seteui(inst->eui64);

                    inst->newrangetagaddress = inst->eui64[0] + (inst->eui64[1] << 8);

                    //set source address into the message structure
                    inst->msg_f.sourceAddr[0] = inst->eui64[0];
                    shortadd = inst->msg_f.sourceAddr[1] = inst->eui64[1];
                    shortadd = (shortadd << 8) + inst->msg_f.sourceAddr[0];
                    dwt_setaddress16(shortadd);

                    //Start off by Sleeping 1st -> set instToSleep to TRUE
                    inst->nextState = TA_TXPOLL_WAIT_SEND;
                    inst->testAppState = TA_TXE_WAIT;
                    inst->instToSleep = FALSE;

                    mode = (DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV);

					if((dwt_getldotune() != 0)) //if we need to use LDO tune value from OTP kick it after sleep
							mode |= DWT_LOADLDO;

					if(inst->configData.txPreambLength == DWT_PLEN_64)  //if using 64 length preamble then use the corresponding OPSet
						mode |= DWT_LOADOPSET;

                }
                break;
                case ANCHOR:
                {
                	uint16 shortadd = 0;

                    memcpy(inst->eui64, &inst->payload.anchorAddress, ADDR_BYTE_SIZE_S);
                    dwt_seteui(inst->eui64);

                    dwt_setpanid(inst->panid);

                    //set source address into the message structure
                    inst->msg_f.sourceAddr[0] = inst->eui64[0];
                    shortadd = inst->msg_f.sourceAddr[1] = inst->eui64[1];
                    shortadd = (shortadd << 8) + inst->msg_f.sourceAddr[0];
                    dwt_setaddress16(shortadd);

                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;

                    // First time anchor listens we don't do a delayed RX
					dwt_setrxaftertxdelay(0);
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;

                    dwt_setrxtimeout(0);

                }
                break;
                case LISTENER:
                {
                    dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
					dwt_setrxaftertxdelay(0); //no delay of turning on of RX
                    dwt_setrxtimeout(0);
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;
                    //inst->listen_begin_time = portGetTickCount();
                }
                break ; // end case TA_INIT
                default:
                break;
            }
            break; // end case TA_INIT

        case TA_TXE_WAIT : //either go to sleep or proceed to TX a message
            // printf("TA_TXE_WAIT") ;
            //if we are scheduled to go to sleep before next transmission then sleep first.
            if((inst->nextState == TA_TXPOLL_WAIT_SEND)
                    && (inst->instToSleep)  //go to sleep before sending the next poll/ starting new ranging exchange
                    )
            {
            	MyRangeProcessingRoundFinished();
            	inst->mode = LISTENER;
            	inst->testAppState = TA_INIT;
            	inst->done = INST_NOT_DONE_YET;
            	break;
            }
            else //proceed to configuration and transmission of a frame
            {
                inst->testAppState = inst->nextState;
                inst->nextState = 0; //clear
            }
            break ; // end case TA_TXE_WAIT

        case TA_TXPOLL_WAIT_SEND :
            {

            	// NOTE: For DecaRangeRTLS TREK application, we want to poll in turn anchors 0, 1, 2, and 3.
            	//       The selection of which anchor to poll next is done here !!!!
				destaddress(inst); // Set Anchor Address to poll next
				while (!MyRangeToAnchorShouldStart(inst->msg_f.destAddr[0])){
				}


                inst->msg_f.messageData[POLL_RNUM] = inst->rangeNum;
                inst->msg_f.messageData[POLL_PNUM] = 0;
                setupmacframedata(inst, TAG_POLL_MSG_LEN, FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC, RTLS_DEMO_MSG_TAG_POLL);
                dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

				//set the delayed rx on time (the response message will be sent after this delay)
				dwt_setrxaftertxdelay((uint32)inst->fixedReplyDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
				dwt_setrxtimeout((uint16)inst->fwtoTime_sy);  //units are us - wait for 7ms after RX on (but as using delayed RX this timeout should happen at response time + 7ms)

				//response is expected
				inst->wait4ack = DWT_RESPONSE_EXPECTED;

				dwt_writetxfctrl(inst->psduLength, 0);

				dwt_starttx(DWT_START_TX_IMMEDIATE | inst->wait4ack);

                inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
                inst->previousState = TA_TXPOLL_WAIT_SEND ;
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)

            }
            break;

        case TA_TXRESPONSE_WAIT_SEND :
        {

				//program option octet and parameters (not used currently)
				inst->msg_f.messageData[RES_R1] = 0;
				inst->msg_f.messageData[RES_R2] = 0;

				setupmacframedata(inst, ANCH_RESPONSE_MSG_LEN, FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC, RTLS_DEMO_MSG_ANCH_RESP);
				dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data


                inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                inst->previousState = TA_TXRESPONSE_WAIT_SEND ;

				//set the delayed rx on time (the final message will be sent after this delay)
				dwt_setrxaftertxdelay((uint32)inst->fixedReplyDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

				//response is expected
				inst->wait4ack = DWT_RESPONSE_EXPECTED;

                if(instancesendpacket(inst, DWT_START_TX_DELAYED))
                {
                    inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new poll
					dwt_setrxaftertxdelay(0);
					inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
                }
                else
                {
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;  //no timeout
                }
            }
            break;

        case TA_TXFINAL_WAIT_SEND :
            {
                uint64 tagCalculatedFinalTxTime ;
                // Calculate Time Final message will be sent and write this field of Final message
                // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
                // zeroing its low 9 bits, and then having the TX antenna delay added
                tagCalculatedFinalTxTime = inst->delayedReplyTime & MASK_TXDTS; // 9 lower bits mask

                // getting antenna delay from the device and add it to the Calculated TX Time
                tagCalculatedFinalTxTime = tagCalculatedFinalTxTime + inst->txantennaDelay;

                tagCalculatedFinalTxTime &= MASK_40BIT;

                // Embed into Final message: 40-bit pollTXTime,  40-bit respRxTime,  40-bit finalTxTime
                // Write Poll TX time field of Final message
				memcpy(&(inst->msg_f.messageData[PTXT]), (uint8 *)&inst->txu.tagPollTxTime, 5);

                // Write Response RX time field of Final message
				memcpy(&(inst->msg_f.messageData[RRXT]), (uint8 *)&inst->anchorRespRxTime, 5);

                // Write Calculated TX time field of Final message
				memcpy(&(inst->msg_f.messageData[FTXT]), (uint8 *)&tagCalculatedFinalTxTime, 5);

				setupmacframedata(inst, TAG_FINAL_MSG_LEN, FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC, RTLS_DEMO_MSG_TAG_FINAL);
				dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

				//turn on the receiver to receive the report... as it is coming after the final
				inst->wait4ack = DWT_RESPONSE_EXPECTED;
				dwt_setrxaftertxdelay(DELAYRX_WAIT4REPORT); //the report will not come sooner than 200 us after the final... this is platform dependent
				//the delay here is defined by observation


                instancesendpacket(inst, DWT_START_TX_DELAYED);
                inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                inst->previousState = TA_TXFINAL_WAIT_SEND;
                //if waiting for report - set timeout
				dwt_setrxtimeout((uint16)inst->fwtoTime_sy); //
				inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)
            }
            break;

        case TA_TXREPORT_WAIT_SEND :
            {

				// Write calculated TOF into report message
				memcpy(&(inst->msg_f.messageData[TOFR]), &inst->tof, 5);

				inst->msg_f.messageData[TOFRN] = inst->rangeNum;

				setupmacframedata(inst, TOF_REPORT_MSG_LEN, FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC, RTLS_DEMO_MSG_ANCH_TOFR);

				dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

				//set the delayed rx on time (the ranging init will be sent after this delay)
				//subtract 1ms to make sure the receiver is on before the message comes in
				dwt_setrxaftertxdelay(0);  //units are ~us - wait for wait4respTIM before RX on (delay RX)

				//response is expected
				inst->wait4ack = DWT_RESPONSE_EXPECTED;
				//anchor - we don't use timeout

				dwt_writetxfctrl(inst->psduLength, 0);

				dwt_starttx(DWT_START_TX_IMMEDIATE | inst->wait4ack);

				inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
				inst->previousState = TA_TXREPORT_WAIT_SEND ;

				//use anchor rx timeout to timeout and re-send the ToF report
				inst->done = INST_NOT_DONE_YET;

            }
            break;


        case TA_TX_WAIT_CONF :
		   //printf("TA_TX_WAIT_CONF %d m%d %d states %08x %08x\n", inst->previousState, message, inst->newReportSent, dwt_read32bitreg(0x19), dwt_read32bitreg(0x0f)) ;

                {
				event_data_t* dw_event = instance_getevent(11); //get and clear this event

                //NOTE: Can get the ACK before the TX confirm event for the frame requesting the ACK
                //this happens because if polling the ISR the RX event will be processed 1st and then the TX event
                //thus the reception of the ACK will be processed before the TX confirmation of the frame that requested it.
				if(dw_event->type != DWT_SIG_TX_DONE) //wait for TX done confirmation
                {
					if(dw_event->type != 0)
					{
						if(dw_event->type == DWT_SIG_RX_TIMEOUT) //got RX timeout - i.e. did not get the response (e.g. ACK)
						{
							//printf("RX timeout in TA_TX_WAIT_CONF (%d)\n", inst->previousState);
							//we need to wait for SIG_TX_DONE and then process the timeout and re-send the frame if needed
							inst->gotTO = 1;
						}
						if(dw_event->type == SIG_RX_ACK)
						{
							inst->wait4ack = 0 ; //clear the flag as the ACK has been received
							//printf("RX ACK in TA_TX_WAIT_CONF... wait for TX confirm before changing state (%d)\n", inst->previousState);
						}
					}
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
                        break;

                }

                inst->done = INST_NOT_DONE_YET;

                if (inst->gotTO) //timeout
                {
					//printf("got TO in TA_TX_WAIT_CONF\n");
                    inst_processrxtimeout(inst);
                    inst->gotTO = 0;
					inst->wait4ack = 0 ; //clear this
					break;
                }
                else
                {
					inst->txu.txTimeStamp = dw_event->timeStamp;

                    inst->testAppState = TA_RXE_WAIT ;                      // After sending, tag expects response/report, anchor waits to receive a final/new poll
                    //fall into the next case (turn on the RX)
					message = 0;
                }

            }

            //break ; // end case TA_TX_WAIT_CONF


        case TA_RXE_WAIT :
        // printf("TA_RXE_WAIT") ;
        {

            if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
            {
                //turn RX on
            	dwt_rxenable(0) ;  // turn RX on, without delay
            }
            else
            {
                inst->wait4ack = 0 ; //clear the flag, the next time we want to turn the RX on it might not be auto
            }

            if (inst->mode != LISTENER)
            {
                if (inst->previousState != TA_TXREPORT_WAIT_SEND) //we are going to use anchor timeout and re-send the report
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO
            }

            inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it

            // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
            if(message == 0) break;
        }

        case TA_RX_WAIT_DATA :                                                                     // Wait RX data
		   //printf("TA_RX_WAIT_DATA %d", message) ;

            switch (message)
            {

                case SIG_RX_ACK :
                {
					//event_data_t* dw_event = instance_getevent(14); //get and clear this event
					instance_getevent(14); //get and clear this event
                    //else we did not expect this ACK turn the RX on again
                    inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                    inst->done = INST_NOT_DONE_YET;
                }
                break;

				//if we have received a DWT_SIG_RX_OKAY event - this means that the message is IEEE data type - need to check frame control to know which addressing mode is used
                case DWT_SIG_RX_OKAY :
                {
					event_data_t* dw_event = instance_getevent(15); //get and clear this event
					uint8  srcAddr[8] = {0,0,0,0,0,0,0,0};
					uint8  dstAddr[8] = {0,0,0,0,0,0,0,0};
                    int fcode = 0;
					int fn_code = 0;
					//int srclen = 0;
					//int fctrladdr_len;
					uint8 *messageData;

                    // handle 16 and 64 bit source and destination addresses
					switch(dw_event->msgu.frame[1] & 0xCC)
					{
						case 0xCC: //
							memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ll.sourceAddr[0]), ADDR_BYTE_SIZE_L);
							memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_ll.destAddr[0]), ADDR_BYTE_SIZE_L);
							fn_code = dw_event->msgu.rxmsg_ll.messageData[FCODE];
							messageData = &dw_event->msgu.rxmsg_ll.messageData[0];
							//srclen = ADDR_BYTE_SIZE_L;
							//fctrladdr_len = FRAME_CRTL_AND_ADDRESS_L;
							break;
						case 0xC8: //
							memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_sl.sourceAddr[0]), ADDR_BYTE_SIZE_L);
							memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_sl.destAddr[0]), ADDR_BYTE_SIZE_S);
							fn_code = dw_event->msgu.rxmsg_sl.messageData[FCODE];
							messageData = &dw_event->msgu.rxmsg_sl.messageData[0];
							//srclen = ADDR_BYTE_SIZE_L;
							//fctrladdr_len = FRAME_CRTL_AND_ADDRESS_LS;
							break;
						case 0x8C: //
							memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ls.sourceAddr[0]), ADDR_BYTE_SIZE_S);
							memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_ls.destAddr[0]), ADDR_BYTE_SIZE_L);
							fn_code = dw_event->msgu.rxmsg_ls.messageData[FCODE];
							messageData = &dw_event->msgu.rxmsg_ls.messageData[0];
							//srclen = ADDR_BYTE_SIZE_S;
							//fctrladdr_len = FRAME_CRTL_AND_ADDRESS_LS;
							break;
						case 0x88: //
							memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
							memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_ss.destAddr[0]), ADDR_BYTE_SIZE_S);
							fn_code = dw_event->msgu.rxmsg_ss.messageData[FCODE];
							messageData = &dw_event->msgu.rxmsg_ss.messageData[0];
							//srclen = ADDR_BYTE_SIZE_S;
							//fctrladdr_len = FRAME_CRTL_AND_ADDRESS_S;
							break;
					}


                    {
						//non - discovery mode - association is not used, process all messages
						fcode = fn_code;

                        switch(fcode)
                        {

                            case RTLS_DEMO_MSG_TAG_POLL:
                            {
								if((inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
										|| ((dstAddr[0] != inst->eui64[0]) || (dstAddr[1] != inst->eui64[1]))) //if not addressed to us
								{
									inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
									break;
								}

                                inst->rangeNum = messageData[POLL_RNUM];

								inst->tagPollRxTime = dw_event->timeStamp ; //Poll's Rx time

                                inst->delayedReplyTime = inst->tagPollRxTime + inst->fixedReplyDelay ;  // time we should send the response
                                inst->delayedReplyTime &= MASK_TXDTS ;

                                inst->testAppState = TA_TXRESPONSE_WAIT_SEND ; // send our response


								inst->msg_f.destAddr[0] = srcAddr[0];
								inst->msg_f.destAddr[1] = srcAddr[1];

                            }
                            break; //RTLS_DEMO_MSG_TAG_POLL

                            case RTLS_DEMO_MSG_ANCH_RESP:
                            {

								if((inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
										|| ((dstAddr[0] != inst->eui64[0]) || (dstAddr[1] != inst->eui64[1]))) //if not addressed to us
								{
                                    inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                    break;
                                }

								inst->anchorRespRxTime = dw_event->timeStamp ; //Response's Rx time

								inst->delayedReplyTime = inst->anchorRespRxTime + inst->fixedReplyDelay ;  // time we should send the response
                                inst->delayedReplyTime &= MASK_TXDTS ;

                                inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final

								inst->relpyAddress[0] = srcAddr[0];
								inst->relpyAddress[1] = srcAddr[1];

                            }
                            break; //RTLS_DEMO_MSG_ANCH_RESP

                            case RTLS_DEMO_MSG_ANCH_TOFR:
                            {
                                if(inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
                                {
                                    inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                	RangeProcessingDetected(dstAddr[0], srcAddr[0], fcode, inst->mode);
                                    break;
                                }


								memcpy(&inst->tof, &(messageData[TOFR]), 5);

								if(inst->mode == TAG){
                                    reportTOF(inst);

									inst->lastReportSN = dw_event->msgu.frame[2];
									inst->newrangeancaddress = srcAddr[0] + ((uint16) srcAddr[1] << 8);
									inst->newrangetagaddress = dstAddr[0]  + ((uint16) dstAddr[1]  << 8);
									ReportRangeResult(dstAddr[0], srcAddr[0], instancegetidist()*1000);
                                    inst->testAppState = TA_TXE_WAIT;
                                    inst->nextState = TA_TXPOLL_WAIT_SEND ; // send next poll
                                } else {
									inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
								}
                            }
                            break; //RTLS_DEMO_MSG_ANCH_TOFR

                            case RTLS_DEMO_MSG_TAG_FINAL:
                            {
                                uint64 tRxT, tTxT, aRxT, aTxT ;
                                uint64 tagFinalTxTime  = 0;
                                uint64 tagFinalRxTime  = 0;
                                uint64 tagPollTxTime  = 0;
                                uint64 anchorRespRxTime  = 0;
                                uint64 pollRespRTD  = 0;
                                uint64 respFinalRTD  = 0;

								if((inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
										|| ((dstAddr[0] != inst->eui64[0]) || (dstAddr[1] != inst->eui64[1]))) //if not addressed to us
								{
                                    inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                    break;
                                }

                                // time of arrival of Final message
								tagFinalRxTime = dw_event->timeStamp ; //Final's Rx time

								//printf("FinalRx Timestamp: %4.15e\n", convertdevicetimetosecu(dw_event.timeStamp));
                                inst->delayedReplyTime = 0 ;

                                // times measured at Tag extracted from the message buffer
                                // extract 40bit times
								memcpy(&tagPollTxTime, &(messageData[PTXT]), 5);
								memcpy(&anchorRespRxTime, &(messageData[RRXT]), 5);
								memcpy(&tagFinalTxTime, &(messageData[FTXT]), 5);

                                // poll response round trip delay time is calculated as
                                // (anchorRespRxTime - tagPollTxTime) - (anchorRespTxTime - tagPollRxTime)
                                aRxT = (anchorRespRxTime - tagPollTxTime) & MASK_40BIT;
                                aTxT = (inst->txu.anchorRespTxTime - inst->tagPollRxTime) & MASK_40BIT;
                                pollRespRTD = (aRxT - aTxT) & MASK_40BIT;


                                // response final round trip delay time is calculated as
                                // (tagFinalRxTime - anchorRespTxTime) - (tagFinalTxTime - anchorRespRxTime)
                                tRxT = (tagFinalRxTime - inst->txu.anchorRespTxTime) & MASK_40BIT;
                                tTxT = (tagFinalTxTime - anchorRespRxTime) & MASK_40BIT;
                                respFinalRTD = (tRxT - tTxT) & MASK_40BIT;

                                // add both round trip delay times
                                inst->tof = ((pollRespRTD + respFinalRTD) & MASK_40BIT);

                                reportTOF(inst);

                                inst->newrangetagaddress = srcAddr[0] + ((uint16) srcAddr[1] << 8);
                                inst->newrangeancaddress = inst->eui64[0] + ((uint16) inst->eui64[1] << 8);

								inst->testAppState = TA_TXREPORT_WAIT_SEND ; // send the report with the calculated time of flight

								instancesetantennadelays(); //this will update the antenna delay if it has changed
					            instancesettxpower(); // configure TX power if it has changed
                            }
                            break; //RTLS_DEMO_MSG_TAG_FINAL


                            default:
                            {
                                //only enable receiver when not using double buffering
                                inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
								dwt_setrxaftertxdelay(0);

                            }
                            break;
						} //end switch (fcode)

						if(dw_event->msgu.frame[0] & 0x20)
						{
							//as we only pass the received frame with the ACK request bit set after the ACK has been sent
							instance_getevent(16); //get and clear the ACK sent event
						}
					} //end else

					if((inst->instToSleep == FALSE) && (inst->mode == LISTENER) /*|| (inst->mode == ANCHOR)*/)//update received data, and go back to receiving frames
					{

						inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
						dwt_setrxaftertxdelay(0);
					}

                }
				break ; //end of DWT_SIG_RX_OKAY

                case DWT_SIG_RX_TIMEOUT :
					instance_getevent(17); //get and clear this event
					//printf("PD_DATA_TIMEOUT %d\n", inst->previousState) ;
                    inst_processrxtimeout(inst);
                    message = 0; //clear the message as we have processed the event
                break ;

				case 0:
                    if(inst->mode == LISTENER && inst->payload.tagAddress < MAX_TAG) // listen enough?
                    {
                        if (MyRangeTurnShouldStart()) {
                        	inst->mode = TAG;
                        	inst->testAppState = TA_INIT;
                        	inst->done = INST_NOT_DONE_YET;
                        	dwt_forcetrxoff();
                        	break;
                        }
                    }

                case DWT_SIG_TX_AA_DONE: //ignore this event - just process the rx frame that was received before the ACK response
				default :
                {
                    if(inst->done == INST_NOT_DONE_YET) inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
                }
                break;

            }
            break ; // end case TA_RX_WAIT_DATA
            default:
                //printf("\nERROR - invalid state %d - what is going on??\n", inst->testAppState) ;
            break;
    } // end switch on testAppState

    return inst->done;
} // end testapprun()

// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else


// -------------------------------------------------------------------------------------------------------------------
// function to set the fixed reply delay time (in us)
//
// This sets delay for RX to TX - Delayed Send, and for TX to RX delayed receive (wait for response) functionality,
// and the frame wait timeout value to use.  This is a function of data rate, preamble length, and PRF

extern uint8 dwnsSFDlen[];

void instancesetreplydelay(int delayus) //delay in us
{
    int instance = 0;

    int margin = 3000; //2000 symbols

	//configure the rx delay receive delay time, it is dependent on the message length
	float msgdatalen = 0;
	float preamblelen = 0;
	int sfdlen = 0;
	int x = 0;

	//Set the RX timeouts based on the longest expected message - the Final message
	//Poll = 14, Response = 15, Final = 27, Report = 18 bytes
	msgdatalen = TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;

	x = (int) ceil(msgdatalen*8/330.0f);

	msgdatalen = msgdatalen*8 + x*48;

	margin = (TAG_FINAL_MSG_LEN - TAG_POLL_MSG_LEN);

	x = (int) ceil(margin*8/330.0f);

	margin = margin*8 + x*48;

	//assume PHR length is 172308us for 110k and 21539us for 850k/6.81M
	if(instance_data[instance].configData.dataRate == DWT_BR_110K)
    {
		msgdatalen *= 8205.13f;
		msgdatalen += 172308; // PHR length in microseconds

		margin *= 8205.13f;

    }
	else if(instance_data[instance].configData.dataRate == DWT_BR_850K)
    {
		msgdatalen *= 1025.64f;
		msgdatalen += 21539; // PHR length in microseconds

		margin *= 1025.64f;
    }
	else
    {
		msgdatalen *= 128.21f;
		msgdatalen += 21539; // PHR length in microseconds

		margin *= 128.21f;
	}

	//SFD length is 64 for 110k (always)
	//SFD length is 8 for 6.81M, and 16 for 850k, but can vary between 8 and 16 bytes
	sfdlen = dwnsSFDlen[instance_data[instance].configData.dataRate];

	switch (instance_data[instance].configData.txPreambLength)
    {
    case DWT_PLEN_4096 : preamblelen = 4096.0f; break;
    case DWT_PLEN_2048 : preamblelen = 2048.0f; break;
    case DWT_PLEN_1536 : preamblelen = 1536.0f; break;
    case DWT_PLEN_1024 : preamblelen = 1024.0f; break;
    case DWT_PLEN_512  : preamblelen = 512.0f; break;
    case DWT_PLEN_256  : preamblelen = 256.0f; break;
    case DWT_PLEN_128  : preamblelen = 128.0f; break;
    case DWT_PLEN_64   : preamblelen = 64.0f; break;
            }

	//preamble  = plen * (994 or 1018) depending on 16 or 64 PRF
	if(instance_data[instance].configData.prf == DWT_PRF_16M)
	{
		preamblelen = (sfdlen + preamblelen) * 0.99359f;
	}
	else
	{
		preamblelen = (sfdlen + preamblelen) * 1.01763f;
	}


	//set the frame wait timeout time - total time the frame takes in symbols
	instance_data[instance].fwtoTime_sy = 16 + (int)((preamblelen + ((msgdatalen + margin)/1000.0))/ 1.0256);

	//this is the delay used for the delayed transmit (when sending the ranging init, response, and final messages)
	instance_data[instance].fixedReplyDelay = convertmicrosectodevicetimeu (delayus) ;

	//this it the delay used for configuring the receiver on delay (wait for response delay),
	instance_data[instance].fixedReplyDelay_sy = (int) (delayus / 1.0256) - 16 - (int)((preamblelen + (msgdatalen/1000.0))/ 1.0256); //subtract 16 symbols, as receiver has a 16 symbol start up time

}

// -------------------------------------------------------------------------------------------------------------------
//
// Set Payload parameters for the instance
//
// -------------------------------------------------------------------------------------------------------------------
void instancesetaddresses(instanceAddressConfig_t *plconfig)
{
    int instance = 0 ;

    instance_data[instance].payload = *plconfig ;       // copy configurations
}


void instance_readaccumulatordata(void)
{
#if DECA_SUPPORT_SOUNDING==1
    int instance = 0;
    uint16 len = 992 ; //default (16M prf)

    if (instance_data[instance].configData.prf == DWT_PRF_64M)  // Figure out length to read
        len = 1016 ;

    instance_data[instance].buff.accumLength = len ;                                       // remember Length, then read the accumulator data

    len = len*4+1 ;   // extra 1 as first byte is dummy due to internal memory access delay

    dwt_readaccdata((uint8*)&(instance_data[instance].buff.accumData->dummy), len, 0);
#endif  // support_sounding
}

#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
