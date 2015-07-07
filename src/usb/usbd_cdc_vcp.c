/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED 
#pragma     data_alignment = 4 
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_vcp.h"
#include "usb_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };


USART_InitTypeDef USART_InitStructure;

extern int local_buff_length;
extern uint8_t local_buff[];
extern int local_have_data;
extern uint16_t local_buff_offset;


/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */
extern uint32_t APP_Rx_length;


/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init     (void);
static uint16_t VCP_DeInit   (void);
static uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
//static uint16_t VCP_DataTx   (uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx   (uint8_t* Buf, uint32_t Len);

static uint16_t VCP_COMConfig(uint8_t Conf);

CDC_IF_Prop_TypeDef VCP_fops = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  VCP_Init
  *         Initializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Init(void)
{
	  // BV: Not calling physical USART initialisation
	  // Suggest we do not use this mechanism for any SPI configurations.  But used some sort of serial commands.
	  // Not sure if it affects the data but just in case we should set 8N1 to ensure full 8-bit data transfer is done.
#if 0
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* EVAL_COM1 default configuration */
  /* EVAL_COM1 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - Parity Odd
        - Hardware flow control disabled
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Odd;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure and enable the USART */
  STM_EVAL_COMInit(COM1, &USART_InitStructure);

  /* Enable the USART Receive interrupt */
  USART_ITConfig(EVAL_COM1, USART_IT_RXNE, ENABLE);

  /* Enable USART Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EVAL_COM1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif
  
  return USBD_OK;
}

/**
  * @brief  VCP_DeInit
  *         DeInitializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_DeInit(void)
{

  return USBD_OK;
}


/**
  * @brief  VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Not  needed for this driver */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Not  needed for this driver */
    break;

  case SET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case GET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case CLEAR_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case SET_LINE_CODING:
    linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
    linecoding.format = Buf[4];
    linecoding.paritytype = Buf[5];
    linecoding.datatype = Buf[6];
    /* Set the new configuration */
    VCP_COMConfig(OTHER_CONFIG);
    break;

  case GET_LINE_CODING:
    Buf[0] = (uint8_t)(linecoding.bitrate);
    Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
    Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
    Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
    Buf[4] = linecoding.format;
    Buf[5] = linecoding.paritytype;
    Buf[6] = linecoding.datatype; 
    break;

  case SET_CONTROL_LINE_STATE:
    /* Not  needed for this driver */
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;    
    
  default:
    break;
  }

  return USBD_OK;
}

/**
  * @brief  VCP_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in 
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
  */
uint16_t VCP_DataTx (uint8_t* Buf, uint32_t Len)
{
	int i = 0;
// BV: Not calling physical USART RX to get data for sending on USB... Will need to integrate SPI here !!!

	i = APP_Rx_ptr_in ;


	if ((APP_Rx_length+Len) < APP_RX_DATA_SIZE)
	{
		int l;
		/* Get the data to be sent */
		for (l = 0; l < Len; l++)
		{
			APP_Rx_Buffer[i++] = Buf[l];
			/* Increment the in pointer */
			if (i>=APP_RX_DATA_SIZE)
				i=0;
		}
	}

	APP_Rx_ptr_in = i;

	return USBD_OK;
}

/**
  * @brief  VCP_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         until exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (i.e. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
  uint32_t i;

  // BV: Not calling physical USART TX to send data ... Will need to integrate SPI TX here !!!
  // ZS: This is where PC (USB Tx) data is received
  for (i = 0; i < Len; i++)
  {
	  if((i + local_buff_offset) < 10000)
	  {
		  local_buff[i + local_buff_offset] = Buf[i];
	  }
  }

  local_buff_length = Len + local_buff_offset;
  //

  if((local_buff[2] + (local_buff[3]<<8)) == local_buff_length)
  {
	  local_buff_offset = 0;
	  local_have_data = 1;
  }
  else
  {
	   if((local_buff_length == 5) &&(local_buff[0] == 100))
	   {
		   local_buff_offset = 0;
		   local_have_data = 1;
	   }
	   else if((Len == 5) &&(Buf[0] == 100)) //reset the buffer
	   {
		   local_buff_offset = 0;
		   local_have_data = 1;
		   for (i = 0; i < Len; i++)
		   {
			   local_buff[i + local_buff_offset] = Buf[i];
		   }
	   }
	   //we have received a antenna calibration or tx power configuration command
	   else if(local_buff_length == 6)
	   {
		   if((local_buff[0] == 0x5) && (local_buff[5] == 0x5)
			   || (local_buff[0] == 0x7) && (local_buff[5] == 0x7))
		   {
			   local_buff_offset = 0;
			   local_have_data = 1;
		   }
	   }
	   //we have received a S1 configuration command
	   else if((local_buff_length == 3) && (local_buff[0] == 0x6) && (local_buff[2] == 0x6))
	   {
	       local_buff_offset = 0;
	   	   local_have_data = 1;
	   }
	   else
	   {
		   local_buff_offset += Len;
	   }
  }



  return USBD_OK;
}

/**
  * @brief  VCP_COMConfig
  *         Configure the COM Port with default values or values received from host.
  * @param  Conf: can be DEFAULT_CONFIG to set the default configuration or OTHER_CONFIG
  *         to set a configuration received from the host.
  * @retval None.
  */
static uint16_t VCP_COMConfig(uint8_t Conf)
{

// BV: Not calling physical USART configuration
// Suggest we do not use this mechanism for any SPI configurations.  But used some sort of serial commands.
// Not sure if it affects the data but just in case we should set 8N1 to ensure full 8-bit data transfer is done.

#if 0

  if (Conf == DEFAULT_CONFIG)  
  {
    /* EVAL_COM1 default configuration */
    /* EVAL_COM1 configured as follow:
    - BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - Parity Odd
    - Hardware flow control disabled
    - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Odd;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Configure and enable the USART */
    STM_EVAL_COMInit(COM1, &USART_InitStructure);
    
    /* Enable the USART Receive interrupt */
    USART_ITConfig(EVAL_COM1, USART_IT_RXNE, ENABLE);
  }
  else
  {
    /* set the Stop bit*/
    switch (linecoding.format)
    {
    case 0:
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      break;
    case 1:
      USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
      break;
    case 2:
      USART_InitStructure.USART_StopBits = USART_StopBits_2;
      break;
    default :
      VCP_COMConfig(DEFAULT_CONFIG);
      return (USBD_FAIL);
    }
    
    /* set the parity bit*/
    switch (linecoding.paritytype)
    {
    case 0:
      USART_InitStructure.USART_Parity = USART_Parity_No;
      break;
    case 1:
      USART_InitStructure.USART_Parity = USART_Parity_Even;
      break;
    case 2:
      USART_InitStructure.USART_Parity = USART_Parity_Odd;
      break;
    default :
      VCP_COMConfig(DEFAULT_CONFIG);
      return (USBD_FAIL);
    }
    
    /*set the data type : only 8bits and 9bits is supported */
    switch (linecoding.datatype)
    {
    case 0x07:
      /* With this configuration a parity (Even or Odd) should be set */
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      break;
    case 0x08:
      if (USART_InitStructure.USART_Parity == USART_Parity_No)
      {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      }
      else 
      {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
      }
      
      break;
    default :
      VCP_COMConfig(DEFAULT_CONFIG);
      return (USBD_FAIL);
    }
    
    USART_InitStructure.USART_BaudRate = linecoding.bitrate;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Configure and enable the USART */
    STM_EVAL_COMInit(COM1, &USART_InitStructure);
  }
#endif

  return USBD_OK;
}

/**
  * @brief  EVAL_COM_IRQHandler
  *         
  * @param  None.
  * @retval None.
  */

// BV: I don't expect COM port interrupt handler to be used, or the vector installed
// Suggest we do not use this mechanism for any SPI configurations.  But used some sort of serial commands.
// Not sure if it affects the data but just in case we should set 8N1 to ensure full 8-bit data transfer is done.

#if 0

void EVAL_COM_IRQHandler(void)
{
  if (USART_GetITStatus(EVAL_COM1, USART_IT_RXNE) != RESET)
  {
    /* Send the received data to the PC Host*/
    VCP_DataTx (0,0);
  }

  /* If overrun condition occurs, clear the ORE flag and recover communication */
  if (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_ORE) != RESET)
  {
    (void)USART_ReceiveData(EVAL_COM1);
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
