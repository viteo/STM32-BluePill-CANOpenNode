/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "301/CO_driver.h"

#include "device.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "by_can.h"
#include "misc.h"
#include "generic.h"
/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr)
{
	/* Put CAN module in configuration mode */
	CAN_ITConfig(CANptr, CAN_IT_FMP0, DISABLE);
	CAN_ITConfig(CANptr, CAN_IT_TME, DISABLE);
	CAN_DeInit(CANptr);
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
	/* Put CAN module in normal mode */
	CAN_ITConfig(CANmodule->CANptr, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CANmodule->CANptr, CAN_IT_TME, ENABLE);

	CANmodule->CANnormal = true;
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule, void *CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[], uint16_t txSize,
		uint16_t CANbitRate)
{
	uint16_t i;

	/* verify arguments */
	if (CANmodule == NULL || rxArray == NULL || txArray == NULL)
	{
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	/* Configure object variables */
	CANmodule->CANptr = CANptr;
	CANmodule->rxArray = rxArray;
	CANmodule->rxSize = rxSize;
	CANmodule->txArray = txArray;
	CANmodule->txSize = txSize;
	CANmodule->CANerrorStatus = 0;
	CANmodule->CANnormal = false;
	CANmodule->useCANrxFilters = false;/* microcontroller dependent */
	CANmodule->bufferInhibitFlag = false;
	CANmodule->firstCANtxMessage = true;
	CANmodule->CANtxCount = 0U;
	CANmodule->errOld = 0U;

	for (i = 0U; i < rxSize; i++)
	{
		rxArray[i].ident = 0U;
		rxArray[i].mask = 0xFFFFU;
		rxArray[i].object = NULL;
		rxArray[i].CANrx_callback = NULL;
	}
	for (i = 0U; i < txSize; i++)
	{
		txArray[i].bufferFull = false;
	}

	/* Configure CAN module registers */
	/* Configure CAN timing */
	i = CAN1_Init(CANbitRate);
	if(i == CAN_InitStatus_Failed)
		return CO_ERROR_INVALID_STATE;

	/* Configure CAN module hardware filters */
	if (CANmodule->useCANrxFilters)
	{
		/* CAN module filters are used, they will be configured with */
		/* CO_CANrxBufferInit() functions, called by separate CANopen */
		/* init functions. */
		/* Configure all masks so, that received message must match filter */
	}
	else
	{
		/* CAN module filters are not used, all messages with standard 11-bit */
		/* identifier will be received */
		/* Configure mask 0 so, that all messages with standard identifier are accepted */
		CAN1_Filter_Init();
	}

	/* configure CAN interrupt registers */
	//in main()

	return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
	if (CANmodule != NULL)
	{
		/* turn off the module */
		if(CANmodule->CANptr != NULL)
		{
			CO_CANsetConfigurationMode(CANmodule->CANptr);
		}
	}
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void *object,
		void (*CANrx_callback)(void *object, void *message))
{
	CO_ReturnError_t ret = CO_ERROR_NO;

	if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize))
	{
		/* buffer, which will be configured */
		CO_CANrx_t *buffer = &CANmodule->rxArray[index];

		/* Configure object variables */
		buffer->object = object;
		buffer->CANrx_callback = CANrx_callback;

		/* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
		buffer->ident = ident & 0x07FFU;
		if (rtr)
		{
			buffer->ident |= 0x0800U;
		}
		buffer->mask = (mask & 0x07FFU) | 0x0800U;

		/* Set CAN hardware module filter and mask. */
		if (CANmodule->useCANrxFilters)
		{

		}
	}
	else
	{
		ret = CO_ERROR_ILLEGAL_ARGUMENT;
	}

	return ret;
}

/******************************************************************************/
CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes, bool_t syncFlag)
{
	CO_CANtx_t *buffer = NULL;

	if ((CANmodule != NULL) && (index < CANmodule->txSize))
	{
		/* get specific buffer */
		buffer = &CANmodule->txArray[index];

		/* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
		 * Microcontroller specific. */
		buffer->ident = ((uint32_t) ident & 0x07FFU) | ((uint32_t) (rtr ? 0x8000U : 0U));
		buffer->DLC = noOfBytes & 0xFU;
		buffer->bufferFull = false;
		buffer->syncFlag = syncFlag;
	}

	return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	CO_ReturnError_t err = CO_ERROR_NO;

	/* Verify overflow */
	if (buffer->bufferFull)
	{
		if (!CANmodule->firstCANtxMessage)
		{
			/* don't set error, if bootup message is still on buffers */
			CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
		}
		err = CO_ERROR_TX_OVERFLOW;
	}

	CO_LOCK_CAN_SEND(CANmodule);
	/* if CAN TX buffer is free, copy message to it */
	if (CAN_IsMailboxFree(CANmodule->CANptr) && CANmodule->CANtxCount == 0)
	{
		CANmodule->bufferInhibitFlag = buffer->syncFlag;
		/* copy message and txRequest */
		CanTxMsg txMsg = {0};
		txMsg.StdId = buffer->ident;
		txMsg.DLC = buffer->DLC;
		array_copy_8(txMsg.Data, buffer->data, txMsg.DLC);
		CAN_Transmit(CANmodule->CANptr, &txMsg);
	}
	/* if no buffer is free, message will be sent by interrupt */
	else
	{
		buffer->bufferFull = true;
		CANmodule->CANtxCount++;
	}CO_UNLOCK_CAN_SEND(CANmodule);

	return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
	uint32_t tpdoDeleted = 0U;

	CO_LOCK_CAN_SEND(CANmodule);
	/* Abort message from CAN module, if there is synchronous TPDO.
	 * Take special care with this functionality. */
	uint32_t messageIsOnCanBuffer =
			CAN_TransmitStatus(CANmodule->CANptr, 0)
			? CAN_TransmitStatus(CANmodule->CANptr, 1)
			? CAN_TransmitStatus(CANmodule->CANptr, 2) ? 3 : 2 : 1 : 0;
	if (messageIsOnCanBuffer > 2 && CANmodule->bufferInhibitFlag)
	{
		/* clear TXREQ */
		CAN_CancelTransmit(CANmodule->CANptr, messageIsOnCanBuffer);
		CANmodule->bufferInhibitFlag = false;
		tpdoDeleted = 1U;
	}
	/* delete also pending synchronous TPDOs in TX buffers */
	if (CANmodule->CANtxCount != 0U)
	{
		uint16_t i;
		CO_CANtx_t *buffer = &CANmodule->txArray[0];
		for (i = CANmodule->txSize; i > 0U; i--)
		{
			if (buffer->bufferFull)
			{
				if (buffer->syncFlag)
				{
					buffer->bufferFull = false;
					CANmodule->CANtxCount--;
					tpdoDeleted = 2U;
				}
			}
			buffer++;
		}
	}CO_UNLOCK_CAN_SEND(CANmodule);

	if (tpdoDeleted != 0U)
	{
		CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
	}
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
	uint32_t err;
	rxErrors = CAN_GetReceiveErrorCounter(CANmodule->CANptr);
	txErrors = CAN_GetLSBTransmitErrorCounter(CANmodule->CANptr);

	err = ((uint32_t) txErrors << 16) | ((uint32_t) rxErrors << 8) | overflow;

	if (CANmodule->errOld != err)
	{
		uint16_t status = CANmodule->CANerrorStatus;

		CANmodule->errOld = err;

		if (txErrors >= 256U)
		{
			/* bus off */
			status |= CO_CAN_ERRTX_BUS_OFF;
		}
		else
		{
			/* recalculate CANerrorStatus, first clear some flags */
			status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

			/* rx bus warning or passive */
			if (rxErrors >= 128)
			{
				status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
			}
			else if (rxErrors >= 96)
			{
				status |= CO_CAN_ERRRX_WARNING;
			}

			/* tx bus warning or passive */
			if (txErrors >= 128)
			{
				status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
			}
			else if (rxErrors >= 96)
			{
				status |= CO_CAN_ERRTX_WARNING;
			}

			/* if not tx passive clear also overflow */
			if ((status & CO_CAN_ERRTX_PASSIVE) == 0)
			{
				status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
			}
		}

		if (overflow != 0)
		{
			/* CAN RX bus overflow */
			status |= CO_CAN_ERRRX_OVERFLOW;
		}

		CANmodule->CANerrorStatus = status;
	}
}

/******************************************************************************/
void CO_CANinterruptRx(CO_CANmodule_t *CANmodule)
{
	/* receive interrupt */

	CanRxMsg rcvMsg; /* received message in CAN module */
	uint16_t index; /* index of received message */
	uint32_t rcvMsgIdent; /* identifier of the received message */
	CO_CANrx_t *buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
	bool_t msgMatched = false;

	/* get message from module here */
	CAN_Receive(CANmodule->CANptr, CAN_FIFO0, &rcvMsg);
	rcvMsgIdent = rcvMsg.StdId;
	if (CANmodule->useCANrxFilters)
	{
		/* CAN module filters are used. Message with known 11-bit identifier has */
		/* been received */
		index = 0; /* get index of the received message here. Or something similar */
		if (index < CANmodule->rxSize)
		{
			buffer = &CANmodule->rxArray[index];
			/* verify also RTR */
			if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
			{
				msgMatched = true;
			}
		}
	}
	else
	{
		/* CAN module filters are not used, message with any standard 11-bit identifier */
		/* has been received. Search rxArray form CANmodule for the same CAN-ID. */
		buffer = &CANmodule->rxArray[0];
		for (index = CANmodule->rxSize; index > 0U; index--)
		{
			if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
			{
				msgMatched = true;
				break;
			}
			buffer++;
		}
	}

	/* Call specific function, which will process the message */
	if (msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL))
	{
		buffer->CANrx_callback(buffer->object, &rcvMsg);
	}

	/* Clear interrupt flag */
	//cleared automatically on CAN_Receive()
}

void CO_CANinterruptTx(CO_CANmodule_t *CANmodule)
{
	/* transmit interrupt */

	/* Clear interrupt flag */
	CAN_ClearITPendingBit(CANmodule->CANptr, CAN_IT_TME);

	/* First CAN message (bootup) was sent successfully */
	CANmodule->firstCANtxMessage = false;
	/* clear flag from previous message */
	CANmodule->bufferInhibitFlag = false;
	/* Are there any new messages waiting to be send */
	if (CANmodule->CANtxCount > 0U)
	{
		uint16_t i; /* index of transmitting message */

		/* first buffer */
		CO_CANtx_t *buffer = &CANmodule->txArray[0];
		/* search through whole array of pointers to transmit message buffers. */
		for (i = CANmodule->txSize; i > 0U; i--)
		{
			/* if message buffer is full, send it. */
			if (buffer->bufferFull)
			{
				buffer->bufferFull = false;
				CANmodule->CANtxCount--;

				/* Copy message to CAN buffer */
				CANmodule->bufferInhibitFlag = buffer->syncFlag;

				/* canSend... */
				CO_CANsend(CANmodule, buffer);
				break; /* exit for loop */
			}
			buffer++;
		}/* end of for loop */

		/* Clear counter if no more messages */
		if (i == 0U)
		{
			CANmodule->CANtxCount = 0U;
		}
	}
}

#include "301/CO_SDOclient.h"
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t *buf, size_t bufSize, size_t *readSize)
{
	CO_SDO_return_t SDO_ret;
	CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

	// setup client (this can be skipped, if remote device don't change)
	SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd)
	{
		abortCode = CO_SDO_AB_GENERAL;
	}

	// initiate upload
	SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd)
	{
		abortCode = CO_SDO_AB_GENERAL;
	}

	// upload data
	do
	{
		uint32_t timeDifference_us = 1000;

		SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);
		if (SDO_ret < 0)
		{
			break;
		}

		DWT_Delay_us(timeDifference_us); //todo is it needed?
	} while (SDO_ret > 0);

	// copy data to the user buffer (for long data function must be called
	// several times inside the loop)
	*readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

//	return CO_SDO_AB_NONE;
	return abortCode;
}

CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t *data, size_t dataSize)
{
	CO_SDO_return_t SDO_ret;
	bool_t bufferPartial = false;

	// setup client (this can be skipped, if remote device is the same)
	SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd)
	{
		return -1;
	}

	// initiate download
	SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd)
	{
		return -1;
	}

	// fill data
	size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
	if (nWritten < dataSize)
	{
		bufferPartial = true;
		// If SDO Fifo buffer is too small, data can be refilled in the loop.
	}

	//download data
	do
	{
		uint32_t timeDifference_us = 1000;
		CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

		SDO_ret = CO_SDOclientDownload(SDO_C, timeDifference_us, false, bufferPartial, &abortCode, NULL, NULL);
		if (SDO_ret < 0)
		{
			return abortCode;
		}

		DWT_Delay_us(timeDifference_us); //todo is it needed?
	} while (SDO_ret > 0);

	return CO_SDO_AB_NONE;
}
