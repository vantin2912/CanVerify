/*
 * CanCom.c
 *
 *  Created on: Sep 20, 2022
 *      Author: vanti
 */


#include "CANCom.h"
#include <stdio.h>
#include "cmsis_os.h"

#define CANTransmitID	0x0A2
#define CANRcvID		0x012

static uint8_t MessageCNT = 0;

uint8_t crcTable[256];

void CRCInit(void) {
	uint8_t _crc;
	for (int i = 0; i < 0x100; i++) {
		_crc = i;

		for (uint8_t bit = 0; bit < 8; bit++) _crc = (_crc & 0x80) ? ((_crc << 1) ^ 0x1D) : (_crc << 1);

		crcTable[i] = _crc;
	}
}
uint8_t CalcCRC(uint8_t * buf, uint8_t len) {
        const uint8_t * ptr = buf;
        uint8_t _crc = 0xFF;

        while(len--) _crc = crcTable[_crc ^ *ptr++];

        return ~_crc;
}

static void CAN_ConfigFilter(CAN_HandleTypeDef* hcan)
{
	CAN_FilterTypeDef Filter;
	Filter.FilterActivation = CAN_FILTER_ENABLE;
	Filter.FilterIdHigh = CANRcvID<<5;
	Filter.FilterMode = CAN_FILTERMODE_IDLIST;
	Filter.FilterScale = CAN_FILTERSCALE_16BIT;
	Filter.FilterBank = 1;
	Filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	Filter.SlaveStartFilterBank =20;
	HAL_CAN_ConfigFilter(hcan, &Filter);

}

void CAN_Config(CAN_HandleTypeDef* hcan)
{
	CRCInit();
	CAN_ConfigFilter(hcan);
	HAL_CAN_Start(hcan);
}

void CAN_Receive(CAN_HandleTypeDef* hcan)
{
	HAL_StatusTypeDef Status;
	CAN_RxHeaderTypeDef Can_Header = {0};
	uint32_t FreeLevel;
	FreeLevel = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_FILTER_FIFO0);
	uint8_t RcvBuffer[8] = {0};
	while(FreeLevel > 0)
	{
		Status = HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &Can_Header, RcvBuffer);
		if(Status != HAL_OK) return;

//		MyPrintf("Verify Rcv ID = %.8lx Len = %ld \r\n", Can_Header.StdId, Can_Header.DLC);
		for(uint8_t i = 0; i< Can_Header.DLC ;++i)
		{
			MyPrintf("%d ", RcvBuffer[i]);
		}
//		MyPrintf("\r\n");
		if(Can_Header.StdId == CANRcvID)
		{
			uint8_t check = CalcCRC(RcvBuffer, 7);
			if(check == RcvBuffer[7])
			{
				MyPrintf("CRC Valid\r\n");
			}else
			{
				MyPrintf("Unvalid CRC\r\n");
			}
		}
		FreeLevel = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_FILTER_FIFO0);
	}
}

void CAN_Transmit(CAN_HandleTypeDef* hcan)
{
	CAN_TxHeaderTypeDef Can_Header = {0};
	HAL_StatusTypeDef Status;
	uint32_t Mailbox = 0;
	uint32_t FreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	if(FreeLevel == 0) return;
	uint8_t Data[8]= {10,MessageCNT,0,0,0,0,MessageCNT,0};
	MessageCNT++;

	Can_Header.DLC = 8;
	Can_Header.StdId = CANTransmitID;

	Status = HAL_CAN_AddTxMessage(hcan, &Can_Header, Data, &Mailbox);

	if(Status == HAL_OK)
	{
//		printf("Message in mailbox %ld \r\n", Mailbox);
	}
	else
	{
		printf("Transmit Error Status %d Code = 0x%.8lx \r\n", Status, hcan->ErrorCode);
	}
}


