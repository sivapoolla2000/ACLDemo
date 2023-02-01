/*
 * CAN.c
 *
 *  Created on: Jan 2, 2023
 *      Author: sivac
 */
/*
 * CAN.c
 *
 *  Created on: 31-Dec-2022
 *      Author: sivac
 */


/*
 * CAN.c
 *
 *  Created on: Dec 30, 2022
 *      Author: sivac
 */
#include "main.h"



FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;

uint8_t RxData[8];
uint8_t TxData[8]={0xAA};
uint8_t g_Msg_1aa_RxData_u8[8];
uint8_t g_Msg_2aa_RxData_u8[8];
uint8_t g_Msg_3aa_RxData_u8[8];
uint8_t g_under_voltage_u8;
uint8_t g_over_voltage_u8;
uint8_t g_over_temperature_u8;
uint8_t g_under_temperature_u8;
uint8_t g_over_load_u8 ;
uint8_t g_short_circuit_u8;
uint8_t g_parameter_timeout_u8;
uint8_t g_precharge_fail_u8;
uint8_t g_SOC_u8;
uint32_t g_Fault_u8;
uint8_t g_i_u8=0;
uint8_t g_five_sec_u8=0;
uint8_t g_comm_fail_u8=0;
uint16_t g_current1_u16;
uint16_t g_current2_u16;
uint32_t g_battery_current_u32;
uint8_t g_over_current_u8;
uint8_t g_sleep_u8=0;
uint8_t g_debounce_u8=0;
uint8_t g_timerstart_u8=0;
uint32_t g_four_min_u32;
uint32_t g_counter_u8=0;
uint32_t g_one_sec_u8=0;


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {

    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }


    if(RxHeader.Identifier == 0x1F11FA01)
    {
    	g_Fault_u8 = (uint32_t)RxData[0] | (uint32_t)RxData[1]<<8 | (uint32_t)RxData[2]<<16 | (uint32_t)RxData[3]<<24;
    }
    else if(RxHeader.Identifier == 0x1F10FA01)
    {
    	g_SOC_u8 = RxData[0];
    	g_comm_fail_u8=0;
    }

    else if (RxHeader.Identifier == 0X1AA)
    {
    	for (g_i_u8 = 0; g_i_u8 < 8; g_i_u8++)
    	{
    		g_Msg_1aa_RxData_u8[g_i_u8]=RxData[g_i_u8];

    	}
    	CaptureData_Fault();
    	g_five_sec_u8=0;
    	g_comm_fail_u8=0;
     }
    else if (RxHeader.Identifier == 0X2AA)
    {  	for (g_i_u8 = 0; g_i_u8 < 8; g_i_u8++)			//  	g_Msg_2aa_RxData_u8[8]=RxData[8];
    	{
    		g_Msg_2aa_RxData_u8[g_i_u8]=RxData[g_i_u8];
    	}
    	CaptureData_Battery();

    g_five_sec_u8=0;
    g_comm_fail_u8=0;
    }
    else if (RxHeader.Identifier == 0X3AA)
        {
        	for (g_i_u8 = 0; g_i_u8 < 8; g_i_u8++)
        	{
        		g_Msg_3aa_RxData_u8[g_i_u8]=RxData[g_i_u8];
        	}
        	Capture_Current();
        	g_five_sec_u8=0;
        	g_comm_fail_u8=0;
        }
  }
}

void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  FDCAN_HandleTypeDef hfdcan1;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x1F11FA01;
  sFilterConfig.FilterID2 = 0x1F10FA01;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x000001FA;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;//FDCAN2;
  TxHeader.ErrorStateIndicator =FDCAN_ESI_ACTIVE;//FDCAN_EAN_DLC_BYTESSI_ACTIVE; //
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;// FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{	FDCAN_HandleTypeDef hfdcan1;
		unsigned long txFifoRequest=0;
		g_counter_u8++;
		if (g_timerstart_u8==1)
		{
			g_debounce_u8++;
		}
		if (g_counter_u8>500)
		{	g_counter_u8=0;
			g_counter_u8++;
			g_one_sec_u8++;
			g_four_min_u32++;
			g_five_sec_u8++;


			if (g_four_min_u32>250)//if (g_four_min_u32>490)
			{
				g_four_min_u32=0;
			}
			if (g_one_sec_u8>2)
			{
				g_one_sec_u8=0;
			}
			if (g_five_sec_u8>5)
			{
				g_comm_fail_u8=1;
			}
			if (g_four_min_u32>=0 && g_four_min_u32<120)//if (g_four_min_u32>=0 && g_four_min_u32<240)
			{
				g_sleep_u8=0;
			}
			if (g_four_min_u32>=120 && g_four_min_u32<240)//if (g_four_min_u32>=240 && g_four_min_u32<480)
			{
				g_sleep_u8=1;
			}
			if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>0)
			{
				TxHeader.Identifier = 0x000001FA;
				TxHeader.IdType = FDCAN_EXTENDED_ID;
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
				{
					Error_Handler();
				}
			}

			else
			{
		  	 txFifoRequest = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
		  	 if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,txFifoRequest))
		  	 {
		  		HAL_FDCAN_AbortTxRequest(&hfdcan1,txFifoRequest);
		  	 }
			}
		  }



	}

void MX_FDCAN1_Init(void)
{FDCAN_HandleTypeDef hfdcan1;

 /* USER CODE BEGIN FDCAN1_Init 0 */

 /* USER CODE END FDCAN1_Init 0 */

 /* USER CODE BEGIN FDCAN1_Init 1 */

 /* USER CODE END FDCAN1_Init 1 */
 hfdcan1.Instance = FDCAN1;
 hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
 hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
 hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
 hfdcan1.Init.AutoRetransmission = DISABLE;
 hfdcan1.Init.TransmitPause = DISABLE;
 hfdcan1.Init.ProtocolException = DISABLE;
 hfdcan1.Init.NominalPrescaler = 8;
 hfdcan1.Init.NominalSyncJumpWidth = 1;
 hfdcan1.Init.NominalTimeSeg1 = 9;
 hfdcan1.Init.NominalTimeSeg2 = 6;
 hfdcan1.Init.DataPrescaler = 9;
 hfdcan1.Init.DataSyncJumpWidth = 1;
 hfdcan1.Init.DataTimeSeg1 = 9;
 hfdcan1.Init.DataTimeSeg2 = 6;
 hfdcan1.Init.StdFiltersNbr = 0;
 hfdcan1.Init.ExtFiltersNbr = 0;
 hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
 if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN FDCAN1_Init 2 */

 /* USER CODE END FDCAN1_Init 2 */

}

int CaptureData_Fault(void)
{
	 g_Fault_u8=g_Msg_1aa_RxData_u8[5];
	 if( g_Fault_u8& (1<<0))
	 {
		 g_under_voltage_u8=1;
	 }
	 else if( g_Fault_u8& (1<<1))
	 {
		 g_over_voltage_u8=1;
	 }
	 else if( g_Fault_u8& (1<<2))
	 {
		 g_over_temperature_u8=1;
	 }
	 else if( g_Fault_u8& (1<<3))
	 {
		 g_under_temperature_u8=1;
	 }
	 else if( g_Fault_u8& (1<<4))
	 {
		 g_over_load_u8 =1;
	 }
	 else if( g_Fault_u8& (1<<5))
	 {
		 g_short_circuit_u8=1;
	 }
	 else if( g_Fault_u8& (1<<6))
	 {
		 g_parameter_timeout_u8=1;
	 }
	 else if( g_Fault_u8& (1<<7))
	 {
		 g_precharge_fail_u8=1;
	 }
}

void battery_indicator(void)
{
	if (g_sleep_u8==1)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
	}


	if (g_Fault_u8==0 && g_SOC_u8>=0 && g_SOC_u8<=25 && g_over_current_u8==0)
	{
			if(g_one_sec_u8>1 && g_sleep_u8==0)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
			}

			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	}
	else if (g_Fault_u8==0 && g_SOC_u8>25 && g_SOC_u8<=50 && g_over_current_u8==0)
		{
			if (g_sleep_u8==0)
			{
				if(g_one_sec_u8>1)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
				}
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			}
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		}
	else if (g_Fault_u8==0 && g_SOC_u8>50 && g_SOC_u8<=75 && g_over_current_u8==0)
		{
			if (g_sleep_u8==0)
			{
				if(g_one_sec_u8>1 && g_sleep_u8==0)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
				}

				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
			}
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		}
	else if (g_Fault_u8==0 && g_SOC_u8>75 && g_SOC_u8<=100 && g_over_current_u8==0)
		{	if (g_sleep_u8==0)
			{
				if(g_one_sec_u8>1)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
					}
				else
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
					}
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
			}
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		}
	else if (g_Fault_u8!=0 || g_over_current_u8!=0)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
			if(g_one_sec_u8==1)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
			}

		}
}

void CaptureData_Battery(void)
{
	g_SOC_u8=g_Msg_2aa_RxData_u8[0];
}

void Capture_Current(void)
{
		g_current1_u16=g_Msg_3aa_RxData_u8[0];
		g_current1_u16=g_current1_u16<<8;
		g_current2_u16= g_Msg_3aa_RxData_u8[1];
		g_battery_current_u32=g_current1_u16+g_current2_u16;
		if(g_battery_current_u32>=200)
		{
			g_over_current_u8=1;
		}
		else
		{
			g_over_current_u8=0;
		}
}
void ButtonFunc(void)
{
	if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_RESET)
	{
		g_timerstart_u8=1;
		if (g_debounce_u8>51)
		{
			if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_RESET)
			{
				g_sleep_u8=0;
			}
			else if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_SET)
			{
				g_sleep_u8=1;
			}
			g_timerstart_u8=0;
			g_debounce_u8=0;
		}
	}
}

