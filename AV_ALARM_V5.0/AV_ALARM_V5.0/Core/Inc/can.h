/*
 * can.h
 *
 *  Created on: Jan 25, 2023
 *      Author: sivac
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

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
uint8_t g_Fault_u8;
uint8_t g_i_u8=0;
uint8_t g_five_sec_u8=0;
uint8_t g_comm_fail_u8=0;
uint16_t g_current1_u16;
uint16_t g_current2_u16;
uint32_t g_battery_current_u32;
uint8_t g_over_current_u8;
uint32_t g_four_min_u32;
uint8_t g_sleep_u8;

#endif /* INC_CAN_H_ */
