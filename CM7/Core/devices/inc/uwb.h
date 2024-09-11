#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "deca_port.h"
#include "dwio.h"
#include "main.h"

#include "uwb_consts.h"
#include "uwb_ranging.h"

//Tanya_add
//This is a file of UWB PHY layer

#ifndef __UWB_H_
#define __UWB_H_


typedef struct
{
	UWBPortTypeDef ports[DWT_NUM_DW_DEV];
	//UWB add
	uint16_t antDelay;
//	uint8_t txBuffer[128];
//	uint8_t rxBuffer[128];
	uint16_t err_time;

} UWBDef;



int32_t uwbInit(uint16_t ID, UWB_Role_t role);

//Tanya_add
void rxOkCallback_Ranging(const dwt_cb_data_t *cbData, UWBPortTypeDef *pports);
//Tanya_add end
void rxOkCallback_PDoA(const dwt_cb_data_t *, UWBPortTypeDef *);
void rxToCallback(const dwt_cb_data_t *, UWBPortTypeDef *);
void rxErrCallback(const dwt_cb_data_t *, UWBPortTypeDef *);

uint64_t getDeltaT(uint64_t,uint64_t);

void UWB_Send(uint8_t * pdata, uint8_t len, If_Delay_t is_delayed, uint32_t tx_time, If_Expected_t is_expect, UWBPortTypeDef *pports);

void uwbTxPollMsg(uint16_t,UWBPortTypeDef *);
void uwbTxRespMsg(uint16_t,UWBPortTypeDef *);
void uwbTxFinalMsg(uint16_t,UWBPortTypeDef *);
void uwbTxAckMsg(uint16_t,UWBPortTypeDef *);

float uwb_calculate_rx_power(uint16_t cir_pwr, uint16_t rxpacc, uint16_t rxpacc_nosat);
float uwb_calculate_fp_power(uint16_t fp_amp1, uint16_t fp_amp2, uint16_t fp_amp3, uint16_t rxpacc, uint16_t rxpacc_nosat);

void sliding_filter(float *distance);
float distance_compensate(float distance);
float array_average(float* array, uint8_t num);


#endif