#include "uwb.h"
#include "corecomm.h"
#include "main.h"
# include "agent.h"
#include "spi.h"
#include <math.h>

#include "kalman_filter.h"

#include "uwb_mac.h"


//#define ACC_MEM_TEST 1
/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
	DWT_BR_6M8,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Use-d in RX only. */
};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(UWBPortTypeDef *pports);
static uint64 get_rx_timestamp_u64(UWBPortTypeDef *pports);
static float uwb_get_fp_angle(uint16_t, UWBPortTypeDef *);


UWBDef UWB={
		.err_time = 0
};

extern UWB_Node_t uwb_node;

/**
 * @TODO  当前锚节点？ 标签节点？
 */
//UWB add
extern AgentTypeDef agent;
extern AgentTypeDef leader_agent;
extern AgentTypeDef *p_current_agent;


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi6;


extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim6;


/**
 * @TODO done 物理层的初始化
 * SPI， 帧过滤相关等
 */
int32_t uwbInit(uint16_t ID, UWB_Role_t role)
{

	dwt_txconfig_t config_t = {
			.PGdly = 0xC0,
			.power = 0x851F1F85
	};

	//标签节点只使用一个
    UWB.ports[0].hspi = &hspi6;
    UWB.ports[0].irq_pin = UWB_IRQn_Pin;
    UWB.ports[0].irq_port = UWB_IRQn_GPIO_Port;
    UWB.ports[0].rstn_pin = UWB_RSTn_Pin;
    UWB.ports[0].rstn_port = UWB_RSTn_GPIO_Port;
    UWB.ports[0].spi_csn_pin = UWB_SPICSn_Pin;
    UWB.ports[0].spi_csn_port = UWB_SPICSn_GPIO_Port;
    UWB.ports[0].wakeup_pin = 0;
    UWB.ports[0].wakeup_port = NULL;
    UWB.ports[0].exti_line = EXTI15_10_IRQn;
    UWB.ports[0].port_type = Ds_Twr_Port;

	if (role == anchor) {
		UWB.ports[1].hspi = &hspi1;
		UWB.ports[1].irq_pin = UWB1A_IRQ_Pin;
		UWB.ports[1].irq_port = UWB1A_IRQ_GPIO_Port;
		UWB.ports[1].rstn_pin = UWB1A_RSTn_Pin;
		UWB.ports[1].rstn_port = UWB1A_RSTn_GPIO_Port;
		UWB.ports[1].spi_csn_pin = UWB1A_SPICSn_Pin;
		UWB.ports[1].spi_csn_port = UWB1A_SPICSn_GPIO_Port;
		UWB.ports[1].wakeup_pin = UWB1_WAKEUP_Pin;
		UWB.ports[1].wakeup_port = UWB1_WAKEUP_GPIO_Port;
		UWB.ports[1].exti_line = EXTI0_IRQn;

		UWB.ports[1].port_type = PDoA_Port;

		UWB.ports[2].hspi = &hspi1;
		UWB.ports[2].irq_pin = UWB1B_IRQ_Pin;
		UWB.ports[2].irq_port = UWB1B_IRQ_GPIO_Port;
		UWB.ports[2].rstn_pin = UWB1B_RSTn_Pin;
		UWB.ports[2].rstn_port = UWB1B_RSTn_GPIO_Port;
		UWB.ports[2].spi_csn_pin = UWB1B_SPICSn_Pin;
		UWB.ports[2].spi_csn_port = UWB1B_SPICSn_GPIO_Port;
		UWB.ports[2].wakeup_pin = UWB1_WAKEUP_Pin;
		UWB.ports[2].wakeup_port = UWB1_WAKEUP_GPIO_Port;
		UWB.ports[2].exti_line = EXTI1_IRQn;

		UWB.ports[2].port_type = PDoA_Port;
	}

    for(int i=0;i<DWT_NUM_DW_DEV;i++)
    {
    	reset_DW1000(&UWB.ports[i]);
    	UWB.ports[i].avalible = 0;

    }
    for(int i=0;i<DWT_NUM_DW_DEV;i++)
    {
    	if(role == tag && i > 0){
    		//只要初始化第一个Port
    		break;
    	}
    	UWBPortTypeDef *pports = &UWB.ports[i];
    	int trys = 10;
    	port_set_dw1000_slowrate(pports);
    	dwt_setlocaldataptr(i);
    	do{
			if (dwt_initialise(DWT_LOADUCODE, pports) == DWT_SUCCESS)
			{
				UWB.ports[i].avalible = 1;
			}
			else
			{
				trys--;
			}
    	} while((trys > 0) && UWB.ports[i].avalible != 1);
    	if(trys <= 0)
    	{
    		continue;
    	}

//    	dwt_setpanid(MAC_PANID, pports);
    	dwt_setaddress16(ID, pports);
		port_set_dw1000_fastrate(pports);
		dwt_enableframefilter(DWT_FF_DATA_EN, pports);

		/*Setup Interrupt*/
		dwt_setinterrupt((SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO), 2, pports);
//		dwt_setcallbacks(NULL, rxOkCallback, rxToCallback, rxErrCallback);  //不同的使用不同的这个callback呢？
		if(i == 0){
			dwt_setcallbacks(NULL, rxOkCallback_Ranging, rxToCallback, rxErrCallback);  //不同的使用不同的这个callback呢？
		}else{
			dwt_setcallbacks(NULL, rxOkCallback_Ranging, rxToCallback, rxErrCallback);  //不同的使用不同的这个callback呢？
		}

		port_set_deca_isr(dwt_isr);
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR, pports);
		/* Configure DW1000. See NOTE 7 below. */
		dwt_configure(&config, pports);

		//2024.06.03  发射功测试
		dwt_setsmarttxpower(0, pports);
		//设置发射功率
		dwt_configuretxrf(&config_t, pports);
		//end
		/* Apply default antenna delay value. See NOTE 1 below. */
		dwt_setrxantennadelay(RX_ANT_DLY, pports);
		dwt_settxantennadelay(TX_ANT_DLY, pports);

		/* Set preamble timeout for expected frames. See NOTE 6 below. */
		dwt_setpreambledetecttimeout(PRE_TIMEOUT, pports);
		/* Configure LED */
		dwt_setleds(1, pports);
		/* Set Sync to OSTR*/
		dwt_setecctrl(EC_CTRL_OSTRM, pports);
	}
    uwb_node.device = &UWB;

    return 0;
}


static uint64 get_tx_timestamp_u64(UWBPortTypeDef *pports)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab, pports);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(UWBPortTypeDef *pports)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab, pports);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

void rxOkCallback_Ranging(const dwt_cb_data_t *cbData, UWBPortTypeDef *pports){

	/**
	 * @TODO pdata to store the package data
	 * pdata   allocate or  static buffer array
	 */
	uint8_t * pdata = uwb_node.rxBuffer;
	//这个数据放进哪里去?直接放到Node的缓冲区
	dwt_readrxdata(pdata, cbData->datalength, 0,pports);
	uwb_node.uwb_parse_msg(pports);
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(0 , pports);
	/* Activate reception immediately. */
	/**
	 * @TODO if to enable rx soon or later
	 */
	dwt_rxenable(DWT_START_RX_IMMEDIATE , pports);  //原来是处理完这里之后就又重新使能了接收，所以必然地，每一次的txPoll都要forcertxoff进入idle模式

}

void rxOkCallback_PDoA(const dwt_cb_data_t *, UWBPortTypeDef *pports){
	//这个其实只需要什么？头部就够了
	/**
		 * @TODO pdata to store the package data
		 * pdata   allocate or  static buffer array
		 */
		uint8_t * pdata = (uint8_t*)&uwb_node.pdoa_buffer;  //其实只是一个头部的部分
		//这个数据放进哪里去?直接放到Node的缓冲区
		dwt_readrxdata(pdata, sizeof(PDoA_RX_Buffer), 0,pports);
		/**
		 * @TODO
		 * 和上层的接口
		 */
		uwb_node.uwb_parse_msg(pports);
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(0 , pports);
		/* Activate reception immediately. */
		//undoubtedly turn on this immediately
		dwt_rxenable(DWT_START_RX_IMMEDIATE , pports);
}

void rxToCallback(const dwt_cb_data_t *cbData, UWBPortTypeDef *pports)
{
	/* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0, pports);
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE, pports);
}

void rxErrCallback(const dwt_cb_data_t *cbData, UWBPortTypeDef *pports)
{
	/* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0, pports);
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE, pports);
}

//(1) if have to set the delay
//final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
void UWB_Set_DelayTX(uint32_t tx_time , UWBPortTypeDef *pports){
	dwt_setdelayedtrxtime(tx_time,pports);
}

//发送消息 (2)
void UWB_Send(uint8_t * pdata, uint8_t len, If_Delay_t is_delayed, uint32_t tx_time, If_Expected_t is_expect, UWBPortTypeDef *pports)

{
    /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS,pports);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS, pports);
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS, pports);

	if(is_delayed){  //延迟发送
		dwt_setdelayedtrxtime(tx_time, pports);
	}

    dwt_forcetrxoff(pports); // Turn the RX off
    dwt_rxreset(pports);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS, pports);

    /**
     * @TODO 向UWB芯片写入数据
     */
	//Tanya_change
	dwt_writetxdata(len, pdata, 0, pports); /* Zero offset in TX buffer. */
	/**
	 * @TODO 向UWB写入发送数据
	 * 或可参照LwIP的方式分段
	 */
	dwt_writetxfctrl(len, 0, 1, pports); /* Zero offset in TX buffer, ranging. */

//	memset(UWB.txBuffer , 0, sizeof(UWB.txBuffer));
	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	if((!is_delayed)&&is_expect){
		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED, pports);
	}else if(is_delayed && is_expect){
		dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED , pports);
	}else if(is_delayed &&(!is_expect)){
		dwt_starttx(DWT_START_TX_DELAYED , pports);
	}else{
		dwt_starttx(DWT_START_TX_IMMEDIATE , pports);
	}
}

uint64_t getDeltaT(uint64_t ts1, uint64_t ts2)
{
	if(ts1 > ts2)
	{
		return ts1-ts2;
	}
	else
	{
		return ts1+(0xFFFFFFFFFF-ts2);
	}
}


float uwb_calculate_rx_power(uint16_t cir_pwr, uint16_t rxpacc, uint16_t rxpacc_nosat){
	float adjusted_rxpacc;
	float temp1, temp2;
	const float two_17 = (float)(1<<17);
	if(rxpacc == rxpacc_nosat){
		adjusted_rxpacc = rxpacc + RXPACC_ADJUSTMENT;
	}
	else{
		adjusted_rxpacc = rxpacc;
	}
	temp1 = (float)cir_pwr * two_17;
	temp2 = powf(adjusted_rxpacc, 2);
	temp1 = temp1 / temp2;
	temp1 = log10f(temp1);
	temp1 = 10 * temp1 - A_PRF64M;
	return temp1;
//	return 10*log10f((float)cir_pwr * two_17 / powf(adjusted_rxpacc,2)) - A_PRF64M;
}

float uwb_calculate_fp_power(uint16_t fp_amp1, uint16_t fp_amp2, uint16_t fp_amp3, uint16_t rxpacc, uint16_t rxpacc_nosat){
	float adjusted_rxpacc;
	if(rxpacc == rxpacc_nosat){
		adjusted_rxpacc = rxpacc + RXPACC_ADJUSTMENT;
	}
	else{
		adjusted_rxpacc = rxpacc;
	}
	return 10*log10f((powf((float)fp_amp1,2) + powf((float)fp_amp2,2) + powf((float)fp_amp3,2)) / powf(adjusted_rxpacc,2)) - A_PRF64M;
}


float array_average(float* array, uint8_t num)
{
	float *p = array;
	float sum = 0;
	for(uint8_t i = 0; i < num;i++ )
	{
		sum += *p;
		p++;
	}
	return sum/(float)num;
}

//补偿什么？
float distance_compensate(float distance)
{
	if(distance > 1)
		return distance;
	else
//		return (-0.1594*distance+0.1994);
		return (-0.1966*distance*distance+1.4134*distance-0.2207);
}


void sliding_filter(float *distance)
{
//	return;
	*distance = (0.7*(*distance)+0.2*(*(distance+1)) + 0.1*(*(distance+2)));

	//	*distance = (0.6*(*distance)+0.25*(*(distance+1)) + 0.15*(*(distance+2)));
//	*distance = (0.5*(*distance)+0.25*(*(distance+1)) + 0.15*(*(distance+2)) + 0.1*(*(distance+3)));
//	*distance = (0.4*(*distance)+0.25*(*(distance+1)) + 0.25*(*(distance+2)) + 0.1*(*(distance+3)));

}

void initOtherPorts(void){

	if(DWT_NUM_DW_DEV > 3){

		UWB.ports[3].hspi = &hspi2;
		UWB.ports[3].irq_pin = UWB2A_IRQ_Pin;
		UWB.ports[3].irq_port = UWB2A_IRQ_GPIO_Port;
		UWB.ports[3].rstn_pin = UWB2A_RSTn_Pin;
		UWB.ports[3].rstn_port = UWB2A_RSTn_GPIO_Port;
		UWB.ports[3].spi_csn_pin = UWB2A_SPICSn_Pin;
		UWB.ports[3].spi_csn_port = UWB2A_SPICSn_GPIO_Port;
		UWB.ports[3].wakeup_pin = UWB2_WAKEUP_Pin;
		UWB.ports[3].wakeup_port = UWB2_WAKEUP_GPIO_Port;
		UWB.ports[3].exti_line = EXTI9_5_IRQn;
		UWB.ports[3].port_type = PDoA_Port;

		UWB.ports[4].hspi = &hspi2;
		UWB.ports[4].irq_pin = UWB2B_IRQ_Pin;
		UWB.ports[4].irq_port = UWB2B_IRQ_GPIO_Port;
		UWB.ports[4].rstn_pin = UWB2B_RSTn_Pin;
		UWB.ports[4].rstn_port = UWB2B_RSTn_GPIO_Port;
		UWB.ports[4].spi_csn_pin = UWB2B_SPICSn_Pin;
		UWB.ports[4].spi_csn_port = UWB2B_SPICSn_GPIO_Port;
		UWB.ports[4].wakeup_pin = UWB2_WAKEUP_Pin;
		UWB.ports[4].wakeup_port = UWB2_WAKEUP_GPIO_Port;
		UWB.ports[4].exti_line = EXTI9_5_IRQn;
		UWB.ports[4].port_type = PDoA_Port;

		UWB.ports[5].hspi = &hspi3;
		UWB.ports[5].irq_pin = UWB3A_IRQ_Pin;
		UWB.ports[5].irq_port = UWB3A_IRQ_GPIO_Port;
		UWB.ports[5].rstn_pin = UWB3A_RSTn_Pin;
		UWB.ports[5].rstn_port = UWB3A_RSTn_GPIO_Port;
		UWB.ports[5].spi_csn_pin = UWB3A_SPICSn_Pin;
		UWB.ports[5].spi_csn_port = UWB3A_SPICSn_GPIO_Port;
		UWB.ports[5].wakeup_pin = UWB3_WAKEUP_Pin;
		UWB.ports[5].wakeup_port = UWB3_WAKEUP_GPIO_Port;
		UWB.ports[5].exti_line = EXTI15_10_IRQn;
		UWB.ports[5].port_type = PDoA_Port;

		UWB.ports[6].hspi = &hspi3;
		UWB.ports[6].irq_pin = UWB3B_IRQ_Pin;
		UWB.ports[6].irq_port = UWB3B_IRQ_GPIO_Port;
		UWB.ports[6].rstn_pin = UWB3B_RSTn_Pin;
		UWB.ports[6].rstn_port = UWB3B_RSTn_GPIO_Port;
		UWB.ports[6].spi_csn_pin = UWB3B_SPICSn_Pin;
		UWB.ports[6].spi_csn_port = UWB3B_SPICSn_GPIO_Port;
		UWB.ports[6].wakeup_pin = UWB3_WAKEUP_Pin;
		UWB.ports[6].wakeup_port = UWB3_WAKEUP_GPIO_Port;
		UWB.ports[6].exti_line = EXTI15_10_IRQn;
		UWB.ports[6].port_type = PDoA_Port;

		UWB.ports[7].hspi = &hspi4;
		UWB.ports[7].irq_pin = UWB4A_IRQ_Pin;
		UWB.ports[7].irq_port = UWB4A_IRQ_GPIO_Port;
		UWB.ports[7].rstn_pin = UWB4A_RSTn_Pin;
		UWB.ports[7].rstn_port = UWB4A_RSTn_GPIO_Port;
		UWB.ports[7].spi_csn_pin = UWB4A_SPICSn_Pin;
		UWB.ports[7].spi_csn_port = UWB4A_SPICSn_GPIO_Port;
		UWB.ports[7].wakeup_pin = UWB4_WAKEUP_Pin;
		UWB.ports[7].wakeup_port = UWB4_WAKEUP_GPIO_Port;
		UWB.ports[7].exti_line = EXTI3_IRQn;
		UWB.ports[7].port_type = PDoA_Port;

		UWB.ports[8].hspi = &hspi4;
		UWB.ports[8].irq_pin = UWB4B_IRQ_Pin;
		UWB.ports[8].irq_port = UWB4B_IRQ_GPIO_Port;
		UWB.ports[8].rstn_pin = UWB4B_RSTn_Pin;
		UWB.ports[8].rstn_port = UWB4B_RSTn_GPIO_Port;
		UWB.ports[8].spi_csn_pin = UWB4B_SPICSn_Pin;
		UWB.ports[8].spi_csn_port = UWB4B_SPICSn_GPIO_Port;
		UWB.ports[8].wakeup_pin = UWB4_WAKEUP_Pin;
		UWB.ports[8].wakeup_port = UWB4_WAKEUP_GPIO_Port;
		UWB.ports[8].exti_line = EXTI4_IRQn;
		UWB.ports[8].port_type = PDoA_Port;
	}

}



