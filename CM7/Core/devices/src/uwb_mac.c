/*
 * uwb_mac.c
 *
 *  Created on: Sep 5, 2024
 *      Author: 24848
 */


#include "uwb_mac.h"

#define PAN_ID	0
#define ID		0x1


//静态函数需要声明
static uint8_t uwb_parse_ranging(void);
static uint8_t uwb_parse_pdoa(void);
static uint8_t uwb_parse(UWBPortTypeDef *pports);

static void uwbSendPoll(uint16_t src, uint16_t dest);
static void uwbSendResp(uint16_t src, uint16_t dest);
static void uwbSendFinal(uint16_t src, uint16_t dest);
static void uwbSendAck(uint16_t src, uint16_t dest, uint8_t *p_data);


UWB_Node_t uwb_node = {

		.id = ID,
		.pan_id = PAN_ID,
		.interval = 1,
		.pslot_alloc_table = NULL,
		.ranging_values={0,},
		.uwb_parse_msg = uwb_parse,
		.uwb_phy_init = uwbInit,

};



uint8_t initNode(UWB_Role_t role){
	uwb_node.role = role;
	/**
	 * @TODO 可能要根据role的不同初始化node节点 ？
	 */
	switch(uwb_node.role){
		case anchor:

			break;
		case tag:

			break;
	}
	uwb_node.uwb_phy_init(uwb_node.id, uwb_node.role);
	return 0;
}

static uint8_t uwb_parse(UWBPortTypeDef *pports){
	uint8_t ret = 1;
	switch(pports->port_type){
		case Ds_Twr_Port:
			ret = uwb_parse_ranging();
			break;
		case PDoA_Port:
			ret = uwb_parse_pdoa();
			break;
		default:
			break;
	}
	return ret;
}

static uint8_t uwb_parse_ranging(void){

	return 0;
}
static uint8_t uwb_parse_pdoa(void){

	return 0;
}

//内部调用UWB_Send
static void uwbSendPoll(uint16_t src, uint16_t dest , uint16_t dest_pan_id){

}
static void uwbSendResp(uint16_t src, uint16_t dest, uint16_t dest_pan_id){

}
static void uwbSendFinal(uint16_t src, uint16_t dest, uint16_t dest_pan_id){

}
static void uwbSendAck(uint16_t src, uint16_t dest, uint8_t *p_data, uint16_t dest_pan_id){

}



