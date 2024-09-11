/*
 * uwb_mac.h
 *
 *  Created on: Sep 5, 2024
 *      Author: 24848
 */

#ifndef DEVICES_INC_UWB_MAC_H_
#define DEVICES_INC_UWB_MAC_H_

#include "uwb.h"
#include "uwb_ranging.h"
#include "uwb_msg.h"


#define UWB_REPLY_INTERVAL  3

/**
 * k = ((macro-1)/3)*12 +(macro-1)%3+1;
   printf("宏时隙：%d, 微时隙：%d, %d, %d, %d\n", macro, k, k+3, k+6, k+9);
 *
 */
#define GET_MICRO_SLOT1(macro)  ((((macro)-1)/UWB_REPLY_INTERVAL)*(UWB_REPLY_INTERVAL*4) +((macro)-1)%UWB_REPLY_INTERVAL+1)
#define GET_MICRO_SLOT2(macro)  ((GET_MICRO_SLOT1(macro))+UWB_REPLY_INTERVAL)
#define GET_MICRO_SLOT3(macro)  ((GET_MICRO_SLOT1(macro))+UWB_REPLY_INTERVAL*2)
#define GET_MICRO_SLOT4(macro)  ((GET_MICRO_SLOT1(macro))+UWB_REPLY_INTERVAL*3)

#define  PDoA_RX_Buffer  UWB_Msg_Header_t

typedef struct{
	uint16_t anchor_id;
	uint16_t anchor_pan_id; //感觉pan_id也不是说必需的
	uint16_t signal;
	uint8_t  macro;
	uint8_t  is_valid;
	uint16_t micro1;
	uint16_t micro2;
	uint16_t micro3;
	uint16_t micro4;
	uint8_t  interval;
	uint8_t  to_interval;   //-1  every superframe  锚节点遍历该值确定是否测距标签节点
}Slot_Alloc_t;

typedef struct{
	Slot_Alloc_t slot_alloc;
	Slot_Alloc_t* pnext;
}slot_alloc_node_t;  //20B?

//是用预处理还是写两套？
//那么锚节点就需要很多个这个东西了
typedef struct{

	uint16_t ranging_id;   //当前正在测距节点
	//tag
	uint64 poll_tx_ts;
	uint64 resp_rx_ts;
	uint64 final_tx_ts;
	//anchor
	uint64 poll_rx_ts;
	uint64 resp_tx_ts;
	uint64 final_rx_ts;

	int64 tof_dtu;
	int64 R1, R2, D1, D2;
	/**
	 * @TODO:锚节点增加角度相关记录？
	 */

}UWB_RangingValue_t;


typedef struct{


	uint16_t pan_id;
	uint16_t id;
	//自己的interval
	//对其一下或许能够
	UWB_Role_t role;
	uint8_t interval;
	UWBDef* device;
	//sequence  only 1 octet
	uint8_t sequence;

	slot_alloc_node_t*  pslot_alloc_table;

//	UWB_Msg_Header_t header;

	PDoA_RX_Buffer pdoa_buffer;

	uint8_t rxBuffer[128];       //本来其实125就够了   发送的是否需要，或者一个数据包和一个节点挂钩，算了先考虑标签

	UWB_RangingValue_t ranging_values;  //对于tag节点来说只需一个即可
	/**
	 * @TODO 考虑锚节点则还需要多个记录时间戳的
	 */
	//发送数据包函数
	/**
	 * @TODO UWB发送函数再考虑一下
	 */
	//解析数据包函数
	uint8_t (*uwb_parse_msg)(UWBPortTypeDef *pports);   //哟啊不要写入是哪一个那个的呢？i 呀
	int32_t (*uwb_phy_init)(uint16_t ID, UWB_Role_t role);

}UWB_Node_t;

/*
基准时间 100us			autoReload = 100-1;  ---> 中断
时钟精度 1us             preScale ---->  1us
考虑一个实现：
2ms 时隙
500ms / 2ms = 250
250 -1 = 249 空出一个 ? 缓冲也行
249 - 50*4 = 49
49/2 = 24
最多可以容纳 这么多节点    249 / 4 = 62
 */
//24 * 4ms = 96ms


/**
 * 根据role初始化节点结构体的参数
 */
uint8_t initNode(UWB_Role_t role);


#endif /* DEVICES_INC_UWB_MAC_H_ */
