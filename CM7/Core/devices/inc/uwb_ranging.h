/*
 * uwb_ranging.h
 *
 *  Created on: Sep 6, 2024
 *      Author: 24848
 */

#ifndef DEVICES_INC_UWB_RANGING_H_
#define DEVICES_INC_UWB_RANGING_H_

typedef enum{
	no_delay = 0,
	need_delay = 1,
}If_Delay_t;

typedef enum{
	no_expect = 0,
	expect = 1,
}If_Expected_t;

typedef enum{
	anchor =1,
	tag =2,
}UWB_Role_t;

void uwbRangingPoll(void);
void uwbRangingResp(uint16_t src, uint16_t dest);
void uwbRangingFinal(uint16_t src, uint16_t dest);
void uwbRangingAck(uint16_t src, uint16_t dest, uint8_t *p_data);




#endif /* DEVICES_INC_UWB_RANGING_H_ */
