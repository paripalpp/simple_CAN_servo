/*
 * CAN103xb.h
 *
 *  Created on: Mar 25, 2022
 *      Author: paripal
 */

#ifndef SRC_CAN103XB_H_
#define SRC_CAN103XB_H_

#include "stm32f1xx_hal.h"

namespace stm_CAN {

typedef enum {
	mask = CAN_FILTERMODE_IDMASK,
	list = CAN_FILTERMODE_IDLIST
}Filter_mode;

typedef enum {
	_16 = CAN_FILTERSCALE_16BIT,
	_32 = CAN_FILTERSCALE_32BIT
}Filter_scale;

typedef enum {
	disable = CAN_FILTER_DISABLE,
	enable = CAN_FILTER_ENABLE
}Filter_activation;

typedef enum {
	_0 = CAN_FILTER_FIFO0,
	_1 = CAN_FILTER_FIFO1
}FIFO;

typedef enum {
	std = CAN_ID_STD,
	ext = CAN_ID_EXT
}ID_type;

typedef enum {
	data = CAN_RTR_DATA,
	remote = CAN_RTR_REMOTE
}Frame_type;

typedef enum {
	no_message,
	message_received,
	more_message_received,
	error
}read_retval;

typedef struct {
	enum list{empty = 0, filled_harf, filled} list;
	FIFO fifo;
}filter_buffer_status;

class CAN_103xb {
private:
	CAN_HandleTypeDef *hcan;
	uint32_t subscribed_id[28];
	filter_buffer_status filter_buffer[14];
public:
	HAL_StatusTypeDef send(uint32_t ID, ID_type ide, Frame_type rtr, uint8_t *data, uint32_t data_len);	//send data to CAN bus
	HAL_StatusTypeDef subscribe_message(uint32_t ID, ID_type ide, Frame_type rtr, FIFO fifo);			//subscribe to a message to put message in the FIFO
	HAL_StatusTypeDef unsubscribe_message(uint32_t ID, ID_type ide, Frame_type rtr, FIFO fifo);			//desubscribe to a message
	read_retval read(FIFO fifo, uint8_t *data);
	read_retval read(FIFO fifo, uint8_t *data, CAN_RxHeaderTypeDef *RxHeader);
	CAN_103xb(CAN_HandleTypeDef *hcan);
	virtual ~CAN_103xb();
};

} /* namespace CAN */

#endif /* SRC_CAN103XB_H_ */
