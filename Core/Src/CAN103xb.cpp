/*
 * CAN103xb.cpp
 *
 *  Created on: Mar 25, 2022
 *      Author: paripal
 */

#include "CAN103xb.h"

namespace stm_CAN {

HAL_StatusTypeDef CAN_103xb::send(uint32_t ID, ID_type ide, Frame_type rtr, uint8_t *data, uint32_t data_len) {
	if(data_len > 8)
		return HAL_ERROR;
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.StdId = ID;
	TxHeader.ExtId = ID;
	TxHeader.IDE = ide;
	TxHeader.RTR = rtr;
	TxHeader.DLC = data_len;
	TxHeader.TransmitGlobalTime = DISABLE;
	return HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
}

HAL_StatusTypeDef CAN_103xb::subscribe_message(uint32_t ID, ID_type ide, Frame_type rtr, FIFO fifo) {
	uint32_t id_for_filter = (ID << ((ide == ID_type::std)?21:3)) | ide | rtr;
	int filter_buffer_index = -1;
	for(int i = 0; i < 14; i++){
		switch(this->filter_buffer[i].list){
		case filter_buffer_status::list::empty:
			filter_buffer_index = i;
			break;
		case filter_buffer_status::list::filled_harf:
			if(this->filter_buffer[i].fifo == fifo){
				if(this->subscribed_id[i*2] == id_for_filter)
					return HAL_OK;
				else{
					filter_buffer_index = i;
					break;
				}
			}else{
				continue;
			}
		case filter_buffer_status::list::filled:
			if(this->filter_buffer[i].fifo == fifo){
				if(this->subscribed_id[i*2] == id_for_filter || this->subscribed_id[i*2+1] == id_for_filter)
					return HAL_OK;
				else{
					continue;
				}
			}else{
				continue;
			}
		}
	}
	if(filter_buffer_index == -1)
		return HAL_ERROR;
	uint32_t id_for_filter_first = (this->filter_buffer[filter_buffer_index].list == filter_buffer_status::list::empty)?id_for_filter:this->subscribed_id[filter_buffer_index*2];
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh = id_for_filter_first >> 16;
	filter.FilterIdLow = id_for_filter_first & 0xffff;
	filter.FilterMaskIdHigh = id_for_filter >> 16; 
	filter.FilterMaskIdLow = id_for_filter & 0xffff;
	filter.FilterScale = Filter_scale::_32;
	filter.FilterMode = Filter_mode::list;
	filter.FilterFIFOAssignment = fifo;
	filter.FilterActivation = Filter_activation::enable;
	filter.FilterBank = filter_buffer_index;
	if(HAL_CAN_ConfigFilter(this->hcan, &filter) != HAL_OK)
		return HAL_ERROR;
	this->filter_buffer[filter_buffer_index].fifo = fifo;
	if(this->filter_buffer[filter_buffer_index].list == filter_buffer_status::list::empty){
		this->filter_buffer[filter_buffer_index].list = filter_buffer_status::list::filled_harf;
		this->subscribed_id[filter_buffer_index*2] = id_for_filter;
		return HAL_OK;
	}else if(this->filter_buffer[filter_buffer_index].list == filter_buffer_status::list::filled_harf){
		this->filter_buffer[filter_buffer_index].list = filter_buffer_status::list::filled;
		this->subscribed_id[filter_buffer_index*2+1] = id_for_filter;
		return HAL_OK;
	}else{
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef CAN_103xb::unsubscribe_message(uint32_t ID, ID_type ide, Frame_type rtr, FIFO fifo) {
	uint32_t id_for_filter = (ID << ((ide == ID_type::std)?21:3)) | ide | rtr;
	int filter_buffer_index = -1;
	for(int i = 0; i < 14; i++){
		switch(this->filter_buffer[i].list){
		case filter_buffer_status::list::empty:
			continue;
		case filter_buffer_status::list::filled_harf:
			if(this->filter_buffer[i].fifo == fifo){
				if(this->subscribed_id[i*2] == id_for_filter){
					filter_buffer_index = i;
					break;
				}else{
					continue;
				}
			}else{
				continue;
			}
		case filter_buffer_status::list::filled:
			if(this->filter_buffer[i].fifo == fifo){
				if(this->subscribed_id[i*2] == id_for_filter || this->subscribed_id[i*2+1] == id_for_filter){
					filter_buffer_index = i;
					break;
				}else{
					continue;
				}
			}else{
				continue;
			}
		}
	}
	if(filter_buffer_index == -1)
		return HAL_OK;
	uint32_t id_for_filter_first = (this->subscribed_id[filter_buffer_index*2] == id_for_filter)?
		this->subscribed_id[filter_buffer_index*2+1]
		:this->subscribed_id[filter_buffer_index*2];
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh = id_for_filter_first >> 16;
	filter.FilterIdLow = id_for_filter_first & 0xffff;
	filter.FilterMaskIdHigh = id_for_filter_first >> 16;
	filter.FilterMaskIdLow = id_for_filter_first & 0xffff;
	filter.FilterScale = Filter_scale::_32;
	filter.FilterMode = Filter_mode::list;
	filter.FilterFIFOAssignment = fifo;
	filter.FilterActivation = (this->filter_buffer[filter_buffer_index].list == filter_buffer_status::list::filled)?Filter_activation::enable:Filter_activation::disable;
	filter.FilterBank = filter_buffer_index;
	if(HAL_CAN_ConfigFilter(this->hcan, &filter) != HAL_OK)
		return HAL_ERROR;
	this->filter_buffer[filter_buffer_index].fifo = fifo;
	if(this->filter_buffer[filter_buffer_index].list == filter_buffer_status::list::filled_harf){
		this->filter_buffer[filter_buffer_index].list = filter_buffer_status::list::empty;
		this->subscribed_id[filter_buffer_index*2] = 0;
		this->subscribed_id[filter_buffer_index*2+1] = 0;
		return HAL_OK;
	}else if(this->filter_buffer[filter_buffer_index].list == filter_buffer_status::list::filled){
		this->filter_buffer[filter_buffer_index].list = filter_buffer_status::list::filled_harf;
		if(this->subscribed_id[filter_buffer_index*2] == id_for_filter){
			this->subscribed_id[filter_buffer_index*2+1] = 0;
		}else{
			this->subscribed_id[filter_buffer_index*2] = 0;
		}
		return HAL_OK;
	}else{
		return HAL_ERROR;
	}
}

read_retval CAN_103xb::read(FIFO fifo, uint8_t *data){
	uint32_t fifofilllevel = HAL_CAN_GetRxFifoFillLevel(this->hcan, fifo);
	if(fifofilllevel == 0)
		return read_retval::no_message;
	CAN_RxHeaderTypeDef RxHeader;
	if(HAL_CAN_GetRxMessage(this->hcan, fifo, &RxHeader, data) != HAL_OK)
		return read_retval::error;
	if(fifofilllevel == 1){
		return read_retval::message_received;
	}else{
		return read_retval::more_message_received;
	}
}

read_retval CAN_103xb::read(FIFO fifo, uint8_t *data, CAN_RxHeaderTypeDef *RxHeader){
	uint32_t fifofilllevel = HAL_CAN_GetRxFifoFillLevel(this->hcan, fifo);
	if(fifofilllevel == 0)
		return read_retval::no_message;
	if(HAL_CAN_GetRxMessage(this->hcan, fifo, RxHeader, data) != HAL_OK)
		return read_retval::error;
	if(fifofilllevel == 1){
		return read_retval::message_received;
	}else{
		return read_retval::more_message_received;
	}
}

CAN_103xb::CAN_103xb(CAN_HandleTypeDef *hcan) {
	// TODO Auto-generated constructor stub
	this->hcan = hcan;
	HAL_CAN_Start(this->hcan);
}

CAN_103xb::~CAN_103xb() {
	// TODO Auto-generated destructor stub
	HAL_CAN_Stop(this->hcan);
}

} /* namespace CAN */
