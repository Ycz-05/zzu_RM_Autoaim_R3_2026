/*
 * ArraryList.c
 *
 *  Created on: Jan 13, 2024
 *      Author: jxhmak
 */

#include "ArraryList.h"
void arraryListInit(ArraryListParams* params,int max_length_){
	params->start_pos = params->end_pos = 0;
	params->max_length = max_length_;
	params->arr = (uint8_t * ) malloc (params->max_length * sizeof(uint8_t));
}
void arraryListDeInit(ArraryListParams* params){
	free(params->arr);
	params->arr = NULL;
}
bool assertFlag(ArraryListParams* params,int size){ //判断数组状态并纠错
		bool flag = false;
		if(params->end_pos - params->start_pos > 0 && params->start_pos >= 0 && params->end_pos <= params->max_length) //要考虑一下子全部接满情况,end_Pos比实际有效的位序号要大1.
		{
//			if(end_pos <= (USART_RECV_LEN-size)){
//
//			}
				flag = true;
		}else{  //触发清空
				params->end_pos = params->start_pos = 0;
		}
		return flag;
}

bool availableList(ArraryListParams* params){ //判断数组中是否有有效数
		if(params->end_pos - params->start_pos > 0 && params->start_pos >= 0){
				return true;
		}
		return false;
}
bool appendList(ArraryListParams* params,uint8_t* arr_,uint16_t size){
		assertFlag(params,size);
		uint16_t size_ = 0;
		if(params->end_pos > (params->max_length-size)){
				size_ = params->max_length - params->end_pos;
		}else{
				size_ = size;
		}
		memcpy(params->arr + params->end_pos,arr_,size_);
		//clear(receive_buff); //接收缓冲清空。
		params->end_pos += size_;
		return true;
}

//单字节输出,在使用前一定要先调用available，输出第一个数
uint8_t get(ArraryListParams* params){
		uint8_t data = 0;
		if(assertFlag(params,0)){  //当前表有效，获取数据。
				data = params->arr[params->start_pos];
				params->start_pos += 1;
		}
		return data;
};

   
