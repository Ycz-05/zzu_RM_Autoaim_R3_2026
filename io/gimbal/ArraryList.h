/*
 * ArraryList.h
 *
 *  Created on: Jan 13, 2024
 *  Author: jxhmak
 *  使用C，通用性保证
 */

#ifndef SYSTEM_MATH_LIB_ARRARYLIST_H_
#define SYSTEM_MATH_LIB_ARRARYLIST_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <string.h>
//#include <cstdlib>
#include <stdbool.h>
#include <stdlib.h>
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef struct{
	int max_length;
	int end_pos;
	int start_pos;
	uint8_t* arr;
}ArraryListParams;

extern void arraryListInit(ArraryListParams* params,int max_length_);
extern void arraryListDeInit(ArraryListParams* params);
extern bool appendList(ArraryListParams* params,uint8_t* arr_,uint16_t size);
extern bool availableList(ArraryListParams* params); //判断数组中是否有有效数
extern uint8_t get(ArraryListParams* params);
#ifdef __cplusplus
}
#endif
#endif /* SYSTEM_MATH_LIB_ARRARYLIST_H_ */
