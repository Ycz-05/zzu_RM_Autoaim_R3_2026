//
// Created by jxhhbg on 22-11-5.

#ifndef ASSERT_UART__H
#define ASSERT_UART__H
#ifdef __cplusplus
extern "C" {
#endif
#include "Crc.h"
#include "ArraryList.h"
#include "DataType.h"

#include <time.h>
#define MAX_PACKET_SIZE 256 //根据每帧最大长度修改
#define MAX_PAYLOAD_LEN 13 //每帧数据位最大长度
#define CACHE_LENGTH 100
#define TIME_PER 500
#define DEBUG
extern bool __attribute__((weak)) WriteDataWapper(const unsigned char* pData, unsigned int length);
//
typedef bool (*WRITEDATA)(const unsigned char* pData, unsigned int length);

typedef union{
    float fl[10];
    uint8_t ch[40];
}f2ch10;
//typedef union{
//    uint16_t sh[1];
//    uint8_t ch[2];
//}sh2ch2;

typedef struct{
    crc_16to2 crc_handle;
    ArraryListParams arrary_list;
}DefUartParams;

extern float error_lost;
extern uint8_t readbuff[255];

extern void defUartInit();
extern void defUartDeinit();
extern bool defUartSend(int id);
extern void  defUartRead(uint8_t* readbuff, int nread);
extern void cntLossConnection();
extern void print_frame(const char *desc,uint8_t *buf,int size);
#ifdef __cplusplus
}
#endif



#endif //ASSERT_UART__H
