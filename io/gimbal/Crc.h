//
// Created by jxhhbg on 22-11-12.
//

#ifndef UART_CRC_H
#define UART_CRC_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>

#include<stdint.h>
#define DEBUG
typedef union {
    uint16_t sh[1];
    uint8_t ch[2];
} crc_16to2;

extern uint16_t crc16(const uint8_t *buf, uint32_t len);
extern uint8_t crc8(const uint8_t *ptr, uint8_t len);
/*
 * @parma len 指的是整个帧去除后两位的长度
 * */
extern bool check_crc8(uint8_t *rxBuffer, int len);
/*
 * @parma len 指的是整个帧去除后两位的长度
 * */
extern bool check_crc16(uint8_t *rxBuffer, int len,crc_16to2* data_union);

#ifdef __cplusplus
}
#endif
#endif //UART_CRC_H
