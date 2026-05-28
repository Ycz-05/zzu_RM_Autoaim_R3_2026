// Created by jxhhbg on 22-11-5.
// Corrected by hjh on 25-9-27.
//
#include "uart.h"
#include <stdio.h>
#include "crc16.h"
#include "crc8.h"
// #include "Crc.h"
bool check_crc16_correct(uint8_t *rxBuffer, uint16_t total_len);

#define FALSE -1
#define TRUE 0
#define RED    "\033[0m\033[1;31m"
#define CEND "\033[0m"
#define BLUE  "\033[0;32;34m"

int flag_register = 30 << 8 | 0b00000001;
int crc16_t;

uint8_t readbuff[255];
uint8_t sendbuff[255];
crc_16to2 crc_addnum;
float error_lost = 50.0;
//计时
clock_t start = 0;
clock_t end = 0;
clock_t connection_clock = 0;
float send_tim = 0;

//
uint8_t _rxBuf[MAX_PACKET_SIZE];
uint8_t _rxBufPos = 0;

Gimbal gimbal_receive ={27.0,-12.6};

//
bool __attribute__((weak)) WriteDataWapper(const unsigned char* pData, unsigned int length){
    printf("-----xxxx\r\n");
}
//bool  WriteDataWapper(const unsigned char* pData, unsigned int length){
//    printf("-----xxxx\r\n");
//}
//
//ENCODE encode_ = encode;
f2ch10 sendcache;
DefUartParams def_uart_params;

///
/// \param params
void defUartInit(){
    arraryListInit(&def_uart_params.arrary_list,CACHE_LENGTH);
}
///
/// \param params
void defUartDeinit(){
    arraryListDeInit(&def_uart_params.arrary_list);
}

/// @brief 检查接收数据的掉包率
/// @param data_length 
/// @param extra_data 
/// @param num 
void cntRead(uint8_t num){
    static uint8_t cnt = 0;
    static uint8_t last_cnt = 0;
    static unsigned int error = 0,loop_times=0;
    last_cnt = cnt;
    cnt = num;  //num get
    if(cnt != last_cnt + 1 && cnt != 0 ){
        error++;
    }
    loop_times++;

    if(loop_times%300 == 0) {   //new feature: changleable cnt_fre.
        error_lost = (float)error/(float)loop_times*100;
        error = loop_times = 0;
    }
}
void cntLossConnection(){ //运行在发送线程中，用于监测接收断链
    if(((clock() - connection_clock)*1000/CLOCKS_PER_SEC) >= 300){ //断联
        error_lost = -100;
    }
}

int encode_(int (*ENCODE)(int id,uint8_t* write_data),int id,uint8_t* write_data){
    return ENCODE(id,write_data);
}

bool decode_(bool (*DECODE)(uint8_t* data),uint8_t* data){
    bool res = DECODE(data);
    cntRead(data[3]);
    return res;
}

/// @brief 用于准备发送的数据包
/// @param len 
/// @param id 
/// @param cache 
// void prepareSend(int data_length,int extra_data,int id,f2ch10 *cache){
//     static uint8_t cnt = 0; 
//     sendbuff[0] = 0xa5;
//     sendbuff[1] = 0xa8;
//     sendbuff[2] = id;
//     sendbuff[3] = cnt;
//     sendbuff[4] = data_length;

//     for(int i = extra_data-2; i < data_length+extra_data-2; i++){
//         sendbuff[i] = cache->ch[i-extra_data+2];
//     }
//     crc_addnum.sh[0]=crc16(sendbuff,data_length+extra_data-2);

//     sendbuff[data_length+extra_data-2] = crc_addnum.ch[0];    //low
//     sendbuff[data_length+extra_data-1] = crc_addnum.ch[1];    //high
//     cnt++;if(cnt>200) cnt = 0;
//  }

void prepareSend(int data_length, int extra_data, int id, f2ch10 *cache)
{
    static uint8_t cnt = 0;
    sendbuff[0] = 0xa5;
    sendbuff[1] = data_length & 0xff;
    sendbuff[2] = (data_length>>8) & 0xff;

    sendbuff[3] = crc_8(&sendbuff[0],3);

    sendbuff[4] = id & 0xff;
    sendbuff[5] = (id >> 8) & 0xff;

    sendbuff[6] = flag_register & 0xff;
    sendbuff[7] = (flag_register >> 8) & 0xff;

    for (int i = extra_data - 2; i < data_length + extra_data - 2; i++)
    {
        sendbuff[i] = cache->ch[i - extra_data + 2];
    }

    crc16_t = crc_16(&sendbuff[0], data_length + 6);

    sendbuff[data_length + extra_data - 2] = crc16_t & 0xff;     // low
    sendbuff[data_length + extra_data - 1] = (crc16_t >> 8) & 0xff; // high

    cnt++;
    if (cnt > 200)
        cnt = 0;
}

//在shell输出字，用于debug
void print_frame(const char *desc,uint8_t *buf,int size)
{
    int i;
    printf(RED"[%s] [LEN=%d]"CEND,desc,size);
    for(i=0; i<size; i++)
    {
        printf(BLUE"[%.2x]"CEND,buf[i]);
    }
    printf("\n");
}



// Shift the bytes in the RxBuf down by cnt bytes
//将RxBuf中的字节下移cnt字节
void shiftRxBuffer(uint8_t cnt)
{
    // If removing the whole thing, just set pos to 0
    if (cnt >= _rxBufPos)
    {
        _rxBufPos = 0;
        return;
    }

//    if (cnt == 1 && onOobData)
//        onOobData(_rxBuf[0]);

    // Otherwise do the slow shift down
    uint8_t *src = &_rxBuf[cnt];
    uint8_t *dst = &_rxBuf[0];
    _rxBufPos -= cnt;
    uint8_t left = _rxBufPos;
    while (left--)
        *dst++ = *src++;
}

// void handleByteReceived()
// {
//     bool reprocess;
//     do
//     {
//         reprocess = false;
//         if (_rxBufPos > extra_len) //数组存在数据
//         {
//             uint8_t len = _rxBuf[4];
//             if (len < 2 || len > (MAX_PAYLOAD_LEN) || _rxBuf[0]!= 0xfe || _rxBuf[1] != 0xa8) //一定不为crsf类型
//             {
//                 shiftRxBuffer(1);
//                 reprocess = true;
//             }

//             else if (_rxBufPos >= (len + 7)) //可能为crsf数据，进行crc校验。
//             {

//                 if (check_crc16(_rxBuf,len+extra_len-2,&crc_addnum))
//                 {
//                     decode_(decode,_rxBuf);
// #ifdef DEBUG
//                     print_frame("PSD",(uint8_t*)_rxBuf,len+extra_len);
// #endif
//                     shiftRxBuffer(len + extra_len);
//                     reprocess = true;
//                 }
//                 else
//                 {
//                     shiftRxBuffer(1);
//                     reprocess = true; //并重新循环，直到找到或者没有数据了
//                 }
//             }  // if complete packet
//         } // if pos > 1
//     } while (reprocess);
// }


// 一个跳过所有CRC校验的、仅用于调试的解码函数
void handleByteReceived()
{
    bool reprocess = true;

    while (reprocess) {
        reprocess = false;

        if (_rxBufPos < 4) {
            return;
        }

        if (_rxBuf[0] != 0xa5) {
            shiftRxBuffer(1);
            reprocess = true;
            continue;
        }
        
        uint16_t payload_len = (uint16_t)_rxBuf[1] | ((uint16_t)_rxBuf[2] << 8);
        uint16_t total_len = payload_len + 8;
        
        // 合理性检查：确保解析出的长度不会导致程序崩溃
        if (total_len > MAX_PACKET_SIZE || total_len < 8) {
            shiftRxBuffer(1);
            reprocess = true;
            continue;
        }

        if (_rxBufPos < total_len) {
            return;
        }
        
        // --- 成功捕获一个完整的数据包，开始提取数据 ---

        // printf("\n---=== [NO-CRC MODE] Packet Found & Decoded ===---\n");

        // 1. 提取 CMD_ID (字节 4, 5)
        uint16_t cmd_id = (uint16_t)_rxBuf[4] | ((uint16_t)_rxBuf[5] << 8);
        // printf("  - CMD_ID: 0x%04x (%u)\n", cmd_id, cmd_id);

        // 2. 提取 Flags Register (字节 6, 7)
        uint16_t flags_register = (uint16_t)_rxBuf[6] | ((uint16_t)_rxBuf[7] << 8);
        // printf("  - Flags Register: 0x%04x\n", flags_register);

        // 3. 提取浮点数 (从字节 8 开始)
        // 检查是否有足够的空间来存放浮点数
        if (payload_len >= 14) { // 至少要包含 cmd_id(2) + flags(2) + 3*floats(12) = 16，但 seasky 协议里 payload_len 是 14
            float yaw, pitch, roll;

            // 提取第一个 float (yaw)
            memcpy(&yaw, &_rxBuf[8], sizeof(float));
            gimbal_receive.yaw = yaw;
            // 提取第二个 float (pitch)
            memcpy(&pitch, &_rxBuf[12], sizeof(float));
            gimbal_receive.pitch = pitch;
            // 提取第三个 float (roll)
            memcpy(&roll, &_rxBuf[16], sizeof(float));
            
            // printf("  - Yaw:   %f\n", yaw);
            // printf("  - Pitch: %f\n", pitch);
            // printf("  - Roll:  %f\n", roll);

        } else {
            printf("  - Payload length (%u) is too short to contain 3 floats.\n", payload_len);
        }
        
        // 移除已处理的数据包
        shiftRxBuffer(total_len);
        reprocess = true;
    }
}


/// @brief 用于发送的主程序
/// @param id
/// @return
bool defUartSend(int id){
    static int length = 0;
    start = clock();
    length = encode_(encode,id,sendcache.ch); //数据位长度
    prepareSend(length,extra_len,id,&sendcache);
    //以后尝试使用async_write
    if(!WriteDataWapper(sendbuff,length+extra_len)) return false;
    end = clock();
    send_tim = (end-start)*1000/CLOCKS_PER_SEC;  //ms
    return true;
}

void defUartRead(uint8_t* readbuff, int nread) {
    for (int i = 0; i < nread; ++i) {
        if (_rxBufPos < MAX_PACKET_SIZE) {
            _rxBuf[_rxBufPos++] = readbuff[i];
        } 
    }
    handleByteReceived();

    // 这个验证信息可以暂时注释掉，减少干扰
    // printf("[VERIFICATION] defUartRead function was called with %d bytes, buffer size is now %d.\n", nread, _rxBufPos);
}


bool check_crc16_correct(uint8_t *rxBuffer, uint16_t total_len) {
    if (total_len < 2) {
        return false;
    }
    uint16_t calculated_crc = crc_16(rxBuffer, total_len - 2);
    uint16_t received_crc = (uint16_t)rxBuffer[total_len - 2] | ((uint16_t)rxBuffer[total_len - 1] << 8);
    return calculated_crc == received_crc;
}




