//
// Created by jxhhbg on 22-11-5.
// 用于硬件串口控制的父类,涉及硬件读写以及初始化
//
#ifndef UART_COMMON_H__
#define UART_COMMON_H__

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <cerrno>
#include <fcntl.h>  //文件控制
#include <unistd.h>  //Unix 标准函数定义
#include <vector>
#include <cstdlib>
#include <iostream>
#include <termios.h>  //PPSIX 终端控制定义
#include <ctime>
#include <queue>
#include <condition_variable>
#include <stdint.h>

#include "uart_hd.h"
#include "uart.h"


#define QUEUE_SIZE 6
using namespace  std;
//#define DEBUG
#define ROS_SYSTEM

/*********************************************
 * socat -d -d pty,raw,echo=0 pty,raw,echo=0
 *********************************************/
//string ppname =  "/dev/pts/1";

#define PNAME  "/dev/pts/1"
/************************************************
 * 0xfe  msg_id num len pay
 * ad crc1  crc2
 * 1       1     1   1     n     1     1
 * 0       1     2    3         4+n    5+n
**************************************************/

template<int N>
union fl2ch{
    float fl[N];
    unsigned char ch[4*N];
};
template<int N>
union sh2ch{
    uint16_t sh[N];
    uint8_t ch[2*N];
};

class UartLinux {
    public:
        int read_period = 0;
        string init_name;
        struct {
            int nSpeed = 115200;
            char nEvent;
            int nBits;
            int nStop;
            bool wait_uart = true;
            int fd;
        } uart_parma;
    
    private:
        std::thread read_thread_;
        bool is_running_;
    
    public:
        void startReading();
        void stopReading();
    
    private:
        void readLoop();
    
    public:
        fd_set rd;
    
    public:
        UartLinux(string uart_name = "/dev/ttyUSB0", int nSpeed = 115200, char nEvent = 'N', int nBits = 8, int nStop = 1) : 
            uart_parma({nSpeed, nEvent, nBits, nStop}),
            init_name(std::move(uart_name)) {
            is_running_ = false; // 在构造函数里初始化一下标志位
            defUartInit();
        }
    
        bool setUartName(const string& name); // 函数声明
    
        // 这是修改后的析构函数
        ~UartLinux() {
            stopReading();
            defUartDeinit();
        }
    
        string get_uart_name();
        bool init_port(int nSpeed, char nEvent, int nBits, int nStop);
        int set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop);
        bool WriteData(const unsigned char* pData, unsigned int length);
        bool ReadData(); // 这个函数我们虽然没用，但先保留它的声明
    };
extern std::shared_ptr<UartLinux> uart;
#endif