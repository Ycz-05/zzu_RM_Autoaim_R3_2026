#include"uart_hd.h"
#include <fcntl.h>  //文件控制
#include <unistd.h>  //Unix 标准函数定义
#include <cstdlib>
#include <iostream>
#include <termios.h>  //PPSIX 终端控制定义
#include <ostream>
#define FALSE -1
#define TRUE 0

    


  
std::shared_ptr<UartLinux> uart(new UartLinux());
int set_Parity (int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if (tcgetattr (fd, &options) != 0)
    {
        perror ("SetupSerial 1");
        return (FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf (stderr, "Unsupported data size\n");
            return (FALSE);
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;	/* Clear parity enable */
            options.c_iflag &= ~INPCK;	/* Enable parity checking */
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;	/* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;	/* Enable parity */
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;	/* Disnable parity checking */
            break;
        case 'S':
        case 's':			/*as no parity */
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf (stderr, "Unsupported parity\n");
            return (FALSE);
    }

    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf (stderr, "Unsupported stop bits\n");
            return (FALSE);
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush (fd, TCIFLUSH);
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;	/* Update the options and do it NOW */
    if (tcsetattr (fd, TCSANOW, &options) != 0)
    {
        perror ("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}
// =========================================================================================================

string UartLinux::get_uart_name() {
    FILE *ls = popen("ls /dev/ttyUSB* --color=never", "r");
    char name[20] = {0};
    fscanf(ls, "%s", name);
    return name;
}

int UartLinux::set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop){
    struct termios Opt;
    tcgetattr(fd, &Opt);
    tcflush (fd, TCIOFLUSH);    // 清除缓冲区
    //设置波特率
    cfsetispeed(&Opt, B115200);
    cfsetospeed(&Opt, B115200);
    int status = tcsetattr (fd, TCSANOW, &Opt); //设置属性
    if (status != 0)
    {
        perror ("tcsetattr fd1");
        return 0;
    }
    tcflush (fd, TCIOFLUSH);     //清除缓冲区
    struct termios Opt_;
    tcgetattr(fd, &Opt_);
    Opt_.c_cflag &= ~CSIZE;   //控制模式标志 add 确定字符长度
 
    //设置数据位
    switch (nBits) {
        case 7:
            Opt_.c_cflag |= CS7;
            break;
        case 8:
            Opt_.c_cflag |= CS8;
            break;
        default:
            break;
    }
    //设置校验位
    switch (nEvent) {
        case 'O':  //奇校验
            Opt_.c_cflag |= PARENB;
            Opt_.c_cflag |= PARODD;
            Opt_.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':  //偶校验
            Opt_.c_iflag |= (INPCK | ISTRIP);
            Opt_.c_cflag |= PARENB;
            Opt_.c_cflag &= ~PARODD;
            break;
        case 'N':  //无校验
            Opt_.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }
    //设置停止位
    if (nStop == 1) {
        Opt_.c_cflag &= ~CSTOPB;
    } else if (nStop == 2) {
        Opt_.c_cflag |= CSTOPB;
    }

    //additional
    //https://blog.csdn.net/wmdscjhdpy/article/details/107161147
    Opt_.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input LOCAL*/
	Opt_.c_oflag  &= ~OPOST;   /*Output*/
	Opt_.c_iflag &= ~(IXON | IXOFF | IXANY |BRKINT | ICRNL | ISTRIP );

    tcflush(fd, TCIOFLUSH);         //清除输入输出
    //https://blog.csdn.net/zcabcd123/article/details/7595970 
                                         
    Opt_.c_cc[VTIME] = 1;      //非规范模式读取时的超时时间(单位:0.1秒)，从接收到最后一个字节开始计时，如果超时，则退出READ， //ms
    Opt_.c_cc[VMIN] = 255;	/* Update the options and do it NOW */
    //options.c_cc[VTIME] = 150;
    //options.c_cc[VMIN] = 0;	/* Update the options and do it NOW */
    if (tcsetattr (fd, TCSANOW, &Opt_) != 0)  //导入配置
    {
        perror ("SetupSerial");
        return 0;
    }
    return 1;
}

bool UartLinux::init_port(int nSpeed, char nEvent, int nBits, int nStop) {
    string name =
    //PNAME;
    #ifdef ROS_SYSTEM  //ros get name
    init_name;
    #else 
    get_uart_dev_name(); //uart find name
    #endif
    if (name == "") {
        return false;
    }
    if ((uart_parma.fd = open(name.data(), O_RDWR)) == -1) {   //打开串口
        cout<<"the current io name is "<< name <<endl;
        cout<<"open error"<<endl;
        return false;
    }
    if (set_opt(uart_parma.fd, uart_parma.nSpeed, uart_parma.nEvent, uart_parma.nBits, uart_parma.nStop) < 0) {
        return false;
    }
    return true;
}

bool UartLinux::WriteData(const unsigned char *pData, unsigned int length) {
    int cnt = 0, curr = 0;
    // if (uart_parma.fd <= 0){
    //     if(uart_parma.wait_uart){
    //         init_port(uart_parma.nSpeed, uart_parma.nEvent, uart_parma.nBits, uart_parma.nStop);
    //     }
    //     return false;
    // }
    while ((curr = write(uart_parma.fd, pData + cnt, length - cnt)) > 0 && (cnt += curr) < length);
    if (curr < 0) {
        cout<<"Serial_offline or not set uart_port"<<endl;
        close(uart_parma.fd);
        if (uart_parma.wait_uart) {
            init_port(uart_parma.nSpeed, uart_parma.nEvent, uart_parma.nBits, uart_parma.nStop);
        }
        return false;
    }
    return true;
}


// bool UartLinux::ReadData() {
//     FD_ZERO(&rd);
//     FD_SET(uart_parma.fd, &rd);
//     int read_status = 0;
//     int dest_cnt = 0;
//     int nread = 0; //存储读取到数据位数
//     while(FD_ISSET(uart_parma.fd, &rd))
//     {
//         //https://blog.csdn.net/zhaodeming000/article/details/98618097
//         if(select(uart_parma.fd+1, &rd, NULL,NULL,NULL) < 0)
//         {
//         perror("select error\n");
//         }
//         else
//         {
//             while((nread = read(uart_parma.fd, readbuff, sizeof(readbuff))) > 0)  //一次读取一帧数据
//             {
//                 #ifdef DEBUG
//                 //printf("nread = %d,%s\n",nread, readbuff);
//                 printf("loss_cnt = %f\n",error_lost);
//                 print_frame("RAW",(uint8_t*)readbuff,nread);
//                 #endif
//                 defUartRead(readbuff,nread);
//             }

//         }
//     }
//     close(uart_parma.fd);
// }

bool WriteDataWapper(const unsigned char *pData, unsigned int length) {
    //uart.WriteData(pData, length);
    //printf("----xxxxx");
    uart->WriteData(pData,length);
}


// ==================== 复制并粘贴以下所有代码到 uart_hd.cpp 的文件末尾 ====================

// --- 第1部分：补全 setUartName 的实现 ---
bool UartLinux::setUartName(const string& name){
    init_name = name;
    if (uart_parma.fd > 0) {
        close(uart_parma.fd); // 如果之前已打开，先关闭
    }

    if (uart_parma.wait_uart) {
        cout << "[UART] Wait for serial device " << name << " to be ready..." << endl;
        // 使用循环尝试打开串口，直到成功
        while (!init_port(uart_parma.nSpeed, uart_parma.nEvent, uart_parma.nBits, uart_parma.nStop)) {
            // 每隔1秒重试一次
            std::this_thread::sleep_for(std::chrono::seconds(1)); 
        }
        cout << "[UART] Port " << name << " set successfully!" << endl;
    } else {
        if (init_port(uart_parma.nSpeed, uart_parma.nEvent, uart_parma.nBits, uart_parma.nStop)) {
            cout << "[UART] Port " << name << " set successfully!" << endl;
        } else {
            cout << "[UART] Error: Failed to set port " << name << endl;
            return false;
        }
    }
    return true; 
}


// --- 第2部分：补全 ReadData 的空实现 (为了解决链接错误) ---
bool UartLinux::ReadData() {
    // 这个函数已经被 readLoop 线程替代，不再直接使用。
    // 提供一个空实现以解决临时的链接错误。
    return false;
}


// --- 第3部分：补全线程控制函数的实现 ---

// 启动接收线程
void UartLinux::startReading() {
    if (uart_parma.fd <= 0) {
        std::cerr << "[UART] Error: Cannot start reading, serial port is not open." << std::endl;
        return;
    }
    if (is_running_) {
        return; // 如果已经在运行，则直接返回
    }
    is_running_ = true;
    // 创建并分离一个后台线程，该线程将执行 readLoop 函数
    read_thread_ = std::thread(&UartLinux::readLoop, this);
    read_thread_.detach(); 
    std::cout << "[UART] Serial reading thread started." << std::endl;
}

// 停止接收线程
void UartLinux::stopReading() {
    is_running_ = false;
    // is_running_ 标志将使 readLoop 线程在下一次循环时自然退出。
    // 因为线程是 detached 的，我们不需要也不能 join 它。
}

// 线程主循环 (这是新的、正确的接收逻辑)
void UartLinux::readLoop() {
    unsigned char read_buffer[255];
    int nread = 0;

    // 使用 is_running_ 作为循环条件
    while (is_running_) {
        // read() 函数是阻塞的，它会在这里等待数据到来
        nread = read(uart_parma.fd, read_buffer, sizeof(read_buffer));

        if (nread > 0) {
            // 成功读到数据，调用 C 语言解析函数
            defUartRead(read_buffer, nread);
        } else if (nread < 0) {
            // 读取出错
            if (is_running_) { // 只有在不是主动停止的情况下才报错
                perror("[UART] Serial read error");
            }
            break; // 退出循环
        }
        // nread == 0 表示超时但没读到数据 (根据VTIME和VMIN设置)，继续下一次循环
    }
    std::cout << "[UART] Serial reading thread stopped." << std::endl;
    if (uart_parma.fd > 0) {
        close(uart_parma.fd); // 线程退出时关闭串口
        uart_parma.fd = -1;   // 标记为已关闭
    }
}

// =======================================================================================


/************ other select   no need *********/
    /*
    termios newtio{}, oldtio{};  //
    if (tcgetattr(fd, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            break;
    }
    switch (nEvent) {
        case 'O':  //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':  //偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':  //无校验
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }
    switch (nSpeed) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }
    if (nStop == 1) {
        newtio.c_cflag &= ~CSTOPB;
    } else if (nStop == 2) {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
     */



