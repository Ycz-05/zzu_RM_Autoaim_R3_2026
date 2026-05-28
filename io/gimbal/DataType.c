#include "DataType.h"
#include <stdio.h> // <--- 添加这一行！

HeartBeatReceive heartbeat_send = {0,0,0,0B11111111};
HeartBeatSend heartbeat_receive = {0,100,100,0,255,0B11111111};
Gimbal gimbal_send ={87.0,-21.3};
// Gimbal gimbal_receive ={27.0,-12.6};
ControlData control_data ={0,0};
AdditionalData additional_data ={0,0,100,100,0};
ChassisSpeed chassis_send = {12,1,2};
ChassisSpeed chassis_receive = {13,1,3};
YawData yaw_data ={12};
ImuData imu2_data = {12,32,34,5,56,5};
ext_game_status_t game_status_data ={0,0};
const uint8_t extra_len = 10;   //除去数据帧外其他附加位的数量

union{
    float fl[1];
    uint8_t ch[4];
}fl2chtx;
union{
    float fl[1];
    uint8_t ch[4];
}fl2chrx;
void getHeartBeatReceive(uint8_t* data_area){
    static union {
        uint16_t  sh[1];
        uint8_t ch[2];
    }timestamp_union;
    timestamp_union.ch[0] = data_area[0];
    timestamp_union.ch[1] = data_area[1];
    heartbeat_receive.timestamp = timestamp_union.sh[0];

    uint8_t _pos = 2;
    heartbeat_receive.battery = data_area[++_pos];
    heartbeat_receive.life = data_area[++_pos];
    heartbeat_receive.color = data_area[++_pos];
    heartbeat_receive.bullet = data_area[++_pos];
    heartbeat_receive.fault_flag = data_area[++_pos];
}
void getGimbalReceive(uint8_t* data_area){
    // memcpy(fl2chrx.ch,data_area,sizeof(float));
    // gimbal_receive.yaw = fl2chrx.fl[0];
    // memcpy(fl2chrx.ch,data_area+sizeof(float),sizeof(float));
    // gimbal_receive.pitch = fl2chrx.fl[0];
    // printf("[DEBUG] C-Level Received: yaw=%.2f, pitch=%.2f\n", gimbal_receive.yaw, gimbal_receive.pitch);
}
void getAdditional(uint8_t* data_area){
    uint8_t _pos = 0;
    additional_data.launch = data_area[++_pos];
    additional_data.arm = data_area[++_pos];
    additional_data.base_hp_our = data_area[++_pos];
    additional_data.base_hp_enemy = data_area[++_pos];
    additional_data.judge_warning_level = data_area[++_pos];
}
void getChassisReceive(uint8_t* data_area){
    memcpy(fl2chrx.ch,data_area,sizeof(float));
    chassis_receive.vx = fl2chrx.fl[0];
    memcpy(fl2chrx.ch,data_area+sizeof(float),sizeof(float));
    chassis_receive.vy = fl2chrx.fl[0];
    memcpy(fl2chrx.ch,data_area+2*sizeof(float),sizeof(float));
    chassis_receive.vz = fl2chrx.fl[0];
}
void getYawData(uint8_t* data_area){
    memcpy(fl2chrx.ch,data_area,sizeof(float));
    yaw_data.yaw = fl2chrx.fl[0];
}
void getGameStatus(uint8_t* data_area){
    union{
        uint16_t sh[0];
        uint8_t ch[2];
    }remain_time_union;
    remain_time_union.ch[0] = data_area[0];
    remain_time_union.ch[1] = data_area[1];

    game_status_data.stage_remain_time = remain_time_union.sh[0];
    game_status_data.game_progress = data_area[2];

}

void setHeartBeatSend(uint8_t* data_area){
    static union {
        uint16_t  sh[1];
        uint8_t ch[2];
    }timestamp_union;
    timestamp_union.sh[0] = heartbeat_send.timestamp;
    uint8_t _pos = 0;
    data_area[++_pos] = timestamp_union.ch[0];
    data_area[++_pos] = timestamp_union.ch[1];
    data_area[++_pos] = heartbeat_send.set_launch;
    data_area[++_pos] = heartbeat_send.set_arm;
    data_area[++_pos] = heartbeat_send.fault_flag;
}
void setGimbalSend(uint8_t* data_area){
    fl2chtx.fl[0] = gimbal_send.yaw;
    memcpy(data_area,fl2chtx.ch,sizeof(float));
    fl2chtx.fl[0] = gimbal_send.pitch;
    memcpy(data_area+sizeof(float),fl2chtx.ch,sizeof(float));
    data_area[sizeof(float) * 2] = gimbal_send.fire; 
}
void setControlData(uint8_t* data_area){
    fl2chtx.fl[0] = control_data.velocity_top;
    memcpy(data_area,fl2chtx.ch,sizeof(float));
    data_area[sizeof(float)] = control_data.shoot;
}
void setChassisSend(uint8_t* data_area){
    fl2chtx.fl[0] = chassis_send.vx;
    memcpy(data_area,fl2chtx.ch,sizeof(float));
    fl2chtx.fl[0] = chassis_send.vy;
    memcpy(data_area+sizeof(float),fl2chtx.ch,sizeof(float));
    fl2chtx.fl[0] = chassis_send.vz;
    memcpy(data_area+2*sizeof(float),fl2chtx.ch,sizeof(float));
}

/// @brief 对数据编码成消息包，用于发送给电控
/// @param  id 制定的数据类型
/// @return 一个信息帧的总长。
int encode(int id,uint8_t* write_data) {
    static int len;  //数据位长度
    switch(id){
        case heartbeat:
            len = 5;
            setHeartBeatSend(write_data);
            break;
        case gimbal:
            len = 9;
            setGimbalSend(write_data);
            break;
        case control:
            len = 5;
            setControlData(write_data);
            break;
        case chassis:
            len = 12;
            setChassisSend(write_data);
            break;
        default:
//            cout<<"<undefined message>"<<id<<endl;
            len = 0; break;
    }
    return len;
}

/// @brief 用于解码
/// @return
// bool decode(uint8_t* readdata){
//     // static int len = 0;
//     // len = readdata[4];
//     switch(*(readdata+2)){    //id段分析。//疑似有错应该+4
//         case heartbeat:
//             getHeartBeatReceive(readdata+extra_len-2);
//             break;
//         case gimbal:
//             getGimbalReceive(readdata+extra_len-2);
//             break;
//         case additional:
//             getAdditional(readdata+extra_len-2);
//             break;
//         case chassis:
//             getChassisReceive(readdata+extra_len-2);
//             break;
//         case imu2:
//             break;
//         case yaw:
//             getYawData(readdata+extra_len-2);
//             break;
//         case game_status:
//             getGameStatus(readdata+extra_len-2);
// //        default:std::cout<<"unexpected uart id"<<endl; break;
//     }
// //#ifdef DEBUG
// //        cout << "\nserial----------------------------------------------------------------\n";
// //        cout << "tile x:\t" << gimble.fl[0] << endl
// //             << "tile y:\t" << gimble.fl[1]<< endl;
// //#endif
//     //end = clock();
//     //read_period = (end-start)*1e6/CLOCKS_PER_SEC;  //us
//     return true;
// }

bool decode(uint8_t* readdata){
    // 根据新协议, ID 是一个16位的整数, 从第4个字节开始(低字节在前)
    uint16_t id = (uint16_t)readdata[4] | ((uint16_t)readdata[5] << 8);

    switch(id){    // 使用我们刚刚解析出的正确ID
        case heartbeat:
            // 注意：你的旧代码指针 readdata+extra_len-2 是 readdata+8，这是对的
            getHeartBeatReceive(readdata + 8);
            printf("SOMETHING RECEIVED!");
 
            break;
        case gimbal:
            getGimbalReceive(readdata + 8);
            printf("SOMETHING RECEIVED!");

            break;
        case additional:
            getAdditional(readdata + 8);
            printf("SOMETHING RECEIVED!");

            break;
        case chassis:
            getChassisReceive(readdata + 8);
            printf("SOMETHING RECEIVED!");

            break;
        case imu2:
            printf("SOMETHING RECEIVED!");

            break;
        case yaw:
            getYawData(readdata + 8);
            printf("SOMETHING RECEIVED!");

            break;
        case game_status:
            getGameStatus(readdata + 8);
            printf("SOMETHING RECEIVED!");
            break;
        default:
            // 可以加一个打印，看看是否收到了未知的ID
            printf("SOMETHING RECEIVED!");
            break;
    }
    return true;
}


