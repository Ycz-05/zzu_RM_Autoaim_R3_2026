#ifndef DATA_DEFINE_H__
#define DATA_DEFINE_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern const uint8_t extra_len;   //除去数据帧外其他附加位的数量
// ======================== 消息定义 ========================================
enum msg_id {
    heartbeat,//【底盘】[云台]//心跳包，双向2HZ，电控回传含电量，生命，装甲板颜色等信息，视觉含使能命令，设置装甲板颜色命令
    gimbal,  //【云台】云台数据，双向100-200HZ到电控是控制，到视觉是反馈（绝对角度）
    control, //[底盘+云台] 视觉到电控，50Hz,发送小陀螺指令和射击指令。
    additional, //【云台】状态数据（回传解锁状态，枪管状态，等信息），单向，5HZ电控到视觉
    chassis,   //【底盘】双向100-200HZ，电控回底盘速度数据。视觉传来控制速度数据
    imu2,      //【云台】【尽量实现，如果无法实现视觉可以采用外置IMU模块】单向100HZ，电控向视觉传云台imu数据 （可选）
    yaw,       //【底盘/云台】单向40-100HZ，电控向视觉传yaw轴与底盘夹角 （也可以传输经过修正的云台数据，那样这一项就可以省去）
    game_status, //【底盘】比赛进行阶段信息1HZ
};

// ========================== 数据定义 ===========================================
//len = 5
//视觉发出：|time1|time2|launch|arm|fault_flag|
typedef struct {
    uint16_t timestamp;   //时间戳，由于判断心跳包存活状态，以视觉为基准,单位是s*10,整数
    uint8_t set_launch,       //用于使能哨兵底盘与云台，=1时，正常运行。=0时，应关闭底盘，速度归零,清除此前小陀螺等特殊状态，回归底盘云台分离状态，云台保持禁止稳定状态。
    set_arm;       //用于使能炮管，=1，启用。=0，关闭。注意，非发射开关，仅代表启用。
    uint8_t fault_flag; //故障标志，0x111111为正常。
} HeartBeatReceive;
//   //####当心跳包严重超时或者接收不到心跳包时，停用底盘与枪管。#####

//len = 7
//电控发出time1|time2|battery|life|color|bullet|fault_flag|
typedef struct {
    uint16_t timestamp;
    uint8_t battery; //剩余电量，百分制度
    uint8_t life; //剩余血量，百分制
    uint8_t color;     //当前装甲板颜色 //  0 undefine   1 blue   2 紫色   3 red
    uint8_t bullet;   //子弹剩余数量，仅能以unsigned 0-700映射到0-255
    uint8_t fault_flag;
} HeartBeatSend;
//


// len = 7
//电控发出: yaw1|yaw2|yaw3|yaw4|pitch1|pitch2|pitch3|pitch4
//视觉发出：yaw1|yaw2|yaw3|yaw4|pitch1|pitch2|pitch3|pitch4
typedef struct {
    float yaw, pitch;        //弧度制
    uint8_t fire; 
} Gimbal;

//,gimble_receive;
//电控回传经处理后的绝对yaw[total angle]，pitch角度。
//视觉传来的弧度角度信息：
//采用绝对角度控制，角度根据电控传回数据。

// len = 5
//视觉传来:velocity_top||shoot|
//            |float|  |char|
typedef struct {
    float velocity_top; // >0表小陀螺向右旋转 <0表示小陀螺向左 =0 表示关闭小陀螺
    uint8_t shoot;     //1表示射击，0表示不射击
} ControlData;


//电控回传到视觉：launch|arm|gun_status|current_gun|
typedef struct {
    uint8_t launch,        //回传底盘云台是否已经启用
    arm,           //回传摩擦轮是否已经启动
    base_hp_our,   //我方基地站血量,百分制
    base_hp_enemy, //对方基地在血量
    judge_warning_level;  //裁判系统警告
} AdditionalData;
//={0,0,100,100,0};

// len = 12
//vx1|vx2|vx3|vx4|vy1|vy2|vy3|vy4|vz1|vz2|vz3|vz4
typedef struct {
    float vx, vy, vz;
} ChassisSpeed;
//电控回传底盘各方向的运动数据
//视觉传来控制数据。


// len = 24
//电控发出：vx1|vx2|vx3|vx4|vy1|vy2|vy3|vy4|vz1|vz2|vz3|vz4|ax1|ax2|ax3|ax4|ay1|ay2|ay3|ay4|az1|az2|az3|az4
typedef struct {
    float vx, vy, vz, ax, ay, az;
} ImuData;
//imu2是云台imu数据。


//id = 7 len = 4
//电控发出：yaw1|yaw2|yaw3|yaw4
typedef struct {
    float yaw;//云台与底盘之间的相对角度delta，以底盘正前方为起始边，云台正前方为终边，右转为正，左转为负。可累积也可0-2pi
} YawData;

// ==================================== 数据定义【附曾,根据裁判系统串口定义】===============================

typedef struct {
    uint16_t stage_remain_time;
    //uint8_t game_type : 4;
    uint8_t game_progress;
} ext_game_status_t;
//={0,0};//比赛阶段数据

extern HeartBeatReceive heartbeat_send;
extern HeartBeatSend heartbeat_receive;
extern Gimbal gimbal_send;
extern Gimbal gimbal_receive;

extern ControlData control_data;
extern AdditionalData additional_data;
extern ChassisSpeed chassis_send;
extern ChassisSpeed chassis_receive;
extern YawData yaw_data;
extern ImuData imu2_data;
extern ext_game_status_t game_status_data;

extern int encode(int id,uint8_t* write_data);

extern bool decode(uint8_t *readdata);

#ifdef __cplusplus
}
#endif
#endif