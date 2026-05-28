#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <string>

// 包含我们要测试的 gimbal 模块的头文件
#include "io/gimbal/gimbal.hpp"

int main(int argc, char **argv) {
    // ==========================================================
    // 1. 获取串口设备名
    // ==========================================================
    std::string device_name = "/dev/ttyUSB0"; // 默认设备名

    // 允许通过命令行参数覆盖默认设备名
    if (argc > 1) {
        device_name = argv[1];
        std::cout << "[INFO] Using device specified from command line: " << device_name << std::endl;
    } else {
        std::cout << "[INFO] Using default device: " << device_name << std::endl;
        std::cout << "[INFO] You can specify another device, e.g., ./gimbal_test /dev/ttyUSB1" << std::endl;
    }

    try {
        // ==========================================================
        // 2. 初始化 Gimbal 对象 (不使用配置文件)
        // ==========================================================
        std::cout << "[INFO] Initializing Gimbal..." << std::endl;
        // 调用我们之前准备好的、不依赖配置文件的构造函数
        io::Gimbal gimbal(device_name); 
        std::cout << "[SUCCESS] Gimbal initialized successfully." << std::endl;
        std::cout << "\n--- Starting Standalone Gimbal API Test Loop ---" << std::endl;

        double angle = 0.0;
        int loop_count = 0;

        while (true) {
            loop_count++;
            
            // ==========================================================
            // 3. 测试 send() 接口
            // ==========================================================
            
            // 模拟一个动态的目标角度
            angle += 0.05;
            float target_yaw = 10.0;
            //30.0f * static_cast<float>(cos(angle));
            float target_pitch = 25.0;
            //15.0f * static_cast<float>(sin(angle));
            
            // 模拟开火指令
            uint8_t fire_command = 1;

            gimbal.send(true, fire_command, target_yaw, 0, 0, target_pitch, 0, 0);

            // ==========================================================
            // 4. 测试 state(), q(), mode() 等获取接口
            // ==========================================================
            io::GimbalState current_state = gimbal.state();
            io::GimbalMode current_mode = gimbal.mode();
            Eigen::Quaterniond orientation_q = gimbal.q();

            // ==========================================================
            // 5. 打印所有获取到的信息
            // ==========================================================
            
            // 使用 \r 实现原地刷新，避免刷屏
            printf("\r--> Sent: Y=%6.1f,P=%6.1f | <-- Rcvd: Y=%6.1f,P=%6.1f,R=%6.1f | Mode: %s | Fire: %s", 
                   target_yaw, target_pitch,
                   current_state.yaw, current_state.pitch, current_state.roll,
                   gimbal.str(current_mode).c_str(),
                   fire_command ? "YES" : "NO ");
            fflush(stdout); // 强制刷新输出缓冲区

            // 控制循环频率
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

    } catch (const std::exception& e) {
        std::cerr << "\n[FATAL ERROR] An exception occurred: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
