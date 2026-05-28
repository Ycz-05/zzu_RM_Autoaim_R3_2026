#include <fmt/format.h>
#include <chrono> // 需要包含 chrono 头文件

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

// 定义命令行参数
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 初始化绘图器、录制器、退出器
  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::Exiter exiter;

  // 初始化云台
  io::Gimbal gimbal(config_path);

  // ==================== 修改开始 ====================

  // 1. 【移除】不再需要 VisionToGimbal 结构体
  // io::VisionToGimbal plan; // <--- 移除这一行

  // 2. 【定义新的状态变量】我们需要直接管理控制和开火的状态
  bool control_gimbal = true; // 假设我们一直控制云台
  bool fire_command = false;  // 开火指令，默认为 false

  // 3. 【定义目标角度】我们需要定义要发送给云台的目标 yaw 和 pitch
  float target_yaw = 0.0f;
  float target_pitch = 0.0f;
  // 其他速度和加速度参数在新协议中被忽略，可以设为0
  float target_yaw_vel = 0.0f;
  float target_yaw_acc = 0.0f;
  float target_pitch_vel = 0.0f;
  float target_pitch_acc = 0.0f;

  auto last_fire_time = std::chrono::steady_clock::now(); // 变量名改为 last_fire_time 更清晰

  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    auto gs = gimbal.state(); // 获取云台当前状态 (虽然这个测试里没用上)

    // 4. 【更新开火逻辑】根据时间判断是否需要开火
    if (tools::delta_time(now, last_fire_time) > 1.600) {
        fire_command = true; // 设置开火指令为 true
        tools::logger()->debug("fire!");
        last_fire_time = now;
    } else {
        fire_command = false; // 在其他时间，开火指令为 false
    }

    // 5. 【调用新的 send API】使用新的状态变量调用 gimbal.send()
    gimbal.send(
        control_gimbal,   // 是否控制云台
        fire_command,     // 是否开火
        target_yaw,
        target_yaw_vel,
        target_yaw_acc,
        target_pitch,
        target_pitch_vel,
        target_pitch_acc
    );

    // ==================== 修改结束 ====================


    // -------------- 调试输出 (这部分逻辑也需要微调) --------------

    nlohmann::json data;

    // 【微调】现在我们直接用 fire_command 来判断
    if (control_gimbal) { // 或者直接判断 if(true)
      data["shoot"] = fire_command ? 1 : 0;
    }

    plotter.plot(data);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
