/**
 * @file auto_aim_debug_mpc.cpp
 * @brief 自动瞄准MPC轨迹规划调试程序
 * 
 * 功能：
 * 1. 实现自动瞄准系统的完整流程：图像采集、目标检测、跟踪和MPC轨迹规划
 * 2. 实时可视化目标跟踪和MPC规划结果
 * 3. 记录和绘制关键数据用于调试和分析
 * 4. 支持通过命令行参数配置
 * 
 * 主要模块：
 * - 相机图像采集
 * - YOLO目标检测
 * - 目标跟踪（Tracker）
 * - MPC轨迹规划（Planner）
 * - 云台控制（Gimbal）
 * - 数据可视化和记录
 */

#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

// 系统模块头文件
#include "io/camera.hpp"         // 相机图像采集模块
#include "io/gimbal/gimbal.hpp"  // 云台控制模块
#include "tasks/auto_aim/planner/planner.hpp"  // MPC轨迹规划模块
#include "tasks/auto_aim/solver.hpp"  // PnP解算模块
#include "tasks/auto_aim/tracker.hpp"  // 目标跟踪模块
#include "tasks/auto_aim/yolo.hpp"     // YOLO目标检测模块

// 工具类头文件
#include "tools/exiter.hpp"          // 程序退出管理
#include "tools/img_tools.hpp"       // 图像处理工具
#include "tools/logger.hpp"          // 日志工具
#include "tools/math_tools.hpp"      // 数学工具
#include "tools/plotter.hpp"         // 数据绘图工具
#include "tools/thread_safe_queue.hpp"  // 线程安全队列

using namespace std::chrono_literals;  // 时间单位字面量

/**
 * @brief 命令行参数定义
 * 
 * 定义程序支持的命令行参数：
 * - help: 输出帮助信息
 * - config-path: YAML配置文件路径（默认：configs/sentry.yaml）
 */
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

/**
 * @brief 程序主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char * argv[])
{
  tools::Exiter exiter;  // 程序退出管理器，处理优雅退出
  tools::Plotter plotter;  // 数据绘图工具，用于可视化调试数据

  // 解析命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);  // 获取配置文件路径
  if (cli.has("help") || config_path.empty()) {  // 如果用户请求帮助或配置文件路径为空
    cli.printMessage();  // 输出帮助信息
    return 0;  // 退出程序
  }

  // 初始化各个系统模块
  io::Gimbal gimbal(config_path);     // 云台控制模块，负责与下位机通信和控制云台运动
  io::Camera camera(config_path);     // 相机图像采集模块，负责从相机获取图像数据

  auto_aim::YOLO yolo(config_path, true);  // YOLO目标检测模块，第二个参数true表示启用调试模式
  auto_aim::Solver solver(config_path);     // PnP解算模块，负责将图像坐标转换为世界坐标
  auto_aim::Tracker tracker(config_path, solver);  // 目标跟踪模块，管理目标的跟踪状态和优先级
  auto_aim::Planner planner(config_path);  // MPC轨迹规划模块，根据目标状态生成最优控制指令

  // 创建线程安全队列，用于在主线程和规划线程之间传递目标信息
  // 模板参数说明：
  // - std::optional<auto_aim::Target>: 队列元素类型，可以为空（表示无目标）
  // - true: 表示队列是覆盖型的，当队列满时新元素会覆盖旧元素
  // 构造函数参数1: 队列最大容量为1
  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);  // 初始化为空目标

  std::atomic<bool> quit = false;  // 线程退出标志，用于通知规划线程退出循环

  /**
   * @brief MPC规划线程
   * 
   * 功能：
   * 1. 从目标队列获取当前跟踪目标
   * 2. 使用MPC算法生成云台控制指令
   * 3. 向云台发送控制指令
   * 4. 记录和绘制调试数据
   * 5. 控制线程执行频率（10ms周期）
   */
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();  // 程序启动时间，用于计算相对时间
    uint16_t last_bullet_count = 0;  // 上一时刻的子弹数量，用于检测是否开火

    while (!quit) {
      auto target = target_queue.front();  // 获取当前跟踪目标
      auto gs = gimbal.state();  // 获取当前云台状态
      auto plan = planner.plan(target, gs.bullet_speed);  // 使用MPC生成规划结果

      // 向云台发送控制指令
      // 参数说明：
      // - plan.control: 是否启用控制（true/false）
      // - plan.fire: 是否开火（true/false）
      // - plan.yaw/pitch: 目标角度位置
      // - plan.yaw_vel/pitch_vel: 目标角速度
      // - plan.yaw_acc/pitch_acc: 目标角加速度
      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      // 检测是否开火（通过比较当前和上一时刻的子弹数量）
      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      // 准备绘制数据
      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);  // 相对时间（秒）

      // 记录云台状态数据
      data["gimbal_yaw"] = gs.yaw;           // 当前云台yaw角度（弧度）
      data["gimbal_yaw_vel"] = gs.yaw_vel;   // 当前云台yaw角速度（弧度/秒）
      data["gimbal_pitch"] = gs.pitch;       // 当前云台pitch角度（弧度）
      data["gimbal_pitch_vel"] = gs.pitch_vel; // 当前云台pitch角速度（弧度/秒）

      // 记录目标角度数据
      data["target_yaw"] = plan.target_yaw;   // 目标yaw角度（弧度）
      data["target_pitch"] = plan.target_pitch; // 目标pitch角度（弧度）

      // 记录MPC规划的Yaw数据
      data["plan_yaw"] = plan.yaw;           // MPC规划的yaw目标位置（弧度）
      data["plan_yaw_vel"] = plan.yaw_vel;   // MPC规划的yaw目标角速度（弧度/秒）
      data["plan_yaw_acc"] = plan.yaw_acc;   // MPC规划的yaw目标角加速度（弧度/秒²）

      // 记录MPC规划的Pitch数据
      data["plan_pitch"] = plan.pitch;       // MPC规划的pitch目标位置（弧度）
      data["plan_pitch_vel"] = plan.pitch_vel; // MPC规划的pitch目标角速度（弧度/秒）
      data["plan_pitch_acc"] = plan.pitch_acc; // MPC规划的pitch目标角加速度（弧度/秒²）

      // 记录开火状态数据
      data["fire"] = plan.fire ? 1 : 0;      // MPC是否发出开火指令（0/1）
      data["fired"] = fired ? 1 : 0;         // 实际是否开火（0/1）

      // 记录目标位置和速度数据（如果有目标）
      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   // 目标z坐标（米）
        data["target_vz"] = target->ekf_x()[5];  // 目标z方向速度（米/秒）
      }

      // 记录目标角速度数据（如果有目标）
      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];  // 目标角速度（弧度/秒）
      } else {
        data["w"] = 0.0;  // 无目标时角速度为0
      }

      plotter.plot(data);  // 将数据发送到绘图工具进行可视化

      std::this_thread::sleep_for(10ms);  // 控制线程周期为10ms
    }
  });

  cv::Mat img;  // 图像矩阵，用于存储相机采集的图像
  std::chrono::steady_clock::time_point t;  // 时间戳，用于记录图像采集时间

  /**
   * @brief 主循环
   * 
   * 功能：
   * 1. 从相机读取图像
   * 2. 获取云台姿态并更新坐标系转换
   * 3. 检测装甲板
   * 4. 跟踪目标
   * 5. 可视化跟踪结果和MPC瞄准点
   * 6. 处理用户输入
   */
  while (!exiter.exit()) {  // 当用户未请求退出时循环执行
    camera.read(img, t);  // 从相机读取图像和对应的时间戳
    auto q = gimbal.q(t);  // 根据时间戳获取对应的云台姿态（四元数表示）

    // 更新PnP解算器中的云台到世界坐标系的旋转矩阵
    solver.set_R_gimbal2world(q);  // 使用四元数更新旋转矩阵
    auto armors = yolo.detect(img);  // 使用YOLO算法检测当前图像中的装甲板
    auto targets = tracker.track(armors, t);  // 基于检测到的装甲板进行目标跟踪
    
    // 将跟踪到的目标发送到MPC规划线程
    // 如果有跟踪目标，发送第一个（优先级最高的）目标
    // 否则发送空目标
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    // 如果有跟踪目标，可视化跟踪结果
    if (!targets.empty()) {
      auto target = targets.front();  // 获取优先级最高的跟踪目标

      // 当前帧target更新后，绘制所有装甲板的投影
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      // 遍历目标的所有装甲板（xyza表示装甲板的位置和姿态）
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        // 将装甲板从世界坐标系重新投影到图像坐标系
        // 参数说明：
        // - xyza.head(3): 装甲板在世界坐标系中的位置（x, y, z）
        // - xyza[3]: 装甲板的旋转角度（yaw）
        // - target.armor_type: 装甲板类型（小装甲/大装甲）
        // - target.name: 目标名称
        auto image_points = 
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});  // 绿色绘制装甲板轮廓
      }

      // 绘制MPC计算的瞄准点
      Eigen::Vector4d aim_xyza = planner.debug_xyza;  // 获取MPC规划的瞄准点坐标
      // 将瞄准点从世界坐标系重新投影到图像坐标系
      auto image_points = 
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});  // 红色绘制瞄准点
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸（0.5倍缩放，方便查看）
    cv::imshow("reprojection", img);  // 显示带有跟踪结果的图像窗口
    auto key = cv::waitKey(1);  // 等待1毫秒，处理窗口事件
    if (key == 'q') break;  // 按下'q'键退出程序
  }

  // 程序退出处理
  quit = true;  // 设置线程退出标志，通知规划线程退出循环
  if (plan_thread.joinable()) plan_thread.join();  // 等待规划线程安全结束
  // 发送停止控制指令：禁用控制，不开火，所有角度和速度设为0
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);  // 停止云台控制

  return 0;  // 程序正常退出
}