#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
/**
 * @brief Aimer类构造函数，从配置文件加载瞄准参数
 * 
 * @param config_path 配置文件路径，包含瞄准相关的参数配置
 */
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  // 加载YAML配置文件
  auto yaml = YAML::LoadFile(config_path);
  
  // 加载基本瞄准参数（角度单位从度转换为弧度）
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // yaw轴偏移量（度转弧度）
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // pitch轴偏移量（度转弧度）
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // 进入角度阈值（度转弧度）
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // 离开角度阈值（度转弧度）
  
  // 加载时间延迟参数
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();  // 高速目标延迟时间
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();    // 低速目标延迟时间
  decision_speed_ = yaml["decision_speed"].as<double>();                // 速度决策阈值
  
  // 加载左右射击模式偏移量（如果配置文件中定义了）
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // 左射击模式yaw偏移（度转弧度）
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // 右射击模式yaw偏移（度转弧度）
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

/**
 * @brief 主瞄准函数，计算瞄准指令（不考虑射击模式）
 * 
 * @param targets 目标列表，通常选择第一个目标进行瞄准
 * @param timestamp 当前时间戳
 * @param bullet_speed 弹丸速度
 * @param to_now 是否使用当前时间（true）或固定延迟（false）
 * @return io::Command 瞄准指令，包含是否射击、yaw和pitch角度
 */
io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  // 检查目标列表是否为空
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  // 获取目标EKF状态并计算延迟时间（根据角速度选择高速或低速延迟）
  auto ekf = target.ekf();
  double delay_time =
    target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  // 确保弹丸速度不低于最小值
  if (bullet_speed < 14) bullet_speed = 23;

  // 计算未来时间点（考虑检测器、跟踪器和发弹延迟）
  auto future = timestamp;
  if (to_now) {
    // 使用当前时间计算延迟
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }
  else {
    // 使用固定延迟时间（检测器-瞄准器耗时0.005秒 + 发弹延迟）
    auto dt = 0.005 + delay_time;
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  // 选择初始瞄准点并检查有效性
  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  // 计算初始弹道（基于水平距离和高度）
  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);  // 水平距离
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间（最多10次迭代，收敛条件：相邻两次飞行时间差 < 0.001秒）
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算新的瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter]);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());  // 更新水平距离
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件：相邻两次飞行时间差小于0.001秒
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终瞄准角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;  // yaw角（考虑偏移）
  double pitch = -(current_traj.pitch + pitch_offset_);  // pitch角（世界坐标系下pitch向上为负）
  
  // 返回瞄准指令（允许射击，不开启摩擦轮，返回yaw和pitch角度）
  return {true, false, yaw, pitch};
}

/**
 * @brief 带射击模式的瞄准函数，根据射击模式调整yaw偏移量
 * 
 * @param targets 目标列表
 * @param timestamp 当前时间戳
 * @param bullet_speed 弹丸速度
 * @param shoot_mode 射击模式（左射击、右射击、默认）
 * @param to_now 是否使用当前时间
 * @return io::Command 调整后的瞄准指令
 */
io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  // 根据射击模式选择对应的yaw偏移量
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();  // 左射击模式偏移
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value(); // 右射击模式偏移
  } else {
    yaw_offset = yaw_offset_;               // 默认偏移
  }

  // 调用基础瞄准函数获取初始指令
  auto command = aim(targets, timestamp, bullet_speed, to_now);
  
  // 调整yaw角度：移除默认偏移，添加指定射击模式的偏移
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

/**
 * @brief 选择瞄准点，根据目标状态和运动模式选择最优的装甲板进行瞄准
 * 
 * @param target 目标对象，包含EKF状态和装甲板信息
 * @return AimPoint 选择的瞄准点，包含有效性和装甲板信息
 */
AimPoint Aimer::choose_aim_point(const Target & target)
{
  // 获取目标EKF状态向量和所有装甲板的位置信息
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  
  // 如果装甲板未发生过跳变（目标刚被检测到），则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 计算整车旋转中心在球坐标系中的yaw角
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 计算每个装甲板相对于旋转中心的delta角度（装甲板朝向角与中心yaw角的差值）
  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 非小陀螺情况处理（角速度绝对值 <= 2 rad/s 且不是前哨站）
  if (std::abs(target.ekf_x()[7]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板（delta角度绝对值小于60度）
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    
    // 如果没有装甲板在可射击范围内，记录警告并返回无效瞄准点
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  // 小陀螺或前哨站情况：设置进入和离开角度阈值
  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;  // 前哨站：进入角度70度
    leaving_angle = 30 / 57.3; // 前哨站：离开角度30度
  } else {
    coming_angle = comming_angle_;  // 普通目标：使用配置的进入角度
    leaving_angle = leaving_angle_; // 普通目标：使用配置的离开角度
  }

  // 小陀螺模式下：一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  // 选择策略：
  // 1. 装甲板必须在进入角度范围内（delta_angle绝对值 < coming_angle）
  // 2. 根据旋转方向选择离开角度范围内的装甲板
  for (int i = 0; i < armor_num; i++) {
    // 跳过不在进入角度范围内的装甲板
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    
    // 顺时针旋转（角速度 > 0）：选择delta_angle < leaving_angle的装甲板
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    
    // 逆时针旋转（角速度 < 0）：选择delta_angle > -leaving_angle的装甲板
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  // 如果没有找到合适的装甲板，返回无效瞄准点
  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim