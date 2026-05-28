#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
/**
 * @brief Tracker类构造函数 - 初始化目标跟踪器参数
 * @param solver Solver对象引用，用于装甲板3D
 * @param config_path 配置文件路径，包含跟踪器坐标解算
 * 
 * 初始化跟踪器状态、计数器和配置参数
 */
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},  // 引用Solver对象，用于3D坐标解算
  detect_count_(0), // 检测到目标的次数计数器
  temp_lost_count_(0), // 临时丢失计数器，用于判断目标是否丢失
  state_{"lost"}, // 当前状态，初始化为"lost"
  pre_state_{"lost"}, // 上一个状态，用于状态机判断
  last_timestamp_(std::chrono::steady_clock::now()),  // 上次检测时间，用于计算时间间隔
  omni_target_priority_{ArmorPriority::fifth} // 前哨站目标优先级，默认第五高 
{
  auto yaml = YAML::LoadFile(config_path);
  // 读取敌方颜色（红/蓝）
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
}

/**
 * @brief 获取当前跟踪状态
 * @return 当前状态字符串（"lost"、"detecting"、"tracking"等）
 */
  // state函数
  
std::string Tracker::state() const { return state_; }

/**
 * @brief 目标跟踪主函数 - 单相机版本
 * @param armors 检测到的装甲板列表
 * @param t 时间戳
 * @param use_enemy_color 是否按敌方颜色过滤
 * @return 跟踪到的目标列表（含单个Target对象）
 * 
 * 功能：
 * 1. 计算时间间隔，检测相机离线
 * 2. 过滤非敌方装甲板
 * 3. 装甲板排序（先按图像中心距离，再按优先级）
 * 4. 根据当前状态执行目标初始化或更新
 * 5. 更新状态机
 * 6. 检测目标发散和收敛效果
 */
std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非敌方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO: 应该从配置文件读取图像尺寸
    auto distance_1 = cv::norm(a.center - img_center);// 计算装甲板a到图像中心的距离
    auto distance_2 = cv::norm(b.center - img_center);// 计算装甲板b到图像中心的距离
    return distance_1 < distance_2;// 按距离排序，靠近图像中心的优先
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测，调用Target类的diverged()函数
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：如果最近的NIS失败次数超过窗口大小的40%，认为收敛效果差
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

/**
 * @brief 目标跟踪主函数 - 多相机版本（支持全向感知）
 * @param detection_queue 全向感知相机的检测结果队列
 * @param armors 主相机检测到的装甲板列表
 * @param t 时间戳
 * @param use_enemy_color 是否按敌方颜色过滤
 * @return 包含切换目标和跟踪目标列表的元组
 * 
 * 功能：
 * 1. 支持主相机和全向感知相机的目标跟踪
 * 2. 实现基于优先级的目标切换逻辑
 * 3. 处理全向感知相机发现更高优先级目标的情况
 * 4. 维护跟踪状态机
 */
std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO: 应该从配置文件读取图像尺寸
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  // 处于切换状态时，检查主相机是否找到全向感知指定优先级的目标
  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  // 从切换状态进入检测状态时，设置新目标
  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  // 其他情况更新现有目标
  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

/**
 * @brief 跟踪状态机 - 管理目标跟踪的各个状态转换
 * @param found 是否检测到目标
 * 
 * 状态转换逻辑：
 * - lost → detecting (首次检测到目标)
 * - detecting → tracking (连续检测次数达标)
 * - tracking → temp_lost (目标丢失)
 * - temp_lost → lost (丢失次数超标) / tracking (重新检测到目标)
 * - switching → detecting (找到切换目标)
 * - switching → lost (切换超时)
 */
void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些，因为其目标特征可能不明显
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

/**
 * @brief 设置新的跟踪目标
 * @param armors 检测到的装甲板列表
 * @param t 时间戳
 * @return 是否成功设置目标
 * 
 * 功能：
 * 1. 选择优先级最高的装甲板作为目标
 * 2. 调用Solver解算装甲板的3D坐标
 * 3. 根据不同装甲板类型（平衡步兵、前哨站、基地等）初始化目标跟踪参数
 * 4. 创建并初始化Target对象
 */
bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();  // 选择优先级最高的装甲板
  solver_.solve(armor);  // 解算装甲板的3D坐标

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    // 平衡步兵目标参数配置
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    // 前哨站目标参数配置
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    // 基地目标参数配置
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    // 其他目标默认参数配置
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

/**
 * @brief 更新现有的跟踪目标
 * @param armors 检测到的装甲板列表
 * @param t 时间戳
 * @return 是否成功更新目标
 * 
 * 功能：
 * 1. 预测目标当前状态（基于EKF）
 * 2. 查找与当前目标匹配的装甲板
 * 3. 解算匹配装甲板的3D坐标
 * 4. 使用新的观测数据更新目标状态（EKF更新）
 */
bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);  // 使用EKF预测目标当前状态

  int found_count = 0;
  double min_x = 1e10;  // 画面最左侧
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }

  if (found_count == 0) return false;  // 没有找到匹配的装甲板

  for (auto & armor : armors) {
    if (
      armor.name != target_.name || armor.type != target_.armor_type
      //  || armor.center.x != min_x
    )
      continue;

    solver_.solve(armor);  // 解算匹配装甲板的3D坐标

    target_.update(armor);  // 使用观测数据更新EKF状态
  }

  return true;
}

}  // namespace auto_aim