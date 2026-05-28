#include "planner.hpp"

#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
/**
 * @brief Planner类 - 实现基于模型预测控制(MPC)的目标轨迹规划器
 * 
 * 该类负责：
 * 1. 基于目标的预测位置和子弹飞行时间，计算理想的瞄准角度
 * 2. 使用MPC算法生成平滑的yaw和pitch轨迹
 * 3. 考虑子弹飞行时间和目标运动，预测目标未来位置
 * 4. 生成最终的云台控制指令，包括角度、角速度和角加速度
 */
/**
 * @brief Planner类构造函数 - 初始化规划器参数和MPC求解器
 * @param config_path 配置文件路径
 * 
 * 从配置文件加载各种参数，并初始化yaw和pitch轴的MPC求解器
 */
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;   // yaw轴偏移量（转换为弧度）
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3; // pitch轴偏移量（转换为弧度）
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");         // 开火阈值
  decision_speed_ = tools::read<double>(yaml, "decision_speed");    // 速度决策阈值
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time"); // 高速目标延迟时间
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");   // 低速目标延迟时间

  setup_yaw_solver(config_path);   // 初始化yaw轴MPC求解器
  setup_pitch_solver(config_path); // 初始化pitch轴MPC求解器
}

/**
 * @brief 规划目标跟踪轨迹并生成控制指令
 * @param target 目标对象，包含当前状态和预测信息
 * @param bullet_speed 子弹速度(m/s)
 * @return 规划结果，包含控制指令和开火决策
 * 
 * 该函数执行以下步骤：
 * 1. 子弹速度有效性检查和限制
 * 2. 计算子弹飞行时间并预测目标未来位置
 * 3. 生成目标跟踪的参考轨迹
 * 4. 使用MPC求解yaw轴轨迹
 * 5. 使用MPC求解pitch轴轨迹
 * 6. 生成最终控制指令和开火决策
 */
Plan Planner::plan(Target target, double bullet_speed)
{
  // 0. 子弹速度有效性检查，限制在合理范围内
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;  // 默认子弹速度22m/s
  }

  // 1. 计算子弹飞行时间并预测目标未来位置
  Eigen::Vector3d xyz;  // 目标位置
  auto min_dist = 1e10; // 最小距离初始值
  // 选择距离最近的装甲板作为瞄准目标
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();  // 计算水平距离
    if (dist < min_dist) {
      min_dist = dist;  // 更新最小距离
      xyz = xyza.head<3>();  // 保存目标位置
    }
  }
  // 创建子弹轨迹对象，计算飞行时间
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  // 预测子弹飞行时间后的目标位置
  target.predict(bullet_traj.fly_time);

  // 2. 生成目标跟踪的参考轨迹
  double yaw0;  // 初始yaw角度
  Trajectory traj;  // 轨迹对象
  try {
    // 计算初始瞄准角度
    yaw0 = aim(target, bullet_speed)(0);
    // 生成完整的参考轨迹
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    // 无法生成轨迹时记录警告
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};  // 返回失败的规划结果
  }

  // 3. 使用MPC求解yaw轴轨迹
  Eigen::VectorXd x0(2);  // 初始状态向量 [yaw, yaw_vel]
  x0 << traj(0, 0), traj(1, 0);  // 设置初始状态
  tiny_set_x0(yaw_solver_, x0);  // 配置MPC求解器初始状态

  // 设置MPC参考轨迹
  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);  // 求解yaw轴MPC问题

  // 4. 使用MPC求解pitch轴轨迹
  x0 << traj(2, 0), traj(3, 0);  // 设置初始状态 [pitch, pitch_vel]
  tiny_set_x0(pitch_solver_, x0);  // 配置MPC求解器初始状态

  // 设置MPC参考轨迹
  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);  // 求解pitch轴MPC问题

  // 5. 生成规划结果
  Plan plan;
  plan.control = true;  // 启用控制

  // 设置目标角度
  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  // 设置当前控制指令
  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  // 计算开火决策：检查预测轨迹与MPC轨迹的偏差是否在阈值内
  auto shoot_offset_ = 2;  // 开火时间偏移量
  plan.fire = 
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) - 
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

/**
 * @brief 带可选目标的规划函数重载版本
 * @param target 可选的目标对象，包含当前状态和预测信息
 * @param bullet_speed 子弹速度(m/s)
 * @return 规划结果，包含控制指令和开火决策
 * 
 * 该函数是plan函数的重载版本，处理目标可能不存在的情况：
 * 1. 检查目标是否存在
 * 2. 根据目标角速度选择不同的延迟时间
 * 3. 预测未来时刻的目标位置
 * 4. 调用基本版本的plan函数进行规划
 */
Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  // 检查目标是否存在
  if (!target.has_value()) return {false};

  // 根据目标角速度选择不同的延迟时间
  double delay_time = 
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  // 计算未来预测时刻
  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  // 预测未来时刻的目标位置
  target->predict(future);

  // 调用基本版本的plan函数进行规划
  return plan(*target, bullet_speed);
}

/**
 * @brief 初始化yaw轴MPC求解器
 * @param config_path 配置文件路径
 * 
 * 该函数初始化yaw轴的MPC求解器，设置系统模型、约束和权重：
 * 1. 从配置文件加载yaw轴相关参数
 * 2. 定义状态空间模型矩阵A、B、f
 * 3. 设置代价函数权重矩阵Q和R
 * 4. 初始化MPC求解器
 * 5. 设置状态和控制约束
 * 6. 配置求解器参数
 */
void Planner::setup_yaw_solver(const std::string & config_path)
{
  // 加载配置文件
  auto yaml = tools::load(config_path);
  // 读取yaw轴最大角加速度
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  // 读取状态权重Q_yaw
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  // 读取控制权重R_yaw
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  // 定义状态空间模型矩阵
  Eigen::MatrixXd A{{1, DT}, {0, 1}};  // 状态转移矩阵
  Eigen::MatrixXd B{{0}, {DT}};  // 控制输入矩阵
  Eigen::VectorXd f{{0, 0}};  // 偏置向量
  // 转换为Eigen矩阵格式
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  // 初始化MPC求解器
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  // 设置状态和控制约束
  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);  // 状态下限
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);   // 状态上限
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);  // 控制输入下限
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);   // 控制输入上限
  // 应用约束到求解器
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  // 设置求解器最大迭代次数
  yaw_solver_->settings->max_iter = 10;
}

/**
 * @brief 初始化pitch轴MPC求解器
 * @param config_path 配置文件路径
 * 
 * 该函数初始化pitch轴的MPC求解器，设置系统模型、约束和权重：
 * 1. 从配置文件加载pitch轴相关参数
 * 2. 定义状态空间模型矩阵A、B、f
 * 3. 设置代价函数权重矩阵Q和R
 * 4. 初始化MPC求解器
 * 5. 设置状态和控制约束
 * 6. 配置求解器参数
 */
void Planner::setup_pitch_solver(const std::string & config_path)
{
  // 加载配置文件
  auto yaml = tools::load(config_path);
  // 读取pitch轴最大角加速度
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  // 读取状态权重Q_pitch
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  // 读取控制权重R_pitch
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  // 定义状态空间模型矩阵
  Eigen::MatrixXd A{{1, DT}, {0, 1}};  // 状态转移矩阵
  Eigen::MatrixXd B{{0}, {DT}};  // 控制输入矩阵
  Eigen::VectorXd f{{0, 0}};  // 偏置向量
  // 转换为Eigen矩阵格式
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  // 初始化MPC求解器
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  // 设置状态和控制约束
  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);  // 状态下限
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);   // 状态上限
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);  // 控制输入下限
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);   // 控制输入上限
  // 应用约束到求解器
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  // 设置求解器最大迭代次数
  pitch_solver_->settings->max_iter = 10;
}

/**
 * @brief 计算目标的瞄准角度
 * @param target 目标对象，包含当前状态和预测信息
 * @param bullet_speed 子弹速度(m/s)
 * @return 瞄准角度向量 [yaw, pitch]
 * 
 * 该函数计算目标的理想瞄准角度：
 * 1. 选择距离最近的装甲板作为瞄准目标
 * 2. 计算水平方位角(azim)
 * 3. 创建子弹轨迹对象，计算所需的俯仰角
 * 4. 应用角度偏移并限制到合理范围
 */
Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;  // 目标位置
  double yaw;  // 目标yaw角度
  auto min_dist = 1e10;  // 最小距离初始值

  // 选择距离最近的装甲板作为瞄准目标
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();  // 计算水平距离
    if (dist < min_dist) {
      min_dist = dist;  // 更新最小距离
      xyz = xyza.head<3>();  // 保存目标位置
      yaw = xyza[3];  // 保存目标yaw角度
    }
  }
  // 保存调试信息
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  // 计算水平方位角
  auto azim = std::atan2(xyz.y(), xyz.x());
  // 创建子弹轨迹对象，计算所需的俯仰角
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  // 检查子弹轨迹是否可解
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  // 返回带偏移的角度，并限制到合理范围
  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

/**
 * @brief 生成目标跟踪的参考轨迹
 * @param target 目标对象，包含当前状态和预测信息
 * @param yaw0 初始yaw角度
 * @param bullet_speed 子弹速度(m/s)
 * @return 生成的轨迹对象，包含yaw、pitch及其速度信息
 * 
 * 该函数生成目标跟踪的参考轨迹：
 * 1. 预测历史时刻的目标位置
 * 2. 计算历史、当前和未来时刻的瞄准角度
 * 3. 使用中心差分法计算角速度
 * 4. 生成完整的轨迹数据，包含位置和速度信息
 */
Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;  // 轨迹对象

  // 预测历史时刻的目标位置
  target.predict(-DT * (HALF_HORIZON + 1));
  // 计算历史时刻的瞄准角度
  auto yaw_pitch_last = aim(target, bullet_speed);

  // 预测当前时刻的目标位置
  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  // 计算当前时刻的瞄准角度
  auto yaw_pitch = aim(target, bullet_speed);

  // 生成未来HORIZON个时刻的轨迹
  for (int i = 0; i < HORIZON; i++) {
    // 预测下一个时刻的目标位置
    target.predict(DT);
    // 计算下一个时刻的瞄准角度
    auto yaw_pitch_next = aim(target, bullet_speed);

    // 使用中心差分法计算角速度
    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    // 保存轨迹点：[相对yaw, yaw速度, pitch, pitch速度]
    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    // 更新历史数据
    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

}  // namespace auto_aim