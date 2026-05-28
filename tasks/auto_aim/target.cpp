#include "target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
/**
 * @brief Target构造函数，用于初始化一个新的跟踪目标
 * 
 * @param armor 检测到的装甲板对象
 * @param t 当前时间戳
 * @param radius 初始旋转半径（目标中心到装甲板的距离）
 * @param armor_num 装甲板数量
 * @param P0_dig 初始协方差对角线元素
 */
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig) 
: name(armor.name),             // 目标名称（从装甲板获取）
  armor_type(armor.type),       // 装甲板类型（大/小）
  jumped(false),                // 初始化：未发生装甲板切换
  last_id(0),                   // 上一次跟踪的装甲板ID
  update_count_(0),             // 更新次数计数器
  armor_num_(armor_num),        // 装甲板数量
  t_(t),                        // 当前时间戳
  is_switch_(false),            // 初始化：未在切换装甲板
  is_converged_(false),         // 初始化：EKF未收敛
  switch_count_(0)              // 切换次数计数器
{
  auto r = radius;              // 旋转半径的局部变量
  priority = armor.priority;     // 目标优先级（从装甲板获取）
  const Eigen::VectorXd & xyz = armor.xyz_in_world;  // 装甲板在世界坐标系的位置
  const Eigen::VectorXd & ypr = armor.ypr_in_world;  // 装甲板在世界坐标系的姿态

  // 计算目标旋转中心的坐标
  // 装甲板位置 + 旋转半径 * 方向向量（基于装甲板yaw角）
  auto center_x = xyz[0] + r * std::cos(ypr[0]);  // 旋转中心x坐标
  auto center_y = xyz[1] + r * std::sin(ypr[0]);  // 旋转中心y坐标
  auto center_z = xyz[2];                         // 旋转中心z坐标（与装甲板相同）

  // 初始化EKF状态向量：11维 [x, vx, y, vy, z, vz, a, w, r, l, h]
  // x, y, z: 旋转中心坐标
  // vx, vy, vz: 旋转中心速度
  // a: 旋转角度
  // w: 旋转角速度
  // r: 旋转半径
  // l: 长半轴与短半轴差值
  // h: 高度差值
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};
  // 初始化协方差矩阵
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 定义状态向量加法函数，确保角度值保持在合理范围内
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);  // 限制角度在[-π, π]范围内
    return c;
  };

  // 初始化扩展卡尔曼滤波器
  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

/**
 * @brief Target类构造函数 - 特殊构造函数，主要用于前哨站目标
 * @param x x轴位置
 * @param vyaw 旋转角速度
 * @param radius 旋转半径
 * @param h 高度
 */
Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  // 初始化EKF状态向量
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  // 初始化协方差矩阵（全部为0）
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 定义状态向量加法函数
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);  // 限制角度在合理范围
    return c;
  };

  // 初始化扩展卡尔曼滤波器
  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

/**
 * @brief 基于时间戳的状态预测
 * @param t 当前时间戳
 */
void Target::predict(std::chrono::steady_clock::time_point t)
{
  // 计算时间间隔
  auto dt = tools::delta_time(t, t_);
  // 调用基于时间间隔的预测函数
  predict(dt);
  // 更新时间戳
  t_ = t;
}

/**
 * @brief 基于时间间隔的EKF状态预测
 * @param dt 预测时间间隔（秒）
 */
void Target::predict(double dt)
{
  // 状态转移矩阵F：11×11维，描述状态随时间的变化关系
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},  // x = x + vx*dt
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},  // vx = vx
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},  // y = y + vy*dt
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},  // vy = vy
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},  // z = z + vz*dt
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},  // vz = vz
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},  // a = a + w*dt
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},  // w = w
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},  // r = r
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},  // l = l
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}   // h = h
  };
  // clang-format on

  // 分段白噪声模型 - 用于计算过程噪声协方差Q
  // 参考：https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;  // 加速度和角加速度的方差
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差（较小，运动更规律）
    v2 = 0.1;  // 前哨站角加速度方差（较小，旋转更稳定）
  } else {
    v1 = 100;  // 普通目标加速度方差
    v2 = 400;  // 普通目标角加速度方差
  }
  
  // 计算噪声矩阵系数
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  
  // 过程噪声协方差矩阵Q：11×11维
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},  // x方向噪声
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},  // vx方向噪声
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},  // y方向噪声
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},  // vy方向噪声
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},  // z方向噪声
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},  // vz方向噪声
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},  // 角度噪声
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},  // 角速度噪声
    // {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},  // r方向噪声（半径可缓慢变化）
    // {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},  // l方向噪声（长半轴差值）
    // {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}   // h方向噪声（高度差值）
    {     0,      0,      0,      0,      0,      0,      0,      0, a * v1, b * v1, 0},  // r方向噪声（半径可缓慢变化）
    {     0,      0,      0,      0,      0,      0,      0,      0, b * v1, c * v1, 0},  // l方向噪声（长半轴差值）
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, c * v1}   // h方向噪声（高度差值）
  };
  // clang-format on

  // 定义状态转移函数，确保角度值保持在合理范围内
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);  // 限制角度在[-π, π]范围内
    return x_prior;
  };

  // 前哨站转速特殊处理：限制最大角速度
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;  // 限制在±2.51 rad/s

  // 执行EKF预测步骤
  ekf_.predict(F, Q, f);
}

/**
 * @brief 更新目标状态 - 基于新检测到的装甲板
 * @param armor 新检测到的装甲板对象
 */
void Target::update(const Armor & armor)
{
  // 装甲板匹配：找到与当前预测最匹配的装甲板ID
  int id;
  auto min_angle_error = 1e10;  // 初始化最小角度误差为很大的值
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();  // 获取预测的所有装甲板位置

  // 将装甲板按距离排序
  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  // 按距离从小到大排序
  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));  // 转换为yaw-pitch-distance
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];  // 按距离排序
    });

  // 从距离最近的3个装甲板中选择角度误差最小的
  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));  // 预测的yaw-pitch-distance
    
    // 计算角度误差：yaw角误差 + 装甲板朝向角误差
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    // 更新最小角度误差和对应的装甲板ID
    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  // 记录装甲板是否发生跳转
  if (id != 0) jumped = true;

  // 检测装甲板是否切换
  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  // 更新切换计数
  if (is_switch_) switch_count_++;

  // 更新状态
  last_id = id;
  update_count_++;

  // 执行EKF更新
  update_ypda(armor, id);
}

/**
 * @brief 基于Yaw-Pitch-Distance-Angle的EKF状态更新
 * @param armor 新检测到的装甲板对象
 * @param id 匹配到的装甲板ID
 */
void Target::update_ypda(const Armor & armor, int id)
{
  // 计算观测模型的雅可比矩阵
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  
  // 计算目标中心的yaw角
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  // 计算装甲板朝向与目标中心yaw角的差值
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  
  // 观测噪声协方差矩阵对角线元素
  // 根据距离和角度差动态调整噪声
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

  // 构造观测噪声协方差矩阵R
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义观测模型函数h: 状态向量 -> 观测向量
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);  // 计算装甲板在世界坐标系中的位置
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);  // 转换为yaw-pitch-distance
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);  // 计算装甲板朝向角
    return {ypd[0], ypd[1], ypd[2], angle};  // 返回观测向量
  };

  // 定义观测向量减法函数，确保角度差在合理范围内
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);  // yaw角差
    c[1] = tools::limit_rad(c[1]);  // pitch角差
    c[3] = tools::limit_rad(c[3]);  // 朝向角差
    return c;
  };

  // 获取实际观测值
  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  // 观测向量：[yaw, pitch, distance, angle]

  // 执行EKF更新步骤
  ekf_.update(z, H, R, h, z_subtract);
}

/**
 * @brief 获取EKF的状态向量
 * @return 当前状态向量
 */
Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

/**
 * @brief 获取EKF滤波器实例
 * @return EKF滤波器引用
 */
const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

/**
 * @brief 获取所有装甲板的预测位置和朝向角
 * @return 装甲板列表，每个元素为包含(x, y, z, 朝向角)的向量
 */
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  // 遍历所有装甲板，计算每个装甲板的预测位置和朝向角
  for (int i = 0; i < armor_num_; i++) {
    // 计算第i个装甲板的朝向角：基础角度加上该装甲板的相对角度
    // 使用limit_rad确保角度在合理范围内
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    
    // 计算第i个装甲板的三维位置
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    
    // 将装甲板的位置和朝向角添加到列表中
    // 向量格式：[x, y, z, 朝向角]
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  
  return _armor_xyza_list;
}

/**
 * @brief 判断EKF是否发散
 * @return true如果发散，false如果正常
 */
bool Target::diverged() const
{
  // 检查旋转半径r是否在合理范围内
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  // 检查长半轴l是否在合理范围内
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;  // 正常

  // 记录发散信息
  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;  // 发散
}

/**
 * @brief 判断EKF是否收敛
 * @return true如果收敛，false如果未收敛
 */
bool Target::convergened()
{
  // 普通目标收敛条件：更新次数>3且未发散
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  // 前哨站收敛条件：更新次数>10且未发散
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

/**
 * @brief 计算装甲板中心在世界坐标系中的坐标（考虑长短轴差异）
 * @param x 状态向量
 * @param id 装甲板ID
 * @return 装甲板中心坐标 [x, y, z]
 */
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  // 计算装甲板朝向角
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  // 判断是否使用长半轴和高度偏移（适用于4个装甲板的情况，ID为1和3的装甲板）
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  // 计算旋转半径：普通装甲板使用r，长半轴装甲板使用r+l
  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  // 计算装甲板中心坐标
  auto armor_x = x[0] - r * std::cos(angle);  // x坐标
  auto armor_y = x[2] - r * std::sin(angle);  // y坐标
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];  // z坐标（长半轴装甲板有高度偏移）

  return {armor_x, armor_y, armor_z};
}

/**
 * @brief 计算观测模型的雅可比矩阵
 * @param x 状态向量
 * @param id 装甲板ID
 * @return 雅可比矩阵H
 */
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  // 计算装甲板朝向角
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  // 判断是否使用长半轴和高度偏移
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  // 计算旋转半径
  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  
  // 计算位置对角度的偏导数
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  // 计算位置对半径r和长半轴l的偏导数
  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  // 计算z坐标对高度h的偏导数
  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // 装甲板位置和角度对状态向量的雅可比矩阵H_armor_xyza
  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},  // x对状态的偏导数
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},  // y对状态的偏导数
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},  // z对状态的偏导数
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}   // 角度对状态的偏导数
  };
  // clang-format on

  // 计算装甲板位置
  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  // 计算xyz到ypd的雅可比矩阵
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  
  // 构造ypda到xyza的雅可比矩阵
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},  // yaw对xyz的偏导数
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},  // pitch对xyz的偏导数
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},  // distance对xyz的偏导数
    {                0,                 0,                 0, 1}   // angle对angle的偏导数
  };
  // clang-format on

  // 总的雅可比矩阵H = H_armor_ypda * H_armor_xyza
  return H_armor_ypda * H_armor_xyza;
}

/**
 * @brief 检查目标是否已初始化
 * @return true如果已初始化，false否则
 */
bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
