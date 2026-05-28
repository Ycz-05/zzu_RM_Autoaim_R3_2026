/**
 * @file extended_kalman_filter.cpp
 * @brief 扩展卡尔曼滤波器实现
 * @details 该文件实现了扩展卡尔曼滤波器(Extended Kalman Filter, EKF)的核心算法，
 *          包括预测步骤和更新步骤，以及卡方检验用于滤波器性能评估。
 */

#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
/**
 * @brief 构造函数
 * @param x0 初始状态向量
 * @param P0 初始状态协方差矩阵
 * @param x_add 状态向量加法函数，用于处理非线性状态更新
 * @details 初始化扩展卡尔曼滤波器，设置初始状态、协方差矩阵和状态加法函数，
 *          同时初始化用于调试和性能评估的数据结构
 */
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  // 初始化卡方检验相关数据
  data["residual_yaw"] = 0.0;      // 偏航角残差
  data["residual_pitch"] = 0.0;    // 俯仰角残差
  data["residual_distance"] = 0.0; // 距离残差
  data["residual_angle"] = 0.0;    // 角度残差
  data["nis"] = 0.0;               // 归一化新息平方(Normalized Innovation Squared)
  data["nees"] = 0.0;              // 归一化估计误差平方(Normalized Estimation Error Squared)
  data["nis_fail"] = 0.0;          // NIS检验是否失败
  data["nees_fail"] = 0.0;         // NEES检验是否失败
  data["recent_nis_failures"] = 0.0; // 近期NIS失败率
}

/**
 * @brief 线性预测步骤
 * @param F 状态转移矩阵
 * @param Q 过程噪声协方差矩阵
 * @return 预测后的状态向量
 * @details 使用线性状态转移矩阵F进行状态预测，适用于线性系统
 */
Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

/**
 * @brief 非线性预测步骤
 * @param F 状态转移矩阵（用于协方差更新）
 * @param Q 过程噪声协方差矩阵
 * @param f 非线性状态转移函数
 * @return 预测后的状态向量
 * @details 使用非线性状态转移函数f进行状态预测，同时使用F进行协方差矩阵更新，
 *          适用于非线性系统的预测步骤
 */
Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  // 更新协方差矩阵：P = F * P * F^T + Q
  P = F * P * F.transpose() + Q;
  // 更新状态向量：x = f(x)
  x = f(x);
  return x;
}

/**
 * @brief 线性更新步骤
 * @param z 观测向量
 * @param H 观测矩阵
 * @param R 观测噪声协方差矩阵
 * @param z_subtract 观测向量减法函数，用于处理角度环绕等问题
 * @return 更新后的状态向量
 * @details 使用线性观测矩阵H进行状态更新，适用于线性观测模型
 */
Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

/**
 * @brief 非线性更新步骤
 * @param z 观测向量
 * @param H 观测矩阵（用于卡尔曼增益计算）
 * @param R 观测噪声协方差矩阵
 * @param h 非线性观测函数
 * @param z_subtract 观测向量减法函数，用于处理角度环绕等问题
 * @return 更新后的状态向量
 * @details 使用非线性观测函数h进行状态更新，同时使用H进行卡尔曼增益计算，
 *          适用于非线性观测模型的更新步骤。包含卡方检验用于评估滤波器性能。
 */
Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  // 保存先验状态
  Eigen::VectorXd x_prior = x;
  
  // 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // 使用Joseph形式稳定计算后验协方差矩阵
  // 参考：https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  // 更新状态向量：x = x + K * (z - h(x))
  x = x_add(x, K * z_subtract(z, h(x)));

  /// 卡方检验 - 用于评估滤波器性能
  // 计算新息（观测残差）
  Eigen::VectorXd residual = z_subtract(z, h(x));
  
  // 计算新息协方差矩阵
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  
  // 计算归一化新息平方(NIS)
  double nis = residual.transpose() * S.inverse() * residual;
  
  // 计算归一化估计误差平方(NEES)
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // 卡方检验阈值（自由度=4，取置信水平95%）
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  // 更新检验统计
  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  // 记录近期NIS失败情况
  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  // 保持滑动窗口大小
  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  // 计算近期NIS失败率
  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  // 更新调试数据
  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools
