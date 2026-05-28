/**
 * @file trajectory.cpp
 * @brief 弹道计算类实现
 * @details 该文件实现了基于物理运动学的弹道计算功能，用于求解弹丸的发射角度和飞行时间
 * @author 
 * @version 1.0
 * @date 
 */

#include "trajectory.hpp"

#include <cmath>

// 工具类命名空间
namespace tools
{
/**
 * @brief 重力加速度常量
 * @details 采用当地重力加速度值9.7833 m/s²，用于弹道计算
 */
constexpr double g = 9.7833;

/**
 * @brief 弹道构造函数
 * @param v0 弹丸初速度（单位：米/秒）
 * @param d 目标水平距离（单位：米）
 * @param h 目标垂直高度差（单位：米），正值表示目标高于发射点，负值表示目标低于发射点
 * @details 该构造函数通过求解二次方程计算弹丸的发射角度和飞行时间
 *          考虑了重力加速度的影响，使用抛物线运动模型
 */
Trajectory::Trajectory(const double v0, const double d, const double h)
{
  // 二次方程参数计算：a * tan(pitch)^2 + b * tan(pitch) + c = 0
  // 推导过程：基于抛物线运动学方程，将垂直位移和水平位移关系转换为关于tan(pitch)的二次方程
  auto a = g * d * d / (2 * v0 * v0);  // 二次项系数
  auto b = -d;                         // 一次项系数
  auto c = a + h;                      // 常数项
  auto delta = b * b - 4 * a * c;      // 判别式

  // 检查方程是否有实数解
  if (delta < 0) {
    unsolvable = true;  // 无解标记
    return;
  }

  unsolvable = false;  // 有解标记

  // 求解二次方程，得到两个可能的tan(pitch)值
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);  // 第一个解
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);  // 第二个解

  // 将tan(pitch)转换为角度值（弧度）
  auto pitch_1 = std::atan(tan_pitch_1);  // 第一个可能的发射角度
  auto pitch_2 = std::atan(tan_pitch_2);  // 第二个可能的发射角度

  // 计算两种发射角度下的飞行时间
  auto t_1 = d / (v0 * std::cos(pitch_1));  // 第一个角度对应的飞行时间
  auto t_2 = d / (v0 * std::cos(pitch_2));  // 第二个角度对应的飞行时间

  // 选择飞行时间较短的解作为最终结果
  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;      // 最终发射角度（弧度）
  fly_time = (t_1 < t_2) ? t_1 : t_2;           // 最终飞行时间（秒）
}

}  // namespace tools