/**
 * @file trajectory.hpp
 * @brief 弹道计算类头文件
 * @details 该文件定义了弹道计算的结构体和接口，用于计算弹丸的发射角度和飞行时间
 * @author 
 * @version 1.0
 * @date 
 */

#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

// 工具类命名空间
namespace tools
{
/**
 * @brief 弹道计算结构体
 * @details 用于存储弹道计算的结果，包括发射角度、飞行时间和求解状态
 */
struct Trajectory
{
  bool unsolvable;  ///< 标记弹道是否有解，true表示无解，false表示有解
  double fly_time;  ///< 弹丸飞行时间（单位：秒）
  double pitch;     ///< 发射角度（单位：弧度），抬头为正，低头为负

  /**
   * @brief 弹道构造函数
   * @param v0 弹丸初速度大小（单位：米/秒）
   * @param d 目标水平距离（单位：米）
   * @param h 目标垂直高度差（单位：米），正值表示目标高于发射点，负值表示目标低于发射点
   * @details 该构造函数通过求解二次方程计算弹丸的发射角度和飞行时间
   *          采用抛物线运动模型，假设不考虑空气阻力
   */
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP