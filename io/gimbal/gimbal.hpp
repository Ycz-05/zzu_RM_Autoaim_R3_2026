#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <memory> // For std::unique_ptr

// 前向声明 C++ 串口驱动类
class UartLinux;

namespace io
{
// 公共API中的枚举和结构体保持不变，以确保兼容性
enum class GimbalMode
{
  IDLE,
  AUTO_AIM,
  // 注意：新协议可能不支持以下模式，但为保持API兼容性而保留
  SMALL_BUFF,
  BIG_BUFF
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float roll;
  float bullet_speed;
  uint16_t bullet_count;
  uint8_t fire;
};

class Gimbal
{
public:
  float yaw;
  float pitch;
  float roll;
  // 构造函数API不变，但内部实现会改变
  Gimbal(const std::string & config_path); // 依然可以接收配置文件路径
  Gimbal(const std::string & device_name, int baud_rate); // 或者提供一个更直接的构造函数
  Eigen::Quaterniond q() const; 
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);
  ~Gimbal();

  // 以下公共API签名完全保持不变
  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

private:
  // ==================== 私有成员被完全替换 ====================
  
  // 轮询线程，用于检查C代码中的全局变量
  void pollingLoop();

  // C++ 串口驱动的智能指针
  std::shared_ptr<UartLinux> uart_driver_;

  // 线程控制
  std::thread polling_thread_;
  std::atomic<bool> is_running_{false};
  
  // 互斥锁，用于保护共享状态
  mutable std::mutex mutex_;

  // 内部状态变量，由轮询线程更新
  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_ = {}; // 初始化为空
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
