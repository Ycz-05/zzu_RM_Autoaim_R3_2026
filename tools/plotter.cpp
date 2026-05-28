/**
 * @file plotter.cpp
 * @brief 绘图工具类实现
 * @details 该文件实现了Plotter类，用于通过UDP套接字发送JSON格式的数据到指定的主机和端口，
 *          主要用于实时数据可视化和调试。
 */

#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr - 网络字节序转换和IP地址解析
#include <sys/socket.h>  // socket, sendto - 套接字操作
#include <unistd.h>      // close - 关闭文件描述符

namespace tools
{
/**
 * @class Plotter
 * @brief 绘图工具类
 * @details 用于通过UDP协议发送JSON格式的数据到指定的主机和端口，
 *          通常用于实时数据可视化和调试目的。
 */

/**
 * @brief 构造函数
 * @param host 目标主机的IP地址
 * @param port 目标端口号
 * @details 创建一个UDP套接字并设置目标地址信息
 */
Plotter::Plotter(std::string host, uint16_t port)
{
  // 创建UDP套接字
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  // 设置目标地址信息
  destination_.sin_family = AF_INET;                 // 使用IPv4地址族
  destination_.sin_port = ::htons(port);             // 将端口号转换为网络字节序
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());  // 将IP地址字符串转换为网络字节序
}

/**
 * @brief 析构函数
 * @details 关闭UDP套接字，释放资源
 */
Plotter::~Plotter() { ::close(socket_); }

/**
 * @brief 发送JSON数据到目标主机
 * @param json 要发送的JSON数据对象
 * @details 将JSON对象转换为字符串，并通过UDP套接字发送到指定的目标地址
 */
void Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);  // 加锁，确保线程安全
  auto data = json.dump();                    // 将JSON对象转换为字符串
  // 通过UDP发送数据
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

}  // namespace tools