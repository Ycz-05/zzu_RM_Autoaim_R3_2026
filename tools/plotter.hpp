/**
 * @file plotter.hpp
 * @brief 绘图工具类头文件
 * @details 该文件声明了Plotter类，用于通过UDP套接字发送JSON格式的数据到指定的主机和端口，
 *          主要用于实时数据可视化和调试。
 */

#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in - 网络地址结构

#include <mutex>         // 互斥锁，用于线程安全
#include <nlohmann/json.hpp>  // JSON库，用于数据序列化
#include <string>        // 字符串处理

namespace tools
{
/**
 * @class Plotter
 * @brief 绘图工具类
 * @details 用于通过UDP协议发送JSON格式的数据到指定的主机和端口，
 *          通常用于实时数据可视化和调试目的。
 */
class Plotter
{
public:
  /**
   * @brief 构造函数
   * @param host 目标主机的IP地址，默认为"127.0.0.1"（本地主机）
   * @param port 目标端口号，默认为9870
   */
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

  /**
   * @brief 析构函数
   */
  ~Plotter();

  /**
   * @brief 发送JSON数据到目标主机
   * @param json 要发送的JSON数据对象
   */
  void plot(const nlohmann::json & json);

private:
  int socket_;          ///< UDP套接字文件描述符
  sockaddr_in destination_;  ///< 目标地址信息
  std::mutex mutex_;     ///< 互斥锁，用于确保plot方法的线程安全
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP