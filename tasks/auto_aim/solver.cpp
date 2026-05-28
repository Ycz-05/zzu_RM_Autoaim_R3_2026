/**
 * @file solver.cpp
 * @brief 自动瞄准系统的姿态解算器实现
 * @details 该文件实现了自动瞄准系统中的姿态解算功能，包括使用solvePnP算法获取装甲板姿态、
 *          坐标变换、重投影误差计算以及yaw角优化等功能
 * @author 
 * @version 1.0
 * @date 
 */

#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 自动瞄准命名空间
namespace auto_aim
{
// 常量定义
constexpr double LIGHTBAR_LENGTH = 56e-3;     // 灯条长度，单位：米
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // 大装甲板宽度，单位：米
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // 小装甲板宽度，单位：米

// 大装甲板的三维点坐标（装甲板坐标系，z轴朝里）
const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},   // 右上角
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},  // 左上角
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}, // 左下角
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}    // 右下角
};

// 小装甲板的三维点坐标（装甲板坐标系，z轴朝里）
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},   // 右上角
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},  // 左上角
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}, // 左下角
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}    // 右下角
};

/**
 * @brief 构造函数，从配置文件加载参数
 * @param config_path 配置文件路径
 */
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  // 加载YAML配置文件
  auto yaml = YAML::LoadFile(config_path);

  // 加载坐标系转换矩阵和位移向量
  //R_gimbal2imubody：云台到IMU本体坐标系的旋转矩阵
  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  //R_camera2gimbal：相机到云台坐标系的旋转矩阵
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  //t_camera2gimbal：相机到云台坐标系的位移向量（单位：米）
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  //t_gimbal2imubody：云台到IMU本体坐标系的位移向量（单位：米）
  auto t_gimbal2imubody_data = yaml["t_gimbal2imubody"].as<std::vector<double>>();
  
  // 将向量数据转换为Eigen矩阵
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());
  t_gimbal2imubody_ = Eigen::Matrix<double, 3, 1>(t_gimbal2imubody_data.data());

  // 加载相机内参和畸变系数
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  
  // 将向量数据转换为Eigen矩阵
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  
  // 将Eigen矩阵转换为OpenCV矩阵
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

/**
 * @brief 获取云台到世界坐标系的旋转矩阵
 * @return 云台到世界坐标系的旋转矩阵
 */
Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

/**
 * @brief 设置云台到世界坐标系的旋转矩阵
 * @param q 四元数，表示IMU本体到IMU绝对坐标系的旋转
 */
void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  // 将四元数转换为旋转矩阵：IMU本体（gimbal.q()获取到的此时下位机传上来的云台四元数）到IMU绝对坐标系
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  
  // 计算云台到世界坐标系的旋转矩阵（R_gimbal2world_）
  // 公式：R_gimbal2world_ = R_gimbal2imubody_^T * R_imubody2imuabs * R_gimbal2imubody_
  // 解释：
  // - R_gimbal2imubody_^T：云台到IMU本体坐标系的旋转矩阵的转置
  // - R_imubody2imuabs：IMU本体到IMU绝对坐标系的旋转矩阵
  // - R_gimbal2imubody_：云台到IMU本体坐标系的旋转矩阵
  // 结果：将云台坐标系转换到世界坐标系的旋转矩阵
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
  
  // 计算云台到世界坐标系的位移向量（t_gimbal2world_）
  // 公式：t_gimbal2world_ = R_gimbal2imubody_^T * R_imubody2imuabs * t_gimbal2imubody_
  // 解释：
  // - R_gimbal2imubody_^T：云台到IMU本体坐标系的旋转矩阵的转置
  // - R_imubody2imuabs：IMU本体到IMU绝对坐标系的旋转矩阵
  // - t_gimbal2imubody_：云台到IMU本体坐标系的位移向量
  // 结果：将云台坐标系转换到世界坐标系的位移向量
  t_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * t_gimbal2imubody_;
}

/**
 * @brief 使用solvePnP算法求解装甲板的姿态
 * @param armor 装甲板对象，包含像素坐标和类型信息，解算结果将存储在该对象中
 */
void Solver::solve(Armor & armor) const
{
  // 根据装甲板类型选择对应的三维点坐标
  const auto & object_points = 
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  // 旋转向量和位移向量
  cv::Vec3d rvec, tvec;
  
  // 使用IPPE算法求解PnP问题
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  // 将平移向量转换为Eigen向量（相机坐标系下的目标坐标）
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  
  // 坐标变换：相机坐标系 -> 云台坐标系（将相机坐标系下的目标坐标转换为云台坐标系下的目标坐标）
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  
  // 坐标变换：云台坐标系 -> 世界坐标系（将云台坐标系下的目标坐标转换为世界坐标系下的目标坐标）
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal + t_gimbal2world_;

  // 将旋转向量转换为旋转矩阵
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  
  // 将旋转矩阵转换为Eigen矩阵
  // R_armor2camera：装甲板到相机坐标系的旋转矩阵(solvePnP求解得到的相机坐标系到装甲板坐标系的旋转矩阵)
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  
  // 计算装甲板到云台坐标系的旋转矩阵
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  
  // 计算装甲板到世界坐标系的旋转矩阵
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  
  // 将旋转矩阵转换为欧拉角（yaw, pitch, roll）
  // 解释：
  // - R_armor2gimbal：装甲板到云台坐标系的旋转矩阵
  // - R_armor2world：装甲板到世界坐标系的旋转矩阵
  // - 2, 1, 0：指定欧拉角的顺序为yaw, pitch, roll
  // 结果：将装甲板到云台坐标系和世界坐标系的旋转矩阵转换为对应的欧拉角 
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  // 计算世界坐标系下的yaw, pitch, distance
  // yaw：装甲板在世界坐标系中的yaw角（范围：[-π, π]）
  // pitch：装甲板在世界坐标系中的pitch角（范围：[-π/2, π/2]）
  // distance：装甲板到相机的距离（单位：米）
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 平衡装甲板不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  // 优化yaw角
  optimize_yaw(armor);
}

/**
 * @brief 将装甲板从世界坐标系重投影到图像坐标系
 * @param xyz_in_world 装甲板在世界坐标系中的位置
 * @param yaw yaw角
 * @param type 装甲板类型
 * @param name 装甲板名称
 * @return 重投影后的图像坐标点
 */
std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
// 解释：
// - xyz_in_world：装甲板在世界坐标系中的位置
// - yaw：装甲板在世界坐标系中的yaw角（范围：[-π, π]）
// - type：装甲板类型（大装甲板或小装甲板）
// - name：装甲板名称（前哨站装甲板或其他装甲板）
// 结果：将装甲板从世界坐标系重投影到图像坐标系
{
  // 计算yaw角的正弦和余弦值
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  // 根据装甲板名称设置pitch角
  // 前哨站装甲板pitch角为-15°，其他装甲板pitch角为15°
  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // 构建装甲板到世界坐标系的旋转矩阵
  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // 计算装甲板到相机坐标系的旋转矩阵和位移向量
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  // 解释：
  // - R_camera2gimbal_.transpose()：相机坐标系到云台坐标系的旋转矩阵的转置
  // - R_gimbal2world_.transpose()：云台坐标系到世界坐标系的旋转矩阵的转置
  // - R_armor2world：装甲板到世界坐标系的旋转矩阵
  // 结果：将装甲板到世界坐标系的旋转矩阵转换为装甲板到相机坐标系的旋转矩阵
  Eigen::Matrix3d R_armor2camera = 
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera = 
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * (t_armor2world - t_gimbal2world_) - t_camera2gimbal_);

  // 将旋转矩阵转换为旋转向量
  // 解释：
  // - R_armor2camera：装甲板到相机坐标系的旋转矩阵
  // 结果：将装甲板到相机坐标系的旋转矩阵转换为对应的旋转向量
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  
  // 将位移向量转换为OpenCV向量
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // 根据装甲板类型选择对应的三维点坐标
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  
  // 重投影到图像坐标系
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

/**
 * @brief 计算前哨站装甲板的重投影误差
 * @param armor 装甲板对象
 * @param pitch pitch角
 * @return 重投影误差
 */
double Solver::outpost_reprojection_error(Armor armor, const double & pitch)
{
  // 根据装甲板类型选择对应的三维点坐标
  const auto & object_points = 
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  // 旋转向量和位移向量
  cv::Vec3d rvec, tvec;
  
  // 使用IPPE算法求解PnP问题
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  // 将位移向量转换为Eigen向量
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  
  // 坐标变换：相机坐标系 -> 云台坐标系
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  
  // 坐标变换：云台坐标系 -> 世界坐标系
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal + t_gimbal2world_;

  // 将旋转向量转换为旋转矩阵
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  
  // 将旋转矩阵转换为Eigen矩阵
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  
  // 计算装甲板到云台坐标系的旋转矩阵
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  
  // 计算装甲板到世界坐标系的旋转矩阵
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  
  // 将旋转矩阵转换为欧拉角
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  // 计算世界坐标系下的yaw, pitch, distance
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 获取yaw角和世界坐标
  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  // 计算yaw角的正弦和余弦值
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  // 计算pitch角的正弦和余弦值
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // 构建装甲板到世界坐标系的旋转矩阵
  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // 计算装甲板到相机坐标系的旋转矩阵和位移向量
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera = 
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera = 
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * (t_armor2world - t_gimbal2world_) - t_camera2gimbal_);

  // 将旋转矩阵转换为旋转向量
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  
  // 将位移向量转换为OpenCV向量
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // 重投影到图像坐标系
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  // 计算重投影误差
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

/**
 * @brief 优化装甲板的yaw角
 * @param armor 装甲板对象，优化后的yaw角将存储在该对象中
 */
void Solver::optimize_yaw(Armor & armor) const
{
  // 计算云台坐标系到世界坐标系的欧拉角
  // 解释：
  // - R_gimbal2world_：云台坐标系到世界坐标系的旋转矩阵
  // - 2, 1, 0：指定欧拉角的顺序为yaw, pitch, roll
  // 结果：将云台坐标系到世界坐标系的旋转矩阵转换为对应的欧拉角 
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  // 搜索范围设置为140度
  constexpr double SEARCH_RANGE = 140;  // degree
  
  // 计算搜索起始角度
  // 解释：
  // - gimbal_ypr[0]：云台坐标系到世界坐标系的yaw角
  // - SEARCH_RANGE / 2 * CV_PI / 180.0：搜索范围的一半，转换为弧度
  // 结果：将搜索起始角度设置为云台yaw角减去搜索范围的一半
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  // 初始化最小误差和最佳yaw角
  // 解释：
  // - min_error：初始化一个较大的误差值，用于后续更新
  // - best_yaw：初始化当前yaw角为装甲板的原始yaw角
  // 结果：将最小误差和最佳yaw角初始化为当前状态
  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  // 遍历搜索范围内的所有yaw角
  for (int i = 0; i < SEARCH_RANGE; i++) {
    // 计算当前yaw角
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    
    // 计算重投影误差
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    // 更新最小误差和最佳yaw角
    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  // 保存原始yaw角并更新为优化后的yaw角
  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

/**
 * @brief 计算SJTU代价函数
 * @param cv_refs 参考点的像素坐标
 * @param cv_pts 实际点的像素坐标
 * @param inclined 倾斜角度
 * @return 代价函数值
 */
double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  
  // 将OpenCV点转换为Eigen向量
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  
  double cost = 0.;
  // 遍历所有点对
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;  // 下一个点的索引
    
    // 计算向量
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准向量
    Eigen::Vector2d pt_d = pts[p] - pts[i];     // 实际向量
    
    // 计算像素距离误差
    double pixel_dis =  
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    
    // 计算角度误差
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    
    // 计算代价
    double cost_i = 
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // 角度误差权重
    
    // 累加代价
    cost += std::sqrt(cost_i);
  }
  return cost;
}

/**
 * @brief 计算装甲板的重投影误差
 * @param armor 装甲板对象
 * @param yaw yaw角
 * @param inclined 倾斜角度
 * @return 重投影误差
 */
double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  // 重投影装甲板
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  
  // 计算重投影误差
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  
  // 可以使用SJTU代价函数替代简单的距离误差
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

/**
 * @brief 将世界坐标系中的点转换为像素坐标
 * @param worldPoints 世界坐标系中的点
 * @return 像素坐标
 */
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  // 计算世界坐标系到相机坐标系的旋转矩阵
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  
  // 计算世界坐标系到相机坐标系的位移向量
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_gimbal2world_ + t_camera2gimbal_);

  // 将旋转矩阵转换为OpenCV矩阵
  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  // 过滤掉相机背面的点（z坐标小于等于0）
  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  
  // 如果没有有效点，返回空向量
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  
  // 将有效点投影到图像坐标系
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  
  return pixelPoints;
}
}  // namespace auto_aim