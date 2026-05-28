# 云台控制指令生成与坐标变换流程

## 1. 整体流程概述

通过串口发送给下位机的yaw和pitch指令是经过一系列复杂的坐标变换和控制算法生成的。整体流程可以分为以下几个主要阶段：

1. **图像采集与装甲板检测**：从摄像头获取图像，检测装甲板位置
2. **PnP解算**：将2D图像坐标转换为3D相机坐标系坐标
3. **坐标变换**：将相机坐标系坐标转换为世界坐标系坐标
4. **目标跟踪与状态估计**：使用EKF预测目标未来位置
5. **轨迹规划**：使用MPC生成云台控制轨迹
6. **串口发送**：将计算得到的yaw、pitch转换为合适格式发送给下位机

## 2. 坐标系统定义

在整个流程中，涉及多个坐标系统的转换：

### 2.1 相机坐标系 (Camera Coordinate System)
- 原点：相机光心
- X轴：指向相机右侧
- Y轴：指向相机下方
- Z轴：指向相机前方（光轴方向）

### 2.2 云台坐标系 (Gimbal Coordinate System)
- 原点：云台旋转中心
- X轴：指向云台右侧
- Y轴：指向云台下方
- Z轴：指向云台前方

### 2.3 世界坐标系 (World Coordinate System)
- 原点：机器人底盘中心
- X轴：指向机器人前方
- Y轴：指向机器人左侧
- Z轴：指向机器人上方

## 3. 详细的坐标变换过程

### 3.1 从图像到相机坐标系：PnP解算

```cpp
// 代码位置：tasks/auto_aim/solver.cpp
void Solver::solve(Armor &armor) const {
  // 使用预定义的装甲板3D模型点
  const auto &object_points = (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  
  // 使用OpenCV的solvePnP函数计算3D坐标
  cv::Vec3d rvec, tvec;
  cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
  
  // 将结果转换为Eigen向量
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  
  // 存储相机坐标系下的装甲板坐标
  armor.xyz_in_camera = xyz_in_camera;
}
```

### 3.2 从相机坐标系到云台坐标系

```cpp
// 代码位置：tasks/auto_aim/solver.cpp
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
```

- `R_camera2gimbal_`：相机到云台的旋转矩阵
- `t_camera2gimbal_`：相机到云台的平移向量

### 3.3 从云台坐标系到世界坐标系

```cpp
// 代码位置：tasks/auto_aim/solver.cpp
armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
```

- `R_gimbal2world_`：云台到世界坐标系的旋转矩阵

### 3.4 从世界坐标系到云台控制指令

```cpp
// 代码位置：tasks/auto_aim/planner/planner.cpp
Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed) {
  // ... 省略中间代码 ...
  
  // 计算目标方位角
  auto azim = std::atan2(xyz.y(), xyz.x());
  
  // 计算弹丸轨迹
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");
  
  // 返回yaw和pitch指令，考虑偏移量
  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}
```

## 4. 目标跟踪与状态估计 (EKF)

### 4.1 EKF状态向量定义

```cpp
// 代码位置：tasks/auto_aim/target.cpp
// 初始化EKF状态向量：11维 [x, vx, y, vy, z, vz, a, w, r, l, h]
Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};
```

- x, y, z: 目标旋转中心坐标
- vx, vy, vz: 目标旋转中心速度
- a: 目标旋转角度
- w: 目标旋转角速度
- r: 目标旋转半径
- l: 长半轴与短半轴差值
- h: 高度差值

### 4.2 EKF预测过程

```cpp
// 代码位置：tasks/auto_aim/target.cpp
void Target::predict(double dt) {
  // 构造状态转移矩阵F
  Eigen::MatrixXd F{/* 11x11状态转移矩阵 */};
  
  // 计算过程噪声协方差矩阵Q
  Eigen::MatrixXd Q{/* 11x11噪声协方差矩阵 */};
  
  // 定义状态转移函数
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);  // 限制角度在[-π, π]范围内
    return x_prior;
  };
  
  // 执行EKF预测
  ekf_.predict(F, Q, f);
}
```

### 4.3 EKF更新过程

```cpp
// 代码位置：tasks/auto_aim/target.cpp
void Target::update(const Armor & armor) {
  // 装甲板匹配：找到与当前预测最匹配的装甲板ID
  int id = find_best_matching_armor(armor);
  
  // 执行EKF更新
  update_ypda(armor, id);
}
```

## 5. 轨迹规划与控制 (MPC)

### 5.1 MPC轨迹生成

```cpp
// 代码位置：tasks/auto_aim/planner/planner.cpp
Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed) {
  Trajectory traj;
  
  // 预测未来状态
  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);
  
  target.predict(DT);
  auto yaw_pitch = aim(target, bullet_speed);
  
  // 生成轨迹
  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);
    
    // 计算角速度
    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);
    
    // 填充轨迹数据
    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;
    
    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }
  
  return traj;
}
```

### 5.2 MPC求解

```cpp
// 代码位置：tasks/auto_aim/planner/planner.cpp
Plan Planner::plan(Target target, double bullet_speed) {
  // ... 省略中间代码 ...
  
  // 求解yaw控制器
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);
  
  // 求解pitch控制器
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);
  
  // 构造控制指令
  Plan plan;
  plan.control = true;
  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);
  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);
  
  // ... 省略开火判断代码 ...
  
  return plan;
}
```

## 6. 串口发送与数据格式

### 6.1 数据转换与发送

```cpp
// 代码位置：io/gimbal/gimbal.cpp
void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (control) {
    mode_ = GimbalMode::AUTO_AIM;
    
    // 将弧度转换为角度（下位机期望接收角度值）
    ::gimbal_send.yaw = yaw * 57.3f;
    ::gimbal_send.pitch = pitch * 57.3f;
    
    // 设置开火状态
    ::gimbal_send.fire = fire ? 1 : 0;
    
    // 调用C代码的发送函数
    ::defUartSend(gimbal);
  } else {
    mode_ = GimbalMode::IDLE;
  }
}
```

### 6.2 数据结构定义

```cpp
// 代码位置：DataType.h (C语言头文件)
// 云台发送数据结构
typedef struct
{
    float yaw;        // 云台yaw轴角度（度）
    float pitch;      // 云台pitch轴角度（度）
    uint8_t fire;     // 开火指令：0-不开火，1-开火
} GimbalSend;
```

## 7. 关键算法与技术

### 7.1 扩展卡尔曼滤波 (EKF)

- **状态向量**：11维，包含位置、速度、旋转角度、角速度等
- **预测模型**：使用分段白噪声模型描述目标运动
- **观测模型**：基于装甲板的yaw、pitch、距离和角度进行更新

### 7.2 模型预测控制 (MPC)

- **优化目标**：跟踪参考轨迹，同时考虑控制输入约束
- **预测时域**：HORIZON（如100个时间步）
- **控制输入**：yaw和pitch的加速度

### 7.3 弹丸轨迹计算

考虑重力影响，计算弹丸飞行时间和所需的俯仰角：

```cpp
// 代码位置：tools/trajectory.cpp
tools::Trajectory::Trajectory(double bullet_speed, double distance, double z_diff) {
  // 计算弹丸飞行轨迹
  // ... 省略实现细节 ...
}
```

## 8. 坐标变换矩阵的构建

### 8.1 相机到云台的变换

```cpp
// 代码位置：tasks/auto_aim/solver.cpp
// 旋转矩阵：相机到云台
R_camera2gimbal_ = Eigen::Matrix3d::Identity();
R_camera2gimbal_ << 0, 0, 1,
                    1, 0, 0,
                    0, 1, 0;

// 平移向量：相机到云台
Eigen::Vector3d t_camera2gimbal_ = {0.0f, 0.0f, 0.0f};
```

### 8.2 云台到世界的变换

```cpp
// 代码位置：tasks/auto_aim/solver.cpp
// 根据云台反馈的yaw和pitch构造旋转矩阵
Eigen::Quaterniond q_gimbal = gimbal.q();
R_gimbal2world_ = q_gimbal.toRotationMatrix();
```

## 9. 总结

通过串口发送给下位机的yaw和pitch指令是经过一系列复杂的坐标变换和控制算法生成的。整个过程涉及多个坐标系统的转换，包括图像坐标系、相机坐标系、云台坐标系和世界坐标系。目标跟踪使用EKF进行状态估计，轨迹规划使用MPC生成平滑的控制指令，最终转换为合适的格式发送给下位机。

这种复杂的流程确保了云台能够准确地跟踪和瞄准目标，同时考虑了弹丸飞行轨迹和目标运动预测，提高了射击精度。