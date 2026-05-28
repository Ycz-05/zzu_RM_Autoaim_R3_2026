# MPC预测原理详解

## 概述

模型预测控制（Model Predictive Control, MPC）是一种先进的控制策略，在您的自动瞄准系统中用于生成平滑的云台控制轨迹。MPC通过预测系统未来行为并优化控制输入来实现精确的目标跟踪。

## MPC核心原理

### 1. 预测模型

MPC基于系统的数学模型来预测未来状态。在您的系统中，yaw轴和pitch轴分别使用独立的MPC控制器。

#### 状态空间模型

```cpp
// 状态转移矩阵
Eigen::MatrixXd A{{1, DT}, {0, 1}};  // 状态转移矩阵
Eigen::MatrixXd B{{0}, {DT}};        // 控制输入矩阵
Eigen::VectorXd f{{0, 0}};            // 偏置向量
```

**状态变量定义：**
- yaw轴：`[yaw角度, yaw角速度]`
- pitch轴：`[pitch角度, pitch角速度]`

**状态转移方程：**
```
x(k+1) = A * x(k) + B * u(k) + f
```

其中：
- `x(k)`：当前时刻状态向量
- `u(k)`：当前时刻控制输入（角加速度）
- `A`：状态转移矩阵
- `B`：控制输入矩阵
- `DT`：采样时间间隔

### 2. 滚动优化

MPC在每个控制周期内解决一个有限时域的最优控制问题：

#### 代价函数
```cpp
// 状态权重矩阵Q
Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
// 控制权重矩阵R  
Eigen::Matrix<double, 1, 1> R(R_yaw.data());
```

**优化目标：**
```
min J = Σ [x(i) - x_ref(i)]^T * Q * [x(i) - x_ref(i)] + u(i)^T * R * u(i)
```

其中：
- `Q`：状态跟踪误差权重矩阵
- `R`：控制输入权重矩阵
- `x_ref(i)`：参考轨迹

### 3. 约束处理

MPC能够处理系统的物理约束：

```cpp
// 控制输入约束
Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
```

**约束类型：**
- 角加速度约束：防止云台过载
- 状态约束：确保角度在合理范围内

## 预测流程详解

### 1. 目标预测

#### 子弹飞行时间计算
```cpp
// 计算子弹飞行时间
auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
// 预测目标未来位置
target.predict(bullet_traj.fly_time);
```

**关键步骤：**
1. 计算子弹从发射到命中目标的飞行时间
2. 使用EKF预测目标在飞行时间后的位置
3. 考虑目标运动补偿

#### EKF目标状态预测

**状态向量（11维）：**
```cpp
[x, vx, y, vy, z, vz, a, w, r, l, h]
```
- `x, y, z`：目标旋转中心坐标
- `vx, vy, vz`：目标旋转中心速度  
- `a`：目标旋转角度
- `w`：目标旋转角速度
- `r`：目标旋转半径
- `l`：长半轴与短半轴差值
- `h`：高度差值

**预测方程：**
```cpp
void Target::predict(double dt) {
    // 状态转移矩阵F
    Eigen::MatrixXd F{
        {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},  // x = x + vx*dt
        {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},  // vx = vx
        {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},  // y = y + vy*dt
        // ... 完整的状态转移矩阵
    };
    
    ekf_.predict(F, Q, f);
}
```

### 2. 参考轨迹生成

#### 轨迹生成算法
```cpp
Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed) {
    // 预测历史时刻的目标位置
    target.predict(-DT * (HALF_HORIZON + 1));
    
    // 生成未来HORIZON个时刻的轨迹
    for (int i = 0; i < HORIZON; i++) {
        target.predict(DT);
        auto yaw_pitch_next = aim(target, bullet_speed);
        
        // 使用中心差分法计算角速度
        auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
        
        // 保存轨迹点
        traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;
    }
}
```

**轨迹数据结构：**
```cpp
// 轨迹矩阵：4×HORIZON
// 行0: 相对yaw角度
// 行1: yaw角速度  
// 行2: pitch角度
// 行3: pitch角速度
```

### 3. MPC求解

#### yaw轴MPC求解
```cpp
// 设置初始状态
Eigen::VectorXd x0(2);
x0 << traj(0, 0), traj(1, 0);
tiny_set_x0(yaw_solver_, x0);

// 设置参考轨迹
yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);

// 求解MPC问题
tiny_solve(yaw_solver_);
```

#### pitch轴MPC求解
```cpp
// 类似的求解过程
x0 << traj(2, 0), traj(3, 0);
tiny_set_x0(pitch_solver_, x0);

pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
tiny_solve(pitch_solver_);
```

## 关键技术特点

### 1. 多时间尺度预测

**预测时域结构：**
- `HORIZON`：总预测步数（通常为10-20）
- `HALF_HORIZON`：中间时刻索引
- `DT`：采样时间间隔（通常为0.01-0.05秒）

### 2. 自适应延迟补偿

```cpp
// 根据目标速度选择延迟时间
double delay_time = 
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
```

**策略：**
- 高速目标：使用较短的延迟时间
- 低速目标：使用较长的延迟时间

### 3. 开火决策机制

```cpp
// 检查预测轨迹与MPC轨迹的偏差
plan.fire = std::hypot(
    traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
    traj(2, HALF_HORIZON + shoot_offset_) - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
```

**开火条件：**
- 预测轨迹与MPC轨迹偏差小于阈值
- 考虑子弹飞行时间偏移

## 数学推导

### 1. 状态空间模型推导

**连续时间模型：**
```
dyaw/dt = yaw_vel
dyaw_vel/dt = yaw_acc
```

**离散化（前向欧拉法）：**
```
yaw(k+1) = yaw(k) + DT * yaw_vel(k)
yaw_vel(k+1) = yaw_vel(k) + DT * yaw_acc(k)
```

**矩阵形式：**
```
[x1(k+1)]   [1  DT] [x1(k)]   [0  ]
[x2(k+1)] = [0  1 ] [x2(k)] + [DT] * u(k)
```

### 2. 优化问题求解

**QP问题形式：**
```
min 1/2 * U^T * H * U + f^T * U
s.t. A_eq * U = b_eq
     A_ineq * U ≤ b_ineq
```

其中：
- `U`：控制输入序列
- `H`：Hessian矩阵
- `f`：梯度向量

## 性能优化策略

### 1. 实时性保障

**优化措施：**
- 使用高效的QP求解器（TinyMPC）
- 限制最大迭代次数（通常为10次）
- 预计算不变矩阵

### 2. 数值稳定性

**稳定性措施：**
- 角度值限制在[-π, π]范围内
- 使用Joseph形式更新协方差矩阵
- 添加正则化项防止数值病态

## 应用效果

### 1. 跟踪精度
- 能够准确预测目标未来位置
- 实现平滑的云台运动轨迹
- 减少超调和振荡

### 2. 抗干扰能力
- 对目标运动突变有较好的适应性
- 能够处理短暂的遮挡和丢失
- 对测量噪声有较强的鲁棒性

### 3. 实时性能
- 在嵌入式系统上实现实时控制
- 计算复杂度可控
- 内存占用优化

## 总结

MPC预测原理在您的自动瞄准系统中发挥了核心作用，通过：

1. **精确预测**：基于EKF的目标状态预测
2. **优化控制**：滚动时域优化生成最优控制序列  
3. **约束处理**：考虑系统物理限制
4. **实时性能**：高效求解保证控制实时性

这种预测-优化-执行的闭环控制策略，确保了系统在各种工况下的稳定性和精确性。