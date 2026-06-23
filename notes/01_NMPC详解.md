# 01. NMPC (非线性模型预测控制) 详解

## 1. 基本原理

### 1.1 什么是 NMPC？

**NMPC (Nonlinear Model Predictive Control)** 是一种基于模型的最优控制方法：

1. **预测**: 使用动力学模型预测未来状态
2. **优化**: 求解最优控制序列
3. **执行**: 只执行第一步，然后重新优化 (滚动时域)

```
当前时刻 t
  ↓
预测未来 N 步 (t+1, t+2, ..., t+N)
  ↓
优化控制序列 [u(t), u(t+1), ..., u(t+N-1)]
  ↓
执行 u(t)
  ↓
移动到 t+1，重新优化
```

### 1.2 为什么用 NMPC？

**优势**:
1. **预测能力**: 提前规划，避免碰撞
2. **约束处理**: 显式处理摩擦锥、关节限位
3. **最优性**: 最小化能量消耗、跟踪误差
4. **适应性**: 可以处理复杂的切换动态 (步态切换)

**劣势**:
1. **计算负担**: 需要实时求解优化问题
2. **模型依赖**: 需要准确的动力学模型
3. **调参复杂**: 需要调节权重矩阵 Q, R

## 2. 数学建模

### 2.1 系统状态

**状态变量** (19维):
```
x = [h_com, q_b, q_j]
    ├─ h_com: 质心动量 (6维)
    │   ├─ 线动量 (3维): m * v_com
    │   └─ 角动量 (3维): I * ω
    ├─ q_b: 基座位姿 (7维)
    │   ├─ 位置 (3维): [x, y, z]
    │   └─ 四元数 (4维): [w, x, y, z]
    └─ q_j: 关节角度 (12维)
        └─ 4条腿 × 3个关节
```

**控制输入** (24维):
```
u = [f_c, v_j]
    ├─ f_c: 接触力 (12维)
    │   └─ 4脚 × 3维力 [f_x, f_y, f_z]
    └─ v_j: 关节速度 (12维)
```

### 2.2 动力学模型

**质心动力学** (简化模型):
```
ḣ_com = A * h_com + B * f_c + g

其中:
  h_com = [m*v_com; L]  (质心动量)
  A = [0, 0; 0, 0]      (系统矩阵)
  B = [I_3, 0; 0, I_3]  (输入矩阵)
  g = [0, 0, -m*g, 0, 0, 0]^T  (重力)
```

**广义坐标动力学** (完整模型):
$$
\mathbf{M}(q) * \ddot{q} + \mathbf{C}(q, \dot{q}) * \dot{q} + g(q) = S * \boldsymbol{\tau} + J^{T} * \mathbf{f}_{c}

where:
  \mathbf{M}(q): inertia matrix (19\times19)
  \mathbf{C}(q, \dot{q}): Coriolis matrix
  g(q): gravity vector
  S: selection matrix (selection matrix for joint torques)
  J: Jacobian matrix
$$

### 2.3 优化问题

**目标函数**:
```
min Σ_{k=0}^{N-1} [ (x_k - x_ref_k)^T Q (x_k - x_ref_k) 
                    + (u_k - u_ref_k)^T R (u_k - u_ref_k) ]
+ (x_N - x_ref_N)^T Q_f (x_N - x_ref_N)

其中:
  Q: 状态权重矩阵 (19×19)
  R: 输入权重矩阵 (24×24)
  Q_f: 终端权重矩阵
  N: 预测时域 (通常 50 步)
```

**约束条件**:

1. **动力学约束**:
$$
x_{k+1} = f(\mathbf{x}_{k}, \mathbf{u}_{k})  (discrete dynamics)
$$

2. **摩擦锥约束**:
$$
\sqrt(\mathbf{f}_{x}^{2} + \mathbf{f}_{y}^{2}) \leq \mu * \mathbf{f}_{z}
\mathbf{f}_{z} \geq 0

linearized approximation:
  \mathbf{f}_{x} \leq \mu * \mathbf{f}_{z}
  -\mathbf{f}_{x} \leq \mu * \mathbf{f}_{z}
  \mathbf{f}_{y} \leq \mu * \mathbf{f}_{z}
  -\mathbf{f}_{y} \leq \mu * \mathbf{f}_{z}
$$

3. **接触约束**:
```
支撑脚: v_foot = 0
摆动脚: v_foot = v_ref (参考轨迹)
```

4. **关节限位**:
```
q_min ≤ q ≤ q_max
τ_min ≤ τ ≤ τ_max
```

## 3. 代码分析

### 3.1 LeggedInterface

**文件**: `legged_interface/src/LeggedInterface.cpp`

**主要功能**:
1. 定义最优控制问题 (OCP)
2. 设置动力学模型
3. 定义约束和代价函数

**关键函数**:

```cpp
void LeggedInterface::setupOptimalControlProblem(...) {
    // 1. 设置模型
    setupModel(taskFile, urdfFile, referenceFile, verbose);
    
    // 2. 设置参考管理器
    setupReferenceManager(taskFile, urdfFile, referenceFile, verbose);
    
    // 3. 设置预计算
    setupPreComputation(taskFile, urdfFile, referenceFile, verbose);
    
    // 4. 定义最优控制问题
    problemPtr_ = std::make_unique<OptimalControlProblem>();
    
    // 5. 添加代价函数
    auto cost = getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose);
    problemPtr_->costPtr->add("baseTracking", std::move(cost));
    
    // 6. 添加约束
    auto frictionCone = getFrictionConeConstraint(...);
    problemPtr_->inequalityConstraintPtr->add("frictionCone", std::move(frictionCone));
    
    auto zeroVelocity = getZeroVelocityConstraint(...);
    problemPtr_->equalityConstraintPtr->add("zeroVelocity", std::move(zeroVelocity));
}
```

### 3.2 代价函数

**基座跟踪代价**:
```cpp
std::unique_ptr<StateInputCost> LeggedInterface::getBaseTrackingCost(...) {
    // 状态权重
    matrix_t Q = matrix_t::Identity(stateDim, stateDim);
    Q.block<3, 3>(0, 0) *= weightBasePosition;      // 基座位置
    Q.block<3, 3>(3, 3) *= weightBaseOrientation;   // 基座姿态
    Q.block<3, 3>(6, 6) *= weightBaseVelocity;      // 基座速度
    
    // 输入权重
    matrix_t R = initializeInputCostWeight(taskFile, info);
    
    return std::make_unique<QuadraticStateInputCost>(Q, R);
}
```

**权重矩阵示例**:
```yaml
# task.info
baseTracking:
  weightBasePosition: [10.0, 10.0, 100.0]  # x, y, z (z 更重要)
  weightBaseOrientation: [50.0, 50.0, 10.0]  # roll, pitch, yaw
  weightBaseVelocity: [10.0, 10.0, 10.0]
  weightJointVelocity: [0.1, 0.1, 0.1, ...]  # 12个关节
```

### 3.3 约束实现

**摩擦锥约束**:
```cpp
std::unique_ptr<StateInputConstraint> LeggedInterface::getFrictionConeConstraint(
    size_t contactPointIndex, scalar_t frictionCoefficient) {
    
    return std::make_unique<FrictionConeConstraint>(
        *referenceManagerPtr_,
        RelaxedBarrierPenalty::Config(frictionCoefficient, 1e-3, 1e-4),
        contactPointIndex,
        centroidalModelInfo_
    );
}
```

**零速度约束** (支撑脚):
```cpp
std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroVelocityConstraint(
    const EndEffectorKinematics<scalar_t>& eeKinematics,
    size_t contactPointIndex) {
    
    return std::make_unique<ZeroVelocityConstraintCppAd>(
        *referenceManagerPtr_,
        eeKinematics,
        contactPointIndex,
        centroidalModelInfo_
    );
}
```

### 3.4 MPC 求解器

**求解器配置**:
```cpp
void LeggedController::setupMpc() {
    mpc_ = std::make_unique<SqpMpc>(
        leggedInterface_->getOptimalControlProblem(),
        leggedInterface_->getRollout(),
        *leggedInterface_->getInitializer(),
        leggedInterface_->getSqpSettings()
    );
    
    // SQP 设置
    sqp::Settings& settings = leggedInterface_->getSqpSettings();
    settings.maxIterations = 10;           // 最大迭代次数
    settings.absTol = 1e-4;                // 绝对容差
    settings.relTol = 1e-4;                // 相对容差
    settings.nThreads = 4;                 // 并行线程数
}
```

**MPC 循环**:
```cpp
void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
    // 1. 更新观测
    currentObservation_.time = time.toSec();
    currentObservation_.state = stateEstimate_->getState();
    currentObservation_.input.setZero();
    
    // 2. 更新 MPC
    mpcMrtInterface_->updateObservation(currentObservation_);
    
    // 3. 求解 MPC
    mpcMrtInterface_->advanceMpc();
    
    // 4. 获取最优控制
    vector_t optimalState, optimalInput;
    mpcMrtInterface_->evaluatePolicy(
        currentObservation_.time,
        currentObservation_.state,
        optimalState,
        optimalInput,
        bufferRef_
    );
    
    // 5. 传递给 WBC
    wbc_->update(optimalState, optimalInput, ...);
}
```

## 4. 参数调节

### 4.1 预测时域

**推荐值**: 50 步 (1.5秒，dt=0.03秒)

**权衡**:
- 太长: 计算负担大，实时性差
- 太短: 无法预测未来，性能下降

### 4.2 权重矩阵

**状态权重 Q**:
```
基座位置: [10, 10, 100]  # z 更重要 (高度)
基座姿态: [50, 50, 10]   # roll, pitch 更重要
基座速度: [10, 10, 10]
关节速度: [0.1, 0.1, ...]  # 较小 (允许关节运动)
```

**输入权重 R**:
```
接触力: [0.01, 0.01, 0.01, ...]  # 较小 (允许力变化)
关节速度: [1.0, 1.0, ...]        # 较大 (平滑控制)
```

### 4.3 摩擦系数

**推荐值**: μ = 0.6 - 0.8

**考虑因素**:
- 地面材质 (橡胶、冰、草地)
- 安全裕度 (避免滑动)

## 5. 性能优化

### 5.1 并行计算

**多线程**:
```cpp
settings.nThreads = 4;  // 使用 4 个线程
```

**并行策略**:
- 并行计算雅可比矩阵
- 并行求解 QP 子问题
- 并行评估代价函数

### 5.2 自动微分

**C++ AD (Auto Differentiation)**:
```cpp
// 使用 CppAD 自动计算雅可比
auto adWrapper = std::make_unique<CppAdCodegen::CAd>(...);
```

**优势**:
- 精确的雅可比 (无数值误差)
- 快速的计算 (编译时优化)

### 5.3 热启动

**策略**:
```cpp
// 使用上一次的解作为初始猜测
mpcMrtInterface_->setPreviousSolution(previousSolution);
```

**优势**:
- 减少迭代次数
- 提高收敛速度

## 6. 常见问题

**Q: NMPC 求解失败怎么办？**
A: 
1. 检查初始状态是否合理
2. 减小权重矩阵 Q, R
3. 增加最大迭代次数
4. 检查约束是否过于严格

**Q: 如何调节权重矩阵？**
A:
1. 先设置较大的 Q (跟踪参考)
2. 逐渐增加 R (平滑控制)
3. 观察仿真效果，微调

**Q: 预测时域如何选择？**
A:
1. 快速运动 (跑步): 30-50 步
2. 慢速运动 (行走): 50-100 步
3. 考虑计算能力

**Q: 如何处理模型误差？**
A:
1. 增加鲁棒性约束
2. 使用更准确的模型
3. 结合状态估计校正

## 7. 关键论文

1. **MIT Cheetah 3 MPC**
   - "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control"
   - IEEE IROS 2019
   - 核心思想: 凸优化 + 质心动力学

2. **OCS2 Framework**
   - "OCS2: An Open Source Optimization Suite for Optimal Control"
   - IEEE RSS 2022
   - 核心思想: 通用最优控制框架

3. **Perceptive Locomotion**
   - "Perceptive Locomotion through Nonlinear Model Predictive Control"
   - IEEE TRO 2022
   - 核心思想: 结合感知 + NMPC

## 8. 下一步

- 理解 WBC: 阅读 `02_WBC详解.md`
- 理解状态估计: 阅读 `03_状态估计.md`
- 数学基础: 阅读 `04_数学基础.md`
