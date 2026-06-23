# 02. WBC (全身控制器) 详解

## 1. 基本原理

### 1.1 什么是 WBC？

**WBC (Whole-Body Controller)** 是处理全身动力学的控制器：

1. **输入**: NMPC 的优化结果 (期望状态、接触力)
2. **决策**: 关节扭矩、接触力
3. **约束**: 动力学、摩擦锥、关节限位
4. **输出**: 关节扭矩命令

$$
NMPC 输出
  ├─ 期望状态 x_des
  ├─ 期望接触力 f_c_des
  └─ 期望关节速度 v_j_des
      ↓
WBC 求解
  ├─ 关节加速度 q̈
  ├─ 接触力 f_c
  └─ 关节扭矩 τ
      ↓
电机执行
$$

### 1.2 为什么需要 WBC？

**NMPC 的局限**:
1. 使用简化模型 (质心动力学)
2. 不考虑关节细节
3. 无法处理完整动力学约束

**WBC 的优势**:
1. 使用完整动力学模型
2. 处理所有关节
3. 分层任务优先级
4. 实时性好 (<1ms)

## 2. 数学建模

### 2.1 决策变量

**WBC 决策变量** (42维):
$$
x_wbc = [q̈, f_c, τ]
        ├─ q̈: 广义坐标加速度 (19维)
        │   ├─ 基座加速度 (6维: 线加速度 + 角加速度)
        │   └─ 关节加速度 (12维)
        ├─ f_c: 接触力 (12维)
        │   └─ 4脚 × 3维力
        └─ τ: 关节扭矩 (12维)
$$

### 2.2 动力学约束

**浮动基动力学**:
$$
M(q) * q̈ + C(q, q̇) * q̇ + g(q) = S * τ + J^T * f_c

其中:
  M(q): 惯量矩阵 (19×19)
  C(q, q̇): 科氏力矩阵 (19×1)
  g(q): 重力向量 (19×1)
  S: 选择矩阵 (19×12)
  J: 雅可比矩阵 (3×19 per foot)
$$

**展开形式**:
$$
┌ M_bb  M_bj ┐ ┌ q̈_b ┐   ┌ h_b ┐   ┌ 0  ┐   ┌ J_b^T ┐
│            │ │     │ + │     │ = │    │ + │       │ * f_c
└ M_jb  M_jj ┘ └ q̈_j ┘   └ h_j ┘   └ S  ┘   └ J_j^T ┘

下标:
  b: 基座 (base)
  j: 关节 (joint)
$$

### 2.3 任务层级

WBC 使用**分层优化**，任务优先级从高到低：

| 优先级 | 任务 | 类型 | 数学表达 |
|--------|------|------|----------|
| 1 | 浮动基动力学 | 等式约束 | `M*q̈ + h = S*τ + J^T*f_c` |
| 2 | 无接触运动 | 等式约束 | `J_foot * q̈ + J̇_foot * q̇ = 0` |
| 3 | 摩擦锥约束 | 不等式约束 | `√(f_x² + f_y²) ≤ μ * f_z` |
| 4 | 基座加速度跟踪 | 等式约束 | `q̈_b = q̈_b_des` |
| 5 | 摆动腿跟踪 | 等式约束 | `ẍ_foot = ẍ_foot_des` |
| 6 | 接触力跟踪 | 等式约束 | `f_c = f_c_des` |
| 7 | 关节扭矩最小化 | 代价函数 | `min τ^T * W * τ` |

## 3. 分层优化 (HQP)

### 3.1 什么是 HQP？

**HQP (Hierarchical Quadratic Programming)** 是分层二次规划：

```
优先级 1: min ||A_1 * x - b_1||²
          s.t. A_1 * x = b_1  (严格满足)

优先级 2: min ||A_2 * x - b_2||²
          s.t. A_1 * x = b_1  (保持优先级 1)
             A_2 * x = b_2  (在零空间)

优先级 3: min ||A_3 * x - b_3||²
          s.t. A_1 * x = b_1
             A_2 * x = b_2
             A_3 * x = b_3  (在零空间)
```

### 3.2 零空间投影

**零空间投影**确保高优先级任务不受影响：

$$
对于优先级 k:
  x_k = x_{k-1} + Z_{k-1} * (A_k * Z_{k-1})^# * (b_k - A_k * x_{k-1})

其中:
  Z_{k-1}: 优先级 1 到 k-1 的零空间投影矩阵
  ^#: 伪逆
$$

**零空间投影矩阵**:
```
Z_0 = I
Z_k = Z_{k-1} * (I - (A_k * Z_{k-1})^# * (A_k * Z_{k-1}))
```

### 3.3 HQP 求解器

**HoQp 类**:
```cpp
class HoQp {
public:
    HoQp(const Task& task, HoQp::Ptr higherPriority);
    
    // 求解 HQP
    vector_t getSolutions() const;
    
private:
    Task task_;                          // 当前任务
    HoQp::Ptr higherPriority_;           // 高优先级 HQP
    matrix_t nullSpaceProjector_;        // 零空间投影矩阵
};
```

**求解过程**:
```cpp
void HoQp::solve() {
    // 1. 获取高优先级解
    vector_t x_higher = higherPriority_->getSolutions();
    
    // 2. 计算零空间投影
    matrix_t Z = computeNullSpaceProjector(higherPriority_);
    
    // 3. 投影当前任务到零空间
    matrix_t A_proj = task_.A_ * Z;
    vector_t b_proj = task_.b_ - task_.A_ * x_higher;
    
    // 4. 求解投影后的 QP
    vector_t x_null = solveQP(A_proj, b_proj);
    
    // 5. 组合解
    solution_ = x_higher + Z * x_null;
}
```

## 4. 代码分析

### 4.1 WbcBase 类

**文件**: `legged_wbc/src/WbcBase.cpp`

**主要功能**:
1. 构建分层优化问题
2. 求解 HQP
3. 返回关节扭矩

**关键函数**:

```cpp
vector_t WbcBase::update(const vector_t& stateDesired, 
                         const vector_t& inputDesired,
                         const vector_t& rbdStateMeasured,
                         size_t mode,
                         scalar_t period) {
    // 1. 更新测量状态
    updateMeasured(rbdStateMeasured);
    
    // 2. 更新期望状态
    updateDesired(stateDesired, inputDesired);
    
    // 3. 构建任务
    Task floatingBaseTask = formulateFloatingBaseEomTask();
    Task noContactMotionTask = formulateNoContactMotionTask();
    Task frictionConeTask = formulateFrictionConeTask();
    Task baseAccelTask = formulateBaseAccelTask(stateDesired, inputDesired, period);
    Task swingLegTask = formulateSwingLegTask();
    Task contactForceTask = formulateContactForceTask(inputDesired);
    
    // 4. 构建 HQP (分层)
    HoQp::Ptr hqp = std::make_shared<HoQp>(floatingBaseTask);
    hqp = std::make_shared<HoQp>(noContactMotionTask, hqp);
    hqp = std::make_shared<HoQp>(frictionConeTask, hqp);
    hqp = std::make_shared<HoQp>(baseAccelTask, hqp);
    hqp = std::make_shared<HoQp>(swingLegTask, hqp);
    hqp = std::make_shared<HoQp>(contactForceTask, hqp);
    
    // 5. 求解
    vector_t solution = hqp->getSolutions();
    
    // 6. 提取关节扭矩
    vector_t torque = solution.tail(numJoints_);
    
    return torque;
}
```

### 4.2 任务构建

**浮动基动力学任务**:
```cpp
Task WbcBase::formulateFloatingBaseEomTask() {
    Task task(numDecisionVars_);
    
    // 动力学方程: M*q̈ + h = S*τ + J^T*f_c
    // 重写为: [M, -J^T, -S] * [q̈; f_c; τ] = -h
    
    matrix_t A(6, numDecisionVars_);
    vector_t b(6);
    
    // 提取基座部分
    A.block(0, 0, 6, 18) = pinocchioInterface_.getM().topRows(6);
    A.block(0, 18, 6, 12) = -pinocchioInterface_.getJ().transpose().topRows(6);
    A.block(0, 30, 6, 12) = -pinocchioInterface_.getS();
    
    b = -pinocchioInterface_.getH().head(6);
    
    task.A_ = A;
    task.b_ = b;
    task.d_ = VectorXd::Zero(6);  // 严格等式约束
    
    return task;
}
```

**无接触运动任务**:
```cpp
Task WbcBase::formulateNoContactMotionTask() {
    Task task(numDecisionVars_);
    
    // 支撑脚加速度为 0: J_foot * q̈ + J̇_foot * q̇ = 0
    // 重写为: [J_foot, 0, 0] * [q̈; f_c; τ] = -J̇_foot * q̇
    
    size_t numContacts = contactFlag_.count();
    matrix_t A(3 * numContacts, numDecisionVars_);
    vector_t b(3 * numContacts);
    
    size_t row = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (contactFlag_[i]) {
            A.block(row, 0, 3, 18) = j_.block(3 * i, 0, 3, 18);
            A.block(row, 18, 3, 24) = MatrixXd::Zero(3, 24);
            b.segment(row, 3) = -dj_.block(3 * i, 0, 3, 18) * vMeasured_;
            row += 3;
        }
    }
    
    task.A_ = A;
    task.b_ = b;
    task.d_ = VectorXd::Zero(3 * numContacts);
    
    return task;
}
```

**摆动腿跟踪任务**:
```cpp
Task WbcBase::formulateSwingLegTask() {
    Task task(numDecisionVars_);
    
    // 摆动腿加速度跟踪: ẍ_foot = ẍ_foot_des
    // ẍ_foot = J_foot * q̈ + J̇_foot * q̇
    // 重写为: [J_foot, 0, 0] * [q̈; f_c; τ] = ẍ_foot_des - J̇_foot * q̇
    
    size_t numSwingLegs = 4 - contactFlag_.count();
    matrix_t A(3 * numSwingLegs, numDecisionVars_);
    vector_t b(3 * numSwingLegs);
    
    size_t row = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (!contactFlag_[i]) {
            A.block(row, 0, 3, 18) = j_.block(3 * i, 0, 3, 18);
            A.block(row, 18, 3, 24) = MatrixXd::Zero(3, 24);
            
            // 期望加速度 (PD 控制)
            Vector3d ẍ_des = swingKp_ * (eeKinematics_->getPosition(i) - eeDesired_)
                           + swingKd_ * (eeKinematics_->getVelocity(i) - eeVelDesired_)
                           + ẍ_foot_desired_;
            
            b.segment(row, 3) = ẍ_des - dj_.block(3 * i, 0, 3, 18) * vMeasured_;
            row += 3;
        }
    }
    
    task.A_ = A;
    task.b_ = b;
    task.d_ = VectorXd::Ones(3 * numSwingLegs) * 1e6;  // 软约束
    
    return task;
}
```

### 4.3 Task 类

**文件**: `legged_wbc/include/legged_wbc/Task.h`

**定义**:
```cpp
struct Task {
    matrix_t A_;      // 等式约束矩阵
    vector_t b_;      // 等式约束向量
    vector_t d_;      // 不等式约束上界
    matrix_t C_;      // 不等式约束矩阵
    vector_t c_;      // 不等式约束下界
    
    Task(size_t numVars) {
        A_ = MatrixXd::Zero(0, numVars);
        b_ = VectorXd::Zero(0);
        d_ = VectorXd::Zero(0);
        C_ = MatrixXd::Zero(0, numVars);
        c_ = VectorXd::Zero(0);
    }
};
```

## 5. 参数调节

### 5.1 摆动腿 PD 增益

**推荐值**:
```yaml
swingLeg:
  kp: 1000.0  # 位置增益
  kd: 100.0   # 速度增益
```

**调节方法**:
1. 先设置较小的 kp, kd
2. 逐渐增加，观察跟踪效果
3. 避免过大导致震荡

### 5.2 接触力跟踪权重

**推荐值**:
```yaml
contactForce:
  weight: [100.0, 100.0, 100.0, ...]  # 12维
```

**调节方法**:
1. 权重越大，跟踪越精确
2. 但可能与其他任务冲突
3. 需要权衡

### 5.3 摩擦系数

**推荐值**: μ = 0.6 - 0.8

**考虑因素**:
- 地面材质
- 安全裕度
- 运动类型 (行走、跑步)

## 6. 性能优化

### 6.1 计算效率

**优化策略**:
1. **稀疏矩阵**: 利用 M, J 的稀疏性
2. **并行计算**: 多线程求解 QP
3. **预计算**: 缓存不变量 (如 M, J)

### 6.2 数值稳定性

**技巧**:
1. **正则化**: 添加小量避免奇异
```cpp
A_ += 1e-6 * MatrixXd::Identity(A_.rows(), A_.cols());
```

2. **缩放**: 归一化约束
```cpp
A_ /= A_.norm();
b_ /= b_.norm();
```

3. **伪逆**: 使用 SVD 分解
```cpp
VectorXd x = A_.bdcSvd(ComputeThinU | ComputeThinV).solve(b_);
```

## 7. 常见问题

**Q: WBC 求解失败怎么办？**
A:
1. 检查动力学模型是否准确
2. 检查约束是否冲突
3. 增加正则化项
4. 减小任务权重

**Q: 摆动腿跟踪不好？**
A:
1. 增加 PD 增益 (kp, kd)
2. 检查参考轨迹是否合理
3. 检查摩擦锥约束是否过紧

**Q: 接触力震荡？**
A:
1. 减小接触力跟踪权重
2. 增加摩擦锥约束的松弛变量
3. 检查接触检测是否准确

**Q: 机器人抖动？**
A:
1. 减小 PD 增益
2. 增加关节扭矩最小化权重
3. 检查 IMU 噪声

## 8. 关键论文

1. **WBC 基础**
   - "Perception-less terrain adaptation through whole body control and hierarchical optimization"
   - IEEE Humanoids 2016
   - 核心思想: 分层优化 + 全身控制

2. **HQO 求解器**
   - "A Hierarchical Quadratic Optimization Framework for Whole-Body Control"
   - IEEE TRO 2020
   - 核心思想: 高效的 HQP 求解

3. **实时 WBC**
   - "Real-Time Whole-Body Control for Quadpedal Locomotion"
   - IEEE IROS 2019
   - 核心思想: 实时 WBC 实现

## 9. 下一步

- 理解状态估计: 阅读 `03_状态估计.md`
- 数学基础: 阅读 `04_数学基础.md`
- 代码结构: 阅读 `05_代码结构.md`
