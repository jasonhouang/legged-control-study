# 四足/人形机器人高级控制算法学习

本项目是一个系统学习四足和人形机器人高级控制算法的工作空间，重点研究 **NMPC（非线性模型预测控制）** 和 **WBC（全身控制器）** 的实现原理。

## 📚 项目目标

- 深入理解业界领先的多足机器人控制架构
- 学习 NMPC + WBC 的分层控制策略
- 掌握状态估计、动力学建模、优化求解等核心技术
- 为自研人形机器人控制系统积累理论基础和实践经验

## 🗂️ 项目结构

```
legged_ws/
├── legged_control/          # 四足机器人控制框架（核心学习对象）
│   ├── legged_controllers/  # 主控制器（NMPC + WBC 集成）
│   ├── legged_wbc/          # 全身控制器（分层二次规划）
│   ├── legged_interface/    # OCS2 接口（优化问题定义）
│   ├── legged_estimation/   # 状态估计（卡尔曼滤波）
│   └── legged_gazebo/       # Gazebo 仿真
│
├── ocs2/                    # 优化控制框架（OCS2）
│   ├── ocs2_mpc/            # MPC 求解器
│   ├── ocs2_ddp/            # 微分动态规划
│   ├── ocs2_sqp/            # 序列二次规划
│   └── ocs2_legged_robot/   # 腿式机器人示例
│
├── pinocchio/               # 刚体动力学库
│   └── 用于计算动力学、雅可比、质心等
│
├── hpp-fcl/                 # 碰撞检测库
│   └── 用于自碰撞避免和地形检测
│
├── ocs2_robotic_assets/     # 机器人 URDF 模型
│   └── ANYmal, Unitree A1 等
│
└── notes/                   # 学习笔记（原创内容）
    ├── 00_总览.md           # 整体架构概览
    ├── 01_NMPC详解.md       # 非线性模型预测控制
    ├── 02_WBC详解.md        # 全身控制器
    ├── 03_状态估计.md        # 卡尔曼滤波与状态估计
    ├── 04_数学基础.md        # 动力学与优化基础
    ├── 05_代码结构.md        # 代码文件阅读指南
    ├── 06_对比分析.md        # 与自研项目的对比
    └── 07_学习路径.md        # 推荐学习顺序
```

## 🔬 核心技术栈

### 控制架构

```
用户输入 (cmd_vel, goal)
    ↓
NMPC (100Hz)
  ├─ 预测时域: 1.5秒
  ├─ 优化变量: 质心动量 + 接触力
  ├─ 约束: 摩擦锥、接触、关节限位
  └─ 求解器: SQP + HPIPM
    ↓
State Estimation (500Hz)
  ├─ IMU 积分
  ├─ 运动学校正
  └─ 卡尔曼滤波
    ↓
WBC (500Hz)
  ├─ 决策变量: [q̈, f_c, τ]
  ├─ 任务层级: 7层（分层二次规划）
  └─ 求解器: HQP
    ↓
Joint Controller (1kHz)
  ├─ 前馈: WBC 扭矩
  └─ 反馈: PD 控制
```

### 关键技术

- **NMPC（Nonlinear Model Predictive Control）**
  - 基于质心动力学的预测控制
  - 处理摩擦锥、接触约束
  - 使用 SQP（序列二次规划）求解

- **WBC（Whole-Body Controller）**
  - 分层任务优先级优化
  - 浮动基动力学约束
  - 使用 HQP（分层二次规划）求解

- **状态估计**
  - 线性卡尔曼滤波
  - 融合 IMU + 运动学
  - 估计基座位置和速度

## 📖 学习笔记

详细的中文学习笔记位于 `notes/` 目录，涵盖：

1. **整体架构** - 分层控制原理和数据流
2. **NMPC 详解** - 优化问题构建、约束处理、求解器
3. **WBC 详解** - 分层优化、任务优先级、HQP 求解
4. **状态估计** - 卡尔曼滤波、IMU 积分、运动学校正
5. **数学基础** - 刚体动力学、优化理论、数值方法
6. **代码分析** - 核心文件解读、参数调节、调试技巧
7. **对比分析** - 与自研项目的差距和改进方向

## 🚀 快速开始

### 环境要求

- Ubuntu 20.04 + ROS Noetic（legged_control 仅支持 ROS 1）
- C++17
- CMake 3.10+

### 编译步骤

```bash
# 1. 安装依赖
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
pip install catkin-tools

# 2. 初始化工作空间
cd legged_ws
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 3. 编译 OCS2（约 10 分钟）
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization

# 4. 编译 legged_control
catkin build legged_controllers legged_unitree_description

# 5. 运行仿真
source devel/setup.bash
export ROBOT_TYPE=a1
roslaunch legged_unitree_description empty_world.launch
```

## 📊 性能指标

| 模块 | 频率 | 求解时间 | 说明 |
|------|------|----------|------|
| NMPC | 100-200Hz | 3-5ms | NUC 11代 |
| WBC | 500Hz | 0.5-1ms | HQP 求解 |
| 状态估计 | 500Hz | <0.1ms | 卡尔曼滤波 |
| 关节控制 | 1kHz | <0.1ms | 电机通信 |

## 🔗 参考资源

### 开源项目

- [legged_control](https://github.com/qiayuanl/legged_control) - 四足机器人控制框架
- [OCS2](https://github.com/leggedrobotics/ocs2) - 优化控制框架
- [Pinocchio](https://github.com/leggedrobotics/pinocchio) - 刚体动力学库

### 论文

1. **MIT Cheetah 3 MPC**
   - "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control"
   - IEEE IROS 2019

2. **WBC 分层优化**
   - "Perception-less terrain adaptation through whole body control and hierarchical optimization"
   - IEEE Humanoids 2016

3. **OCS2 Framework**
   - "OCS2: An Open Source Optimization Suite for Optimal Control"
   - IEEE RSS 2022

### 文档

- [OCS2 官方文档](https://leggedrobotics.github.io/ocs2/)
- [Pinocchio 文档](https://stack-of-tasks.github.io/pinocchio/)
- [ROS Control 文档](http://wiki.ros.org/ros_control)

## 🎯 学习路径

### 阶段 1：基础理解（1 周）
- 阅读 `notes/00_总览.md`
- 理解 NMPC + WBC 的分层架构
- 运行 legged_control 仿真

### 阶段 2：深入学习（1 周）
- 阅读 WBC 和 NMPC 的详细笔记
- 分析核心代码实现
- 理解分层优化和约束处理

### 阶段 3：实践应用（2 周）
- 修改参数，观察效果
- 添加新的步态或约束
- 调试和优化性能

### 阶段 4：移植开发（4 周）
- 将核心算法移植到 ROS 2
- 适配到人形机器人
- 实现 Sim2Real 迁移

## 📝 更新日志

- **2026-06-22**: 初始化项目，克隆 legged_control 和依赖库
- **2026-06-22**: 完成 7 篇学习笔记（3660 行）
- **2026-06-23**: 创建 GitHub 仓库

## 🤝 贡献

本项目主要用于个人学习，但欢迎：
- 纠正笔记中的错误
- 补充新的学习内容
- 分享相关的学习资源

## 📄 许可证

- 学习笔记：MIT License
- 第三方代码：遵循各自项目的许可证
  - legged_control: BSD-3-Clause
  - OCS2: BSD-3-Clause
  - Pinocchio: BSD-2-Clause

## 📧 联系方式

如有问题或建议，欢迎通过 GitHub Issues 交流。

---

**关键词**: 四足机器人, 人形机器人, NMPC, WBC, 模型预测控制, 全身控制, 状态估计, 优化控制, ROS, OCS2
