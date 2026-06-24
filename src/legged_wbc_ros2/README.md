# legged_wbc_ros2 - ROS 2 版本的全身控制器

这是一个用于四足机器人学习的 ROS 2 全身控制器（Whole Body Control）实现。

## 功能特性

### 核心算法
- **分层二次规划（HoQP）**: 实现多任务优先级优化
- **全身控制（WBC）**: 基于浮动基动力学的全身控制
- **任务层次**:
  1. 浮动基动力学约束（最高优先级）
  2. 无接触运动约束
  3. 摩擦锥约束
  4. 基座加速度跟踪
  5. 摆动腿跟踪
  6. 扭矩最小化（最低优先级）

### ROS 2 接口
- **订阅话题**:
  - `/robot/joint_states` (sensor_msgs/JointState) - 关节状态
  - `/robot/odom` (nav_msgs/Odometry) - 里程计/基座状态
  - `/robot/cmd_vel` (geometry_msgs/Twist) - 速度命令
  
- **发布话题**:
  - `/robot/joint_commands` (sensor_msgs/JointState) - 关节命令（位置+力矩）

## 编译和运行

### 编译
```bash
cd ~/legged_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select legged_wbc_ros2
source install/setup.bash
```

### 运行
```bash
# 直接运行节点
ros2 run legged_wbc_ros2 legged_wbc_node

# 使用 launch 文件
ros2 launch legged_wbc_ros2 wbc.launch.py
```

### 参数配置
```yaml
control_frequency: 100.0  # 控制频率 (Hz)
robot_mass: 20.0          # 机器人质量 (kg)
```

## 代码结构

```
legged_wbc_ros2/
├── include/legged_wbc_ros2/
│   ├── Task.h          # 任务定义（等式/不等式约束）
│   ├── HoQp.h          # 分层 QP 求解器
│   └── WbcBase.h       # 全身控制器基类
├── src/
│   ├── Task.cpp        # 任务实现
│   ├── HoQp.cpp        # HoQP 求解器实现
│   ├── WbcBase.cpp     # WBC 实现
│   └── wbc_node.cpp    # ROS 2 节点
├── launch/
│   └── wbc.launch.py   # Launch 文件
└── config/             # 配置文件目录
```

## 学习要点

### 1. 分层优化原理
HoQP 通过零空间投影实现多任务优先级：
- 高优先级任务先求解
- 低优先级任务在高优先级任务的零空间中优化
- 保证高优先级任务不受影响

### 2. 任务构建
每个任务表示为：`A * x = b`（等式）或 `lower <= A * x <= upper`（不等式）

决策变量：`x = [base_accel(6), contact_forces(12), joint_torques(12)]`

### 3. 与原始 legged_control 的区别
- **简化版**: 去除了 OCS2 和 Pinocchio 依赖，使用纯 Eigen 实现
- **教学目的**: 代码更简洁，易于理解核心算法
- **ROS 2**: 完全基于 ROS 2 架构

## 扩展方向

1. **添加运动学**: 集成 Pinocchio 或 KDL 计算雅可比矩阵
2. **步态生成**: 添加摆动腿轨迹规划
3. **状态估计**: 集成 IMU 和接触检测
4. **仿真集成**: 与 Gazebo 和 ros2_control 集成
5. **实机部署**: 适配真实硬件接口

## 参考资源

- [legged_control (ROS 1)](https://github.com/qiayuanl/legged_control) - 原始实现
- [OCS2](https://github.com/leggedrobotics/ocs2) - 最优控制框架
- 《四足机器人控制算法》- 理论基础

## 许可证

BSD License
