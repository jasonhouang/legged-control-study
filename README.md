# Legged Control ROS2 Port

Unitree A1 四足机器人 ROS 2 Jazzy 仿真项目，基于 ROS 1 legged_control 移植。

## 项目状态

✅ **已完成功能：**
- Gazebo Harmonic 8.11.0 仿真环境
- ros2_control 集成（GazeboSimSystem 插件）
- 初始姿态设置（JointPositionController 插件）
- PD 控制器 + 重力补偿
- 逆运动学 trot 步态生成器
- cmd_vel 速度指令控制
- 机器人稳定站立和行走

## 项目结构

```
src/
├── legged_a1_description/    # A1 URDF 模型描述
│   ├── urdf/
│   │   ├── robot.xacro      # 主 URDF 文件
│   │   ├── leg.xacro        # 腿部定义
│   │   └── const.xacro      # 常量定义
│   └── meshes/              # 3D 模型文件
│
├── legged_simulation/       # 仿真配置
│   ├── launch/
│   │   └── simulation.launch.py  # 主启动文件
│   ├── config/
│   │   └── controllers.yaml      # 控制器配置
│   └── worlds/
│       └── empty.world          # 仿真世界
│
└── legged_wbc_ros2/         # 控制器实现
    ├── src/
    │   ├── stand_controller.cpp    # 站立/行走控制器
    │   ├── gait_generator.cpp      # 步态生成器
    │   ├── HoQp.cpp               # QP 求解器
    │   └── wbc_node.cpp           # WBC 节点
    └── include/
        └── legged_wbc_ros2/
            ├── stand_controller.hpp
            └── gait_generator.hpp
```

## 关键技术

### 1. 仿真环境
- **Gazebo Harmonic 8.11.0**：与 ROS 2 Jazzy 兼容
- **ros2_control**：硬件抽象层
- **JointPositionController**：初始姿态设置

### 2. 控制架构
- **PD 控制器**：位置跟踪 + 速度阻尼
- **重力补偿**：前馈力矩
- **逆运动学**：足端轨迹到关节角度映射
- **Trot 步态**：对角步态，相位控制

### 3. 关键修复
1. **Gazebo 版本冲突**：卸载 Jetty 10.x，保留 Harmonic 8.x
2. **僵尸进程**：清理旧的 robot_state_publisher 进程
3. **仿真暂停**：激活控制器前先恢复仿真运行
4. **pose_ready 检查**：移除关节位置误差检查，允许机器人行走

## 运行方法

### 1. 启动仿真
```bash
cd ~/legged_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch legged_simulation simulation.launch.py
```

### 2. 恢复仿真运行（如果暂停）
```bash
gz service -s /world/empty/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'pause: false'
```

### 3. 激活控制器
```bash
ros2 control switch_controllers \
  --activate joint_state_broadcaster \
  joint_group_effort_controller
```

### 4. 发送行走指令
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 参数配置

### PD 增益（simulation.launch.py）
```python
'kp': 20.0,      # 位置增益
'kd': 2.0,       # 速度增益
'ramp_time': 3.0 # 过渡时间
```

### 步态参数（gait_generator.hpp）
```cpp
step_height_ = 0.05;  // 抬脚高度
step_length_ = 0.1;   // 步长
gait_period_ = 0.8;   // 步态周期
```

## 已知问题

1. **read/write rate 显示 0 Hz**：统计信息初始化问题，不影响实际功能
2. **移动速度较慢**：需要进一步优化步态参数和 PD 增益
3. **关节位置误差**：初始姿态与目标姿态存在偏差，但不影响行走

## 下一步优化

- [ ] 调整步态参数（步频、步幅）
- [ ] 优化 PD 增益
- [ ] 添加地形适应性
- [ ] 实现更复杂的步态（转弯、侧移）
- [ ] 集成状态估计（IMU + 运动学）
- [ ] 添加 MPC 控制器

## 依赖

- ROS 2 Jazzy
- Gazebo Harmonic 8.11.0
- ros2_control
- gz_ros2_control

## 参考资料

- [ROS 1 legged_control](https://github.com/leggedrobotics/legged_control)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [Unitree A1](https://www.unitree.com/a1)

## 许可证

MIT License
