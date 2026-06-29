#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace legged_controllers {

/**
 * @brief 接触模式枚举（与 ROS 1 保持一致）
 */
enum ModeNumber {
  FLY = 0,
  RH = 1,
  LH = 2,
  LH_RH = 3,
  RF = 4,
  RF_RH = 5,
  RF_LH = 6,
  RF_LH_RH = 7,
  LF = 8,
  LF_RH = 9,
  LF_LH = 10,
  LF_LH_RH = 11,
  LF_RF = 12,
  LF_RF_RH = 13,
  LF_RF_LH = 14,
  STANCE = 15,
};

/**
 * @brief 接触标志类型 {LF, RF, LH, RH}
 */
using contact_flag_t = std::array<bool, 4>;

/**
 * @brief 将模式编号转换为接触标志
 */
inline contact_flag_t modeNumber2StanceLeg(size_t modeNumber) {
  contact_flag_t stanceLegs;
  switch (modeNumber) {
    case 0:  stanceLegs = {false, false, false, false}; break;  // FLY
    case 1:  stanceLegs = {false, false, false, true};  break;  // RH
    case 2:  stanceLegs = {false, false, true, false};  break;  // LH
    case 3:  stanceLegs = {false, false, true, true};   break;  // LH_RH
    case 4:  stanceLegs = {false, true, false, false};  break;  // RF
    case 5:  stanceLegs = {false, true, false, true};   break;  // RF_RH
    case 6:  stanceLegs = {false, true, true, false};   break;  // RF_LH
    case 7:  stanceLegs = {false, true, true, true};    break;  // RF_LH_RH
    case 8:  stanceLegs = {true, false, false, false};  break;  // LF
    case 9:  stanceLegs = {true, false, false, true};   break;  // LF_RH (trot)
    case 10: stanceLegs = {true, false, true, false};   break;  // LF_LH
    case 11: stanceLegs = {true, false, true, true};    break;  // LF_LH_RH
    case 12: stanceLegs = {true, true, false, false};   break;  // LF_RF
    case 13: stanceLegs = {true, true, false, true};    break;  // LF_RF_RH
    case 14: stanceLegs = {true, true, true, false};    break;  // LF_RF_LH
    case 15: stanceLegs = {true, true, true, true};     break;  // STANCE
    default: stanceLegs = {true, true, true, true};     break;
  }
  return stanceLegs;
}

/**
 * @brief 步态定义（与 ROS 1 保持一致）
 */
struct Gait {
  double duration;                    // 一个步态周期的时间 [s]
  std::vector<double> eventPhases;    // 模式切换的相位点（0-1之间）
  std::vector<size_t> modeSequence;   // 接触模式序列
};

/**
 * @brief 根据相位获取当前模式
 */
inline size_t getModeFromPhase(double phase, const Gait& gait) {
  if (phase < 0.0 || phase >= 1.0) {
    phase = std::fmod(phase, 1.0);
    if (phase < 0.0) phase += 1.0;
  }
  
  for (size_t i = 0; i < gait.eventPhases.size(); ++i) {
    if (phase < gait.eventPhases[i]) {
      return gait.modeSequence[i];
    }
  }
  return gait.modeSequence.back();
}

/**
 * @brief 基于足端轨迹的步态生成器
 * 
 * 实现trot步态，使用逆运动学计算关节角度
 */
class GaitGenerator {
public:
    GaitGenerator();
    
    /**
     * @brief 更新步态状态
     * @param dt 时间步长
     * @param vx 前向速度 [m/s]
     * @param vy 侧向速度 [m/s]
     * @param omega 旋转角速度 [rad/s]
     */
    void update(double dt, double vx, double vy, double omega);
    
    /**
     * @brief 获取当前关节位置目标
     * @return 12维关节位置向量 [LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE]
     */
    Eigen::VectorXd getJointPositions() const;
    
    /**
     * @brief 获取当前关节速度目标
     */
    Eigen::VectorXd getJointVelocities() const;
    
    /**
     * @brief 获取当前接触标志
     */
    contact_flag_t getContactFlags() const;
    
    /**
     * @brief 重置步态状态
     */
    void reset();

private:
    // 机器人参数 (Unitree A1)
    double hip_offset_ = 0.0838;      // 髋关节偏移 [m]
    double thigh_length_ = 0.2;       // 大腿长度 [m]
    double calf_length_ = 0.2;        // 小腿长度 [m]
    
    // 步态参数
    double swing_height_ = 0.08;      // 抬腿高度 [m]
    
    // 步态定义（trot）
    Gait trot_gait_;
    
    // 当前状态
    double phase_;                    // 当前相位 [0, 1]
    double vx_, vy_, omega_;          // 当前速度
    
    // 关节位置
    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    
    // 接触标志
    contact_flag_t contact_flags_;
    
    // 默认站立时足端位置（相对于髋关节）
    Eigen::Vector3d default_foot_pos_lf_;
    Eigen::Vector3d default_foot_pos_rf_;
    Eigen::Vector3d default_foot_pos_lh_;
    Eigen::Vector3d default_foot_pos_rh_;
    
    /**
     * @brief 逆运动学：从足端位置计算关节角度
     */
    Eigen::Vector3d inverseKinematics(const Eigen::Vector3d& foot_pos, bool is_left, bool is_front);
    
    /**
     * @brief 计算单腿的足端轨迹
     */
    Eigen::Vector3d computeFootTrajectory(double phase, int leg_index, double vx, double vy, double omega, bool is_swing);
    
    /**
     * @brief 计算摆动腿的Z方向高度（三次样条）
     */
    double computeSwingHeight(double swing_phase);
};

} // namespace legged_controllers
