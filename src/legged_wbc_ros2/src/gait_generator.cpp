#include "legged_wbc_ros2/gait_generator.hpp"
#include <iostream>

namespace legged_controllers {

GaitGenerator::GaitGenerator()
    : swing_height_(0.08)      // 抬腿高度 8cm
    , phase_(0.0)
    , vx_(0.0)
    , vy_(0.0)
    , omega_(0.0)
{
    // 初始化关节位置为默认站立姿态
    joint_positions_ = Eigen::VectorXd::Zero(12);
    joint_velocities_ = Eigen::VectorXd::Zero(12);
    
    // 默认站立时足端位置（相对于髋关节）
    // 根据A1的尺寸：大腿0.2m，小腿0.2m，髋关节偏移0.0838m
    // 站立时足端应该在髋关节下方约0.3m处
    default_foot_pos_lf_ = Eigen::Vector3d(0.0,  hip_offset_, -0.3);
    default_foot_pos_rf_ = Eigen::Vector3d(0.0, -hip_offset_, -0.3);
    default_foot_pos_lh_ = Eigen::Vector3d(0.0,  hip_offset_, -0.3);
    default_foot_pos_rh_ = Eigen::Vector3d(0.0, -hip_offset_, -0.3);
    
    // 初始化接触标志为全部接触（站立）
    contact_flags_ = {true, true, true, true};
    
    // 定义 trot 步态（与 ROS 1 保持一致）
    // trot: LF_RH (9) -> RF_LH (6) -> LF_RH (9)
    // 相位: 0.0 -> 0.5 -> 1.0
    trot_gait_.duration = 0.6;  // 步态周期 0.6s
    trot_gait_.eventPhases = {0.5};  // 在相位0.5切换模式
    trot_gait_.modeSequence = {LF_RH, RF_LH};  // 对角腿交替
    
    // 计算初始关节角度 (顺序: LF, LH, RF, RH)
    joint_positions_.segment<3>(0) = inverseKinematics(default_foot_pos_lf_, true, true);
    joint_positions_.segment<3>(3) = inverseKinematics(default_foot_pos_lh_, true, false);
    joint_positions_.segment<3>(6) = inverseKinematics(default_foot_pos_rf_, false, true);
    joint_positions_.segment<3>(9) = inverseKinematics(default_foot_pos_rh_, false, false);
}

void GaitGenerator::update(double dt, double vx, double vy, double omega) {
    vx_ = vx;
    vy_ = vy;
    omega_ = omega;
    
    // 更新相位
    phase_ += dt / trot_gait_.duration;
    if (phase_ >= 1.0) {
        phase_ -= 1.0;
    }
    
    // 获取当前模式
    size_t current_mode = getModeFromPhase(phase_, trot_gait_);
    contact_flags_ = modeNumber2StanceLeg(current_mode);
    
    // Debug: 输出相位和接触标志
    static int phase_counter = 0;
    if (++phase_counter % 100 == 0) {
        std::cout << "Phase: " << phase_ << " | Mode: " << current_mode 
                  << " | Contacts: [" << contact_flags_[0] << "," << contact_flags_[1] 
                  << "," << contact_flags_[2] << "," << contact_flags_[3] << "]" << std::endl;
    }
    
    // 计算各腿的足端位置
    // 输出顺序: LF, LH, RF, RH (与 ROS 1 一致)
    Eigen::Vector3d foot_lf = computeFootTrajectory(phase_, 0, vx, vy, omega, !contact_flags_[0]);
    Eigen::Vector3d foot_lh = computeFootTrajectory(phase_, 2, vx, vy, omega, !contact_flags_[2]);
    Eigen::Vector3d foot_rf = computeFootTrajectory(phase_, 1, vx, vy, omega, !contact_flags_[1]);
    Eigen::Vector3d foot_rh = computeFootTrajectory(phase_, 3, vx, vy, omega, !contact_flags_[3]);
    
    // 使用逆运动学计算关节角度 (顺序: LF, LH, RF, RH)
    joint_positions_.segment<3>(0) = inverseKinematics(foot_lf, true, true);   // LF -> index 0,1,2
    joint_positions_.segment<3>(3) = inverseKinematics(foot_lh, true, false);  // LH -> index 3,4,5
    joint_positions_.segment<3>(6) = inverseKinematics(foot_rf, false, true);  // RF -> index 6,7,8
    joint_positions_.segment<3>(9) = inverseKinematics(foot_rh, false, false); // RH -> index 9,10,11
    
    // 计算关节速度（简化：使用差分）
    joint_velocities_ = Eigen::VectorXd::Zero(12);
}

Eigen::VectorXd GaitGenerator::getJointPositions() const {
    return joint_positions_;
}

Eigen::VectorXd GaitGenerator::getJointVelocities() const {
    return joint_velocities_;
}

contact_flag_t GaitGenerator::getContactFlags() const {
    return contact_flags_;
}

void GaitGenerator::reset() {
    phase_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
    omega_ = 0.0;
    contact_flags_ = {true, true, true, true};
    joint_positions_.setZero();
    joint_velocities_.setZero();
    
    // 重新计算初始姿态 (顺序: LF, LH, RF, RH)
    joint_positions_.segment<3>(0) = inverseKinematics(default_foot_pos_lf_, true, true);
    joint_positions_.segment<3>(3) = inverseKinematics(default_foot_pos_lh_, true, false);
    joint_positions_.segment<3>(6) = inverseKinematics(default_foot_pos_rf_, false, true);
    joint_positions_.segment<3>(9) = inverseKinematics(default_foot_pos_rh_, false, false);
}

Eigen::Vector3d GaitGenerator::inverseKinematics(const Eigen::Vector3d& foot_pos, bool is_left, bool is_front) {
    double x = foot_pos.x();
    double y = foot_pos.y();
    double z = foot_pos.z();
    
    // Debug: 输出足端位置
    static int ik_counter = 0;
    if (++ik_counter % 200 == 0) {
        std::cout << "IK input: foot_pos=[" << x << ", " << y << ", " << z << "]" << std::endl;
    }
    
    // HAA关节角度（髋关节外展/内收）
    // 对于左腿，y > 0；对于右腿，y < 0
    double y_hip = is_left ? (y - hip_offset_) : (y + hip_offset_);
    double len_yz = std::sqrt(y_hip * y_hip + z * z);
    double haa = std::atan2(y_hip, -z);
    
    // HFE和KFE关节角度（大腿和小腿在X-Z平面的角度）
    // 使用余弦定理计算膝关节角度
    double len_xz = std::sqrt(x * x + len_yz * len_yz);
    
    // 检查是否超出工作空间
    if (len_xz > thigh_length_ + calf_length_ - 0.01) {
        len_xz = thigh_length_ + calf_length_ - 0.01;
    }
    if (len_xz < std::abs(thigh_length_ - calf_length_) + 0.01) {
        len_xz = std::abs(thigh_length_ - calf_length_) + 0.01;
    }
    
    // 膝关节角度（使用余弦定理）
    // 注意：URDF 中 KFE 关节的零位置是腿伸直，弯曲时角度为负
    double cos_kfe = (thigh_length_ * thigh_length_ + calf_length_ * calf_length_ - len_xz * len_xz) / 
                     (2 * thigh_length_ * calf_length_);
    cos_kfe = std::max(-1.0, std::min(1.0, cos_kfe));
    double kfe = -(M_PI - std::acos(cos_kfe));  // 取反以匹配 URDF 符号约定
    
    // 髋关节角度
    // alpha 是足端相对于髋关节的角度（在X-Z平面）
    // 对于站立姿态，足端在髋关节下方，alpha 应该是负值
    double alpha = std::atan2(-x, len_yz);  // 注意：这里使用 -x，因为大腿向前摆动时 x > 0
    double beta = std::acos((thigh_length_ * thigh_length_ + len_xz * len_xz - calf_length_ * calf_length_) / 
                            (2 * thigh_length_ * len_xz));
    double hfe = -(alpha - beta);  // 取反以匹配 URDF 符号约定（正方向为大腿向前）
    
    // Debug: 输出关节角度
    if (ik_counter % 200 == 0) {
        std::cout << "IK output: haa=" << haa << ", hfe=" << hfe << ", kfe=" << kfe << std::endl;
    }
    
    return Eigen::Vector3d(haa, hfe, kfe);
}

Eigen::Vector3d GaitGenerator::computeFootTrajectory(double phase, int leg_index, double vx, double vy, double omega, bool is_swing) {
    // 确定默认足端位置
    Eigen::Vector3d default_pos;
    switch (leg_index) {
        case 0: default_pos = default_foot_pos_lf_; break;
        case 1: default_pos = default_foot_pos_rf_; break;
        case 2: default_pos = default_foot_pos_lh_; break;
        case 3: default_pos = default_foot_pos_rh_; break;
        default: default_pos = default_foot_pos_lf_; break;
    }
    
    Eigen::Vector3d foot_pos = default_pos;
    
    // 根据速度调整步长
    double speed = std::sqrt(vx * vx + vy * vy);
    double step_length = std::min(speed * trot_gait_.duration * 0.5, 0.15);  // 最大步长15cm
    
    if (is_swing) {
        // 摆动相：抬腿并前移
        // 根据 trot 步态定义：
        // - phase < 0.5: Mode 9 (LF_RH 支撑), RF 和 LH 摆动
        // - phase >= 0.5: Mode 6 (RF_LH 支撑), LF 和 RH 摆动
        double swing_phase;
        if (phase < 0.5) {
            // 第一个半周期：RF和LH摆动
            if (leg_index == 1 || leg_index == 2) {
                swing_phase = phase / 0.5;  // 归一化到[0,1]
            } else {
                swing_phase = 0.0;  // 不应该到达这里（支撑相腿不应该调用此分支）
            }
        } else {
            // 第二个半周期：LF和RH摆动
            if (leg_index == 0 || leg_index == 3) {
                swing_phase = (phase - 0.5) / 0.5;  // 归一化到[0,1]
            } else {
                swing_phase = 0.0;  // 不应该到达这里（支撑相腿不应该调用此分支）
            }
        }
        
        if (swing_phase > 0.0) {
            // X方向：从-step_length/2到+step_length/2
            foot_pos.x() = default_pos.x() + step_length * (swing_phase - 0.5);
            
            // Y方向：保持为0（直线行走）
            foot_pos.y() = default_pos.y();
            
            // Z方向：使用三次样条抬腿
            foot_pos.z() = default_pos.z() + computeSwingHeight(swing_phase);
        }
    } else {
        // 支撑相：向后滑动
        // 根据 trot 步态定义：
        // - phase < 0.5: Mode 9 (LF_RH 支撑)
        // - phase >= 0.5: Mode 6 (RF_LH 支撑)
        double stance_phase;
        if (phase < 0.5) {
            // 第一个半周期：LF和RH支撑
            if (leg_index == 0 || leg_index == 3) {
                stance_phase = phase / 0.5;  // 归一化到[0,1]
            } else {
                stance_phase = 0.5;  // 不应该到达这里（摆动相腿不应该调用此分支）
            }
        } else {
            // 第二个半周期：RF和LH支撑
            if (leg_index == 1 || leg_index == 2) {
                stance_phase = (phase - 0.5) / 0.5;  // 归一化到[0,1]
            } else {
                stance_phase = 0.5;  // 不应该到达这里（摆动相腿不应该调用此分支）
            }
        }
        
        if (stance_phase < 1.0) {
            // X方向：从+step_length/2到-step_length/2（向后滑动）
            foot_pos.x() = default_pos.x() + step_length * (0.5 - stance_phase);
            
            // Y方向：保持为0
            foot_pos.y() = default_pos.y();
            
            // Z方向：保持为0（接触地面）
            foot_pos.z() = default_pos.z();
        }
    }
    
    return foot_pos;
}

double GaitGenerator::computeSwingHeight(double swing_phase) {
    // 使用三次样条规划Z方向高度
    // 在swing_phase=0.5时达到最大高度
    if (swing_phase < 0.5) {
        // 上升阶段
        double t = swing_phase / 0.5;  // 归一化到[0,1]
        return swing_height_ * (3 * t * t - 2 * t * t * t);
    } else {
        // 下降阶段
        double t = (swing_phase - 0.5) / 0.5;  // 归一化到[0,1]
        return swing_height_ * (1 - (3 * t * t - 2 * t * t * t));
    }
}

} // namespace legged_controllers
