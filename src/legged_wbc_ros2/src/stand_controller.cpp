#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <cmath>
#include "legged_wbc_ros2/gait_generator.hpp"

using namespace std::chrono_literals;

class StandController : public rclcpp::Node {
public:
    StandController() : Node("stand_controller") {
        // Declare parameters
        this->declare_parameter("control_frequency", 200.0);
        this->declare_parameter("kp", 15.0);
        this->declare_parameter("kd", 12.0);
        this->declare_parameter("robot_mass", 13.5);
        this->declare_parameter("ramp_time", 5.0);  // Seconds to ramp up to target pose
        
        double freq = this->get_parameter("control_frequency").as_double();
        kp_ = this->get_parameter("kp").as_double();
        kd_ = this->get_parameter("kd").as_double();
        double mass = this->get_parameter("robot_mass").as_double();
        ramp_time_ = this->get_parameter("ramp_time").as_double();
        
        // Joint names matching A1 URDF - ROS 1 order: LF, LH, RF, RH
        joint_names_ = {
            "LF_HAA", "LF_HFE", "LF_KFE",
            "LH_HAA", "LH_HFE", "LH_KFE",
            "RF_HAA", "RF_HFE", "RF_KFE",
            "RH_HAA", "RH_HFE", "RH_KFE"
        };
        
        // A1 default standing pose (radians) - from ROS 1 task.info
        // Order: LF, LH, RF, RH
        target_pose_ = {-0.20,  0.72, -1.44,   // LF
                        -0.20,  0.72, -1.44,   // LH
                         0.20,  0.72, -1.44,   // RF
                         0.20,  0.72, -1.44};  // RH
        
        // Gravity compensation torques (approximate)
        double leg_torque = mass * 9.81 / 4.0 * 0.15;  // 力臂 0.15m
        gravity_comp_ = {
            0.0, leg_torque, -leg_torque * 0.5,
            0.0, leg_torque, -leg_torque * 0.5,
            0.0, leg_torque, -leg_torque * 0.5,
            0.0, leg_torque, -leg_torque * 0.5
        };
        
        // Current state
        joint_positions_ = Eigen::VectorXd::Zero(12);
        joint_velocities_ = Eigen::VectorXd::Zero(12);
        state_received_ = false;
        initial_pose_set_ = false;
        start_time_ = this->now();
        
        // 初始化步态生成器
        gait_generator_ = std::make_shared<legged_controllers::GaitGenerator>();
        walking_mode_ = false;
        cmd_vel_ = geometry_msgs::msg::Twist();
        
        // Publishers
        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_effort_controller/commands", 10);
        
        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&StandController::jointStateCallback, this, std::placeholders::_1));
        
        // 速度指令订阅器
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&StandController::cmdVelCallback, this, std::placeholders::_1));
        
        // Timer
        auto period = std::chrono::duration<double>(1.0 / freq);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&StandController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Stand Controller initialized (freq=%.0f Hz, Kp=%.1f, Kd=%.1f, mass=%.1f kg, ramp=%.1fs)", 
            freq, kp_, kd_, mass, ramp_time_);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Map joint names to indices
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_names_.size(); ++j) {
                if (msg->name[i] == joint_names_[j]) {
                    if (i < msg->position.size())
                        joint_positions_[j] = msg->position[i];
                    if (i < msg->velocity.size())
                        joint_velocities_[j] = msg->velocity[i];
                    break;
                }
            }
        }
        
        // Set initial pose on first received state
        if (!initial_pose_set_) {
            initial_pose_ = joint_positions_;
            initial_pose_set_ = true;
            start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Initial pose captured, starting ramp-up");
        }
        
        state_received_ = true;
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_vel_ = *msg;
        
        // 根据速度大小切换行走/站立模式
        double speed = std::sqrt(msg->linear.x * msg->linear.x + 
                                 msg->linear.y * msg->linear.y + 
                                 msg->angular.z * msg->angular.z);
        
        // 进入行走模式（移除 pose_ready 检查，但增加稳定时间要求）
        double elapsed = (this->now() - start_time_).seconds();
        if (speed > 0.05 && !walking_mode_ && elapsed > 5.0) {  // 至少稳定 5 秒
            walking_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "Entering walking mode after %.1fs: vx=%.2f, vy=%.2f, omega=%.2f",
                       elapsed, msg->linear.x, msg->linear.y, msg->angular.z);
            gait_generator_->reset();
        } else if (speed < 0.03 && walking_mode_) {
            walking_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "Returning to standing mode");
        }
    }
    
    void controlLoop() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!state_received_ || !initial_pose_set_) {
            return;
        }
        
        // Compute ramp progress (0 to 1)
        double elapsed = (this->now() - start_time_).seconds();
        double ramp_progress = std::min(1.0, elapsed / ramp_time_);
        
        // Smooth ramp using cubic easing
        double smooth_progress = ramp_progress * ramp_progress * (3.0 - 2.0 * ramp_progress);
        
        // 根据模式选择目标姿态
        Eigen::VectorXd current_target(12);
        if (walking_mode_) {
            // 行走模式：先过渡到默认站立姿态，再使用步态生成器
            if (ramp_progress < 1.0) {
                // 还在过渡阶段，使用默认站立姿态
                for (int i = 0; i < 12; ++i) {
                    current_target[i] = initial_pose_[i] + smooth_progress * (target_pose_[i] - initial_pose_[i]);
                }
            } else {
                // 过渡完成，使用步态生成器
                gait_generator_->update(0.005, cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z);
                current_target = gait_generator_->getJointPositions();
            }
            
            // Debug: 每2秒输出一次步态生成器状态
            static int gait_counter = 0;
            if (++gait_counter % 400 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "Gait: vx=%.2f | Ramp: %.0f%% | Joint targets[0:3]: %.2f %.2f %.2f",
                    cmd_vel_.linear.x,
                    ramp_progress * 100.0,
                    current_target[0], current_target[1], current_target[2]);
            }
        } else {
            // 站立模式：插值到默认姿态
            for (int i = 0; i < 12; ++i) {
                current_target[i] = initial_pose_[i] + smooth_progress * (target_pose_[i] - initial_pose_[i]);
            }
        }
        
        // PD control with gravity compensation
        Eigen::VectorXd torques(12);
        for (int i = 0; i < 12; ++i) {
            double pos_error = current_target[i] - joint_positions_[i];
            double vel_error = -joint_velocities_[i];
            torques[i] = kp_ * pos_error + kd_ * vel_error + gravity_comp_[i];
            
            // Clamp torque
            torques[i] = std::clamp(torques[i], -33.5, 33.5);
        }
        
        // Publish
        std_msgs::msg::Float64MultiArray effort_cmd;
        effort_cmd.data.resize(12);
        for (int i = 0; i < 12; ++i) {
            effort_cmd.data[i] = torques[i];
        }
        effort_pub_->publish(effort_cmd);
        
        // Debug output every 2 seconds
        static int counter = 0;
        if (++counter % 400 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Ramp: %.0f%% | Pos err[1,2,5]: %.2f %.2f %.2f | Torques[1,2,5]: %.1f %.1f %.1f", 
                ramp_progress * 100.0,
                current_target[1] - joint_positions_[1],
                current_target[2] - joint_positions_[2],
                current_target[5] - joint_positions_[5],
                torques[1], torques[2], torques[5]);
        }
    }
    
    // Parameters
    double kp_, kd_;
    double ramp_time_;
    
    // Joint names
    std::vector<std::string> joint_names_;
    std::vector<double> target_pose_;
    std::vector<double> gravity_comp_;
    
    // State
    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    Eigen::VectorXd initial_pose_;
    bool state_received_;
    bool initial_pose_set_;
    rclcpp::Time start_time_;
    
    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // 步态生成器
    std::shared_ptr<legged_controllers::GaitGenerator> gait_generator_;
    geometry_msgs::msg::Twist cmd_vel_;
    bool walking_mode_;
    
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandController>());
    rclcpp::shutdown();
    return 0;
}
