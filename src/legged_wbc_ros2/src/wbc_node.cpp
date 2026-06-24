#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "legged_wbc_ros2/WbcBase.h"

using namespace std::chrono_literals;

class WbcNode : public rclcpp::Node {
public:
    WbcNode() : Node("legged_wbc_node") {
        // Initialize WBC
        wbc_ = std::make_shared<legged_wbc::WbcBase>();
        
        // Publishers
        joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/robot/joint_commands", 10);
        
        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/robot/joint_states", 10,
            std::bind(&WbcNode::jointStateCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odom", 10,
            std::bind(&WbcNode::odomCallback, this, std::placeholders::_1));
        
        // Control command subscriber
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/robot/cmd_vel", 10,
            std::bind(&WbcNode::cmdVelCallback, this, std::placeholders::_1));
        
        // Timer for control loop (100 Hz)
        control_timer_ = this->create_wall_timer(
            10ms, std::bind(&WbcNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "WBC Node initialized");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Update joint positions and velocities
        for (size_t i = 0; i < msg->position.size() && i < robot_state_.jointPositions.size(); ++i) {
            robot_state_.jointPositions[i] = msg->position[i];
        }
        for (size_t i = 0; i < msg->velocity.size() && i < robot_state_.jointVelocities.size(); ++i) {
            robot_state_.jointVelocities[i] = msg->velocity[i];
        }
        
        joint_state_received_ = true;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Update base state from odometry
        robot_state_.basePosition = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        
        robot_state_.baseOrientation = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        
        robot_state_.baseLinearVelocity = Eigen::Vector3d(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );
        
        robot_state_.baseAngularVelocity = Eigen::Vector3d(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z
        );
        
        odom_received_ = true;
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Update control command
        command_.baseVelocityCommand = Eigen::Vector3d(
            msg->linear.x,
            msg->linear.y,
            msg->angular.z
        );
    }
    
    void controlLoop() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!joint_state_received_ || !odom_received_) {
            return;
        }
        
        // Set contact states (simplified: assume all feet in contact for standing)
        robot_state_.contactStates = {true, true, true, true};
        
        // Update WBC
        double dt = 0.01;  // 100 Hz
        Eigen::VectorXd jointTorques = wbc_->update(robot_state_, command_, dt);
        
        // Convert torques to joint positions (impedance control)
        // For now, just send zero positions with gravity compensation
        sensor_msgs::msg::JointState jointCmd;
        jointCmd.header.stamp = this->now();
        jointCmd.name = {"hip_l", "thigh_l", "calf_l",
                         "hip_r", "thigh_r", "calf_r",
                         "hip_l2", "thigh_l2", "calf_l2",
                         "hip_r2", "thigh_r2", "calf_r2"};
        
        jointCmd.position.resize(12, 0.0);
        jointCmd.velocity.resize(12, 0.0);
        jointCmd.effort.resize(12);
        
        for (int i = 0; i < 12; ++i) {
            jointCmd.effort[i] = jointTorques[i];
        }
        
        joint_cmd_pub_->publish(jointCmd);
        
        // Debug output
        static int counter = 0;
        if (++counter % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Base height: %.3f m", 
                       robot_state_.basePosition.z());
        }
    }
    
    // WBC controller
    legged_wbc::WbcBase::Ptr wbc_;
    
    // Robot state
    legged_wbc::RobotState robot_state_;
    legged_wbc::ControlCommand command_;
    
    // ROS interfaces
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State flags
    bool joint_state_received_ = false;
    bool odom_received_ = false;
    
    // Mutex for thread safety
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WbcNode>());
    rclcpp::shutdown();
    return 0;
}
