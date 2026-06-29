#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "legged_wbc_ros2/WbcBase.h"

using namespace std::chrono_literals;

class WbcNode : public rclcpp::Node {
public:
    WbcNode() : Node("legged_wbc_node") {
        // Declare parameters
        this->declare_parameter("control_frequency", 100.0);
        this->declare_parameter("robot_mass", 6.95);  // A1 total mass
        
        double freq = this->get_parameter("control_frequency").as_double();
        double mass = this->get_parameter("robot_mass").as_double();
        
        // Initialize WBC
        wbc_ = std::make_shared<legged_wbc::WbcBase>();
        
        // Joint names matching A1 URDF
        joint_names_ = {
            "LF_HAA", "LF_HFE", "LF_KFE",
            "RF_HAA", "RF_HFE", "RF_KFE",
            "LH_HAA", "LH_HFE", "LH_KFE",
            "RH_HAA", "RH_HFE", "RH_KFE"
        };
        
        // Publishers - effort controller expects Float64MultiArray
        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_effort_controller/commands", 10);
        
        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&WbcNode::jointStateCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/a1/odometry", 10,
            std::bind(&WbcNode::odomCallback, this, std::placeholders::_1));
        
        // Control command subscriber
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&WbcNode::cmdVelCallback, this, std::placeholders::_1));
        
        // Timer for control loop
        auto period = std::chrono::duration<double>(1.0 / freq);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&WbcNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "WBC Node initialized (freq=%.1f Hz, mass=%.2f kg)", freq, mass);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Map joint names to indices
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_names_.size(); ++j) {
                if (msg->name[i] == joint_names_[j]) {
                    if (i < msg->position.size())
                        robot_state_.jointPositions[j] = msg->position[i];
                    if (i < msg->velocity.size())
                        robot_state_.jointVelocities[j] = msg->velocity[i];
                    break;
                }
            }
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
        
        // Publish efforts as Float64MultiArray for effort_controllers
        std_msgs::msg::Float64MultiArray effort_cmd;
        effort_cmd.data.resize(12);
        
        for (int i = 0; i < 12; ++i) {
            effort_cmd.data[i] = jointTorques[i];
        }
        
        effort_pub_->publish(effort_cmd);
        
        // Debug output
        static int counter = 0;
        if (++counter % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Base height: %.3f m | Torques[0:3]: %.2f %.2f %.2f", 
                       robot_state_.basePosition.z(),
                       jointTorques[0], jointTorques[1], jointTorques[2]);
        }
    }
    
    // WBC controller
    legged_wbc::WbcBase::Ptr wbc_;
    
    // Robot state
    legged_wbc::RobotState robot_state_;
    legged_wbc::ControlCommand command_;
    
    // Joint names
    std::vector<std::string> joint_names_;
    
    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
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
