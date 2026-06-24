#pragma once

#include "Task.h"
#include "HoQp.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace legged_wbc {

/**
 * @brief Robot state representation
 */
struct RobotState {
    Eigen::Vector3d basePosition;      // Base position in world frame
    Eigen::Quaterniond baseOrientation; // Base orientation in world frame
    Eigen::Vector3d baseLinearVelocity; // Base linear velocity in base frame
    Eigen::Vector3d baseAngularVelocity; // Base angular velocity in base frame
    Eigen::VectorXd jointPositions;     // Joint positions
    Eigen::VectorXd jointVelocities;    // Joint velocities
    
    // Contact states for each foot (true = in contact)
    std::vector<bool> contactStates;
    
    RobotState() : jointPositions(12), jointVelocities(12), contactStates(4, false) {
        basePosition.setZero();
        baseOrientation.setIdentity();
        baseLinearVelocity.setZero();
        baseAngularVelocity.setZero();
        jointPositions.setZero();
        jointVelocities.setZero();
    }
};

/**
 * @brief Control command for the robot
 */
struct ControlCommand {
    Eigen::Vector3d baseVelocityCommand;  // Desired base velocity [vx, vy, wz]
    double baseHeightCommand;              // Desired base height
    
    ControlCommand() : baseHeightCommand(0.3) {
        baseVelocityCommand.setZero();
    }
};

/**
 * @brief Whole Body Controller base class
 * 
 * Implements hierarchical task-based whole body control for quadruped robots.
 * Tasks are prioritized and solved in a hierarchical manner.
 */
class WbcBase {
public:
    using Ptr = std::shared_ptr<WbcBase>;
    using ConstPtr = std::shared_ptr<const WbcBase>;
    
    WbcBase();
    virtual ~WbcBase() = default;
    
    /**
     * @brief Update the controller and compute joint torques
     * @param state Current robot state
     * @param command Control command
     * @param dt Control time step
     * @return Joint torque commands (12 DOF)
     */
    virtual Eigen::VectorXd update(const RobotState& state, 
                                    const ControlCommand& command, 
                                    double dt);
    
    /**
     * @brief Get the number of decision variables
     * @return Number of decision variables
     */
    size_t getNumDecisionVariables() const { return numDecisionVars_; }
    
    /**
     * @brief Get the number of joints
     * @return Number of joints
     */
    size_t getNumJoints() const { return numJoints_; }

protected:
    size_t numJoints_;           // Number of joints (12 for quadruped)
    size_t numDecisionVars_;     // Number of decision variables
    
    // Robot parameters
    double robotMass_;           // Total robot mass
    Eigen::Matrix3d baseInertia_; // Base inertia tensor
    
    // Task weights
    double baseAccelWeight_;
    double swingLegWeight_;
    double contactForceWeight_;
    double torqueMinWeight_;
    
    /**
     * @brief Formulate floating base EOM task
     * @param state Current robot state
     * @return Task for floating base dynamics
     */
    virtual Task formulateFloatingBaseEomTask(const RobotState& state);
    
    /**
     * @brief Formulate no contact motion task
     * @param state Current robot state
     * @return Task to prevent contact point motion
     */
    virtual Task formulateNoContactMotionTask(const RobotState& state);
    
    /**
     * @brief Formulate friction cone task
     * @param state Current robot state
     * @return Task for friction cone constraints
     */
    virtual Task formulateFrictionConeTask(const RobotState& state);
    
    /**
     * @brief Formulate base acceleration tracking task
     * @param state Current robot state
     * @param command Control command
     * @param dt Time step
     * @return Task for base acceleration tracking
     */
    virtual Task formulateBaseAccelTask(const RobotState& state,
                                         const ControlCommand& command,
                                         double dt);
    
    /**
     * @brief Formulate swing leg tracking task
     * @param state Current robot state
     * @param command Control command
     * @param dt Time step
     * @return Task for swing leg tracking
     */
    virtual Task formulateSwingLegTask(const RobotState& state,
                                        const ControlCommand& command,
                                        double dt);
    
    /**
     * @brief Formulate torque minimization task
     * @return Task for torque minimization
     */
    virtual Task formulateTorqueMinTask();
    
    /**
     * @brief Solve hierarchical QP problem
     * @param tasks Vector of tasks in priority order
     * @param solution Output solution
     * @return true if solved successfully
     */
    bool solveHierarchicalQp(const std::vector<Task>& tasks, 
                              Eigen::VectorXd& solution);
};

}  // namespace legged_wbc
