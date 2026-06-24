#include "legged_wbc_ros2/WbcBase.h"
#include <iostream>

namespace legged_wbc {

WbcBase::WbcBase()
    : numJoints_(12),
      numDecisionVars_(18 + 12 + 12),  // [base_accel(6), contact_forces(12), joint_torques(12)]
      robotMass_(20.0),
      baseAccelWeight_(100.0),
      swingLegWeight_(50.0),
      contactForceWeight_(1.0),
      torqueMinWeight_(0.01) {
    
    baseInertia_ = Eigen::Matrix3d::Identity() * 1.0;
}

Eigen::VectorXd WbcBase::update(const RobotState& state, 
                                 const ControlCommand& command, 
                                 double dt) {
    // Build hierarchical task list
    std::vector<Task> tasks;
    
    // Priority 1: Floating base dynamics (equality constraint)
    tasks.push_back(formulateFloatingBaseEomTask(state));
    
    // Priority 2: No contact motion (equality constraint)
    tasks.push_back(formulateNoContactMotionTask(state));
    
    // Priority 3: Friction cone (inequality constraint)
    tasks.push_back(formulateFrictionConeTask(state));
    
    // Priority 4: Base acceleration tracking
    tasks.push_back(formulateBaseAccelTask(state, command, dt));
    
    // Priority 5: Swing leg tracking
    tasks.push_back(formulateSwingLegTask(state, command, dt));
    
    // Priority 6: Torque minimization
    tasks.push_back(formulateTorqueMinTask());
    
    // Solve hierarchical QP
    Eigen::VectorXd solution;
    if (!solveHierarchicalQp(tasks, solution)) {
        std::cerr << "WBC: Failed to solve QP" << std::endl;
        return Eigen::VectorXd::Zero(numJoints_);
    }
    
    // Extract joint torques from solution
    // solution = [base_accel(6), contact_forces(12), joint_torques(12)]
    Eigen::VectorXd jointTorques = solution.tail(numJoints_);
    
    return jointTorques;
}

Task WbcBase::formulateFloatingBaseEomTask(const RobotState& state) {
    // Floating base equation of motion:
    // M * q_ddot + C * q_dot + g = S * tau + J_c^T * F_c
    // 
    // Decision variables: x = [base_accel(6), contact_forces(12), joint_torques(12)]
    // 
    // This is an equality constraint: A * x = b
    
    // For simplicity, we use a linearized version
    // M_base * base_accel = sum(contact_forces) + gravity
    
    Task task(numDecisionVars_, 6);
    
    // Base acceleration part
    Eigen::MatrixXd M_base = Eigen::MatrixXd::Identity(6, 6);
    M_base.block<3, 3>(0, 0) *= robotMass_;
    M_base.block<3, 3>(3, 3) = baseInertia_;
    
    task.A.block(0, 0, 6, 6) = M_base;
    task.A.block(0, 6, 6, 12) = Eigen::MatrixXd::Zero(6, 12);  // Contact forces don't directly affect base
    task.A.block(0, 18, 6, 12) = Eigen::MatrixXd::Zero(6, 12); // Joint torques don't directly affect base
    
    // Right hand side: gravity compensation
    Eigen::VectorXd b = Eigen::VectorXd::Zero(6);
    b(2) = robotMass_ * 9.81;  // Gravity in z direction
    
    task.b = b;
    task.lowerBound = b;
    task.upperBound = b;
    
    return task;
}

Task WbcBase::formulateNoContactMotionTask(const RobotState& state) {
    // For feet in contact, the contact point should not move
    // J_c * q_ddot + J_c_dot * q_dot = 0
    // 
    // This is an equality constraint
    
    // Count number of contact feet
    size_t numContacts = 0;
    for (bool contact : state.contactStates) {
        if (contact) numContacts++;
    }
    
    Task task(numDecisionVars_, 3 * numContacts);
    
    // For each contact foot, add constraint
    size_t row = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (state.contactStates[i]) {
            // Contact Jacobian (simplified)
            // In reality, this should be computed from kinematics
            Eigen::MatrixXd J_c = Eigen::MatrixXd::Zero(3, numDecisionVars_);
            
            // Base acceleration affects contact point
            J_c.block(0, 0, 3, 6) = Eigen::MatrixXd::Identity(3, 6);
            
            task.A.block(row, 0, 3, numDecisionVars_) = J_c;
            task.b.segment(row, 3) = Eigen::VectorXd::Zero(3);
            task.lowerBound.segment(row, 3) = Eigen::VectorXd::Zero(3);
            task.upperBound.segment(row, 3) = Eigen::VectorXd::Zero(3);
            
            row += 3;
        }
    }
    
    return task;
}

Task WbcBase::formulateFrictionConeTask(const RobotState& state) {
    // Friction cone constraint: |F_t| <= mu * F_n
    // Linearized: F_x <= mu * F_z, -F_x <= mu * F_z, etc.
    // 
    // This is an inequality constraint
    
    double mu = 0.7;  // Friction coefficient
    
    size_t numContacts = 0;
    for (bool contact : state.contactStates) {
        if (contact) numContacts++;
    }
    
    // 4 linear constraints per contact foot
    Task task(numDecisionVars_, 4 * numContacts);
    
    size_t row = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (state.contactStates[i]) {
            size_t forceIdx = 6 + 3 * i;  // Contact force index in decision variables
            
            // F_x <= mu * F_z
            task.A(row, forceIdx + 0) = 1.0;
            task.A(row, forceIdx + 2) = -mu;
            task.upperBound(row) = 0.0;
            row++;
            
            // -F_x <= mu * F_z
            task.A(row, forceIdx + 0) = -1.0;
            task.A(row, forceIdx + 2) = -mu;
            task.upperBound(row) = 0.0;
            row++;
            
            // F_y <= mu * F_z
            task.A(row, forceIdx + 1) = 1.0;
            task.A(row, forceIdx + 2) = -mu;
            task.upperBound(row) = 0.0;
            row++;
            
            // -F_y <= mu * F_z
            task.A(row, forceIdx + 1) = -1.0;
            task.A(row, forceIdx + 2) = -mu;
            task.upperBound(row) = 0.0;
            row++;
        }
    }
    
    task.lowerBound = Eigen::VectorXd::Constant(task.numConstraints, -1e10);
    
    return task;
}

Task WbcBase::formulateBaseAccelTask(const RobotState& state,
                                      const ControlCommand& command,
                                      double dt) {
    // Track desired base acceleration
    // base_accel = base_accel_desired
    // 
    // This is a soft constraint (minimize tracking error)
    
    Task task(numDecisionVars_, 6);
    
    // Desired base acceleration (simplified)
    Eigen::VectorXd baseAccelDesired = Eigen::VectorXd::Zero(6);
    
    // Velocity tracking: base_accel = (v_desired - v_current) / dt
    baseAccelDesired.segment(0, 3) = (command.baseVelocityCommand - state.baseLinearVelocity) / dt;
    
    // Identity matrix for base acceleration
    task.A.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
    task.A.block(0, 6, 6, 24) = Eigen::MatrixXd::Zero(6, 24);
    
    task.b = baseAccelDesired;
    task.lowerBound = baseAccelDesired;
    task.upperBound = baseAccelDesired;
    
    return task;
}

Task WbcBase::formulateSwingLegTask(const RobotState& state,
                                     const ControlCommand& command,
                                     double dt) {
    // Track desired swing leg motion
    // For swing legs, track desired foot position
    // 
    // This is a soft constraint
    
    size_t numSwingLegs = 0;
    for (bool contact : state.contactStates) {
        if (!contact) numSwingLegs++;
    }
    
    Task task(numDecisionVars_, 3 * numSwingLegs);
    
    size_t row = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (!state.contactStates[i]) {
            // Swing leg Jacobian (simplified)
            Eigen::MatrixXd J_swing = Eigen::MatrixXd::Zero(3, numDecisionVars_);
            
            // Base acceleration affects foot position
            J_swing.block(0, 0, 3, 6) = Eigen::MatrixXd::Identity(3, 6);
            
            task.A.block(row, 0, 3, numDecisionVars_) = J_swing;
            
            // Desired foot acceleration (simplified)
            Eigen::VectorXd footAccelDesired = Eigen::VectorXd::Zero(3);
            footAccelDesired(2) = 9.81;  // Compensate gravity
            
            task.b.segment(row, 3) = footAccelDesired;
            task.lowerBound.segment(row, 3) = footAccelDesired;
            task.upperBound.segment(row, 3) = footAccelDesired;
            
            row += 3;
        }
    }
    
    return task;
}

Task WbcBase::formulateTorqueMinTask() {
    // Minimize joint torques
    // min ||tau||^2
    // 
    // This is a soft constraint
    
    Task task(numDecisionVars_, numJoints_);
    
    // Only joint torques
    task.A.block(0, 18, numJoints_, numJoints_) = Eigen::MatrixXd::Identity(numJoints_, numJoints_);
    task.A.block(0, 0, numJoints_, 18) = Eigen::MatrixXd::Zero(numJoints_, 18);
    
    task.b = Eigen::VectorXd::Zero(numJoints_);
    task.lowerBound = Eigen::VectorXd::Constant(numJoints_, -1e10);
    task.upperBound = Eigen::VectorXd::Constant(numJoints_, 1e10);
    
    return task;
}

bool WbcBase::solveHierarchicalQp(const std::vector<Task>& tasks, 
                                     Eigen::VectorXd& solution) {
    if (tasks.empty()) {
        solution = Eigen::VectorXd::Zero(numDecisionVars_);
        return true;
    }
    
    // Build hierarchical QP
    HoQp::Ptr hoqp;
    for (const auto& task : tasks) {
        hoqp = std::make_shared<HoQp>(task, hoqp);
        if (!hoqp->solve()) {
            return false;
        }
    }
    
    solution = hoqp->getSolution();
    return true;
}

}  // namespace legged_wbc
