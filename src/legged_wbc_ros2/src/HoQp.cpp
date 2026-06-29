#include "legged_wbc_ros2/HoQp.h"
#include <iostream>

namespace legged_wbc {

HoQp::HoQp(const Task& task, HoQp::Ptr higherPriority)
    : task_(task), higherPriority_(higherPriority), objectiveValue_(0.0), feasible_(false) {
    
    // Initialize null space projector
    if (higherPriority_) {
        nullSpaceProjector_ = higherPriority_->nullSpaceProjector_;
    } else {
        nullSpaceProjector_ = Eigen::MatrixXd::Identity(task.numVariables, task.numVariables);
    }
}

bool HoQp::solve() {
    // Project task to null space of higher priority tasks
    Task projectedTask = projectToNullSpace(task_);
    
    // Solve the projected QP
    Eigen::VectorXd particularSolution;
    if (!solveQp(projectedTask, particularSolution)) {
        feasible_ = false;
        return false;
    }
    
    // Combine with higher priority solution
    if (higherPriority_) {
        solution_ = higherPriority_->getSolution() + nullSpaceProjector_ * particularSolution;
    } else {
        solution_ = particularSolution;
    }
    
    // Compute objective value
    objectiveValue_ = (task_.A * solution_ - task_.b).squaredNorm();
    feasible_ = true;
    
    return true;
}

bool HoQp::solveQp(const Task& task, Eigen::VectorXd& solution) {
    // Simple QP solver using Eigen's least squares
    // For equality constraints: min ||A*x - b||^2
    
    if (task.numConstraints == 0) {
        solution = Eigen::VectorXd::Zero(task.numVariables);
        return true;
    }
    
    // Check if A matrix is zero or near-zero
    if (task.A.norm() < 1e-10) {
        solution = Eigen::VectorXd::Zero(task.numVariables);
        return true;
    }
    
    // Check if it's an equality constraint
    bool isEquality = (task.lowerBound.array() == task.upperBound.array()).all();
    
    if (isEquality) {
        // Solve equality constrained QP: min ||A*x - b||^2
        // Using pseudo-inverse: x = A^+ * b
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(task.A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        
        // Check for numerical issues
        if (svd.singularValues().size() == 0 || svd.singularValues().minCoeff() < 1e-10) {
            // Matrix is singular or near-singular, use least squares
            solution = svd.solve(task.b);
        } else {
            solution = svd.solve(task.b);
        }
        
        // Update null space projector
        // N = I - A^+ * A
        Eigen::MatrixXd A_pinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
        nullSpaceProjector_ = nullSpaceProjector_ * (Eigen::MatrixXd::Identity(task.numVariables, task.numVariables) - A_pinv * task.A);
        
        return true;
    } else {
        // For inequality constraints, use a simple active set method
        // This is a simplified implementation
        
        // Create a mutable copy of the task
        Task mutableTask = task;
        
        // Start with equality solution
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(mutableTask.A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        solution = svd.solve(mutableTask.b);
        
        // Check if solution satisfies inequality constraints
        Eigen::VectorXd Ax = mutableTask.A * solution;
        bool feasible = true;
        for (size_t i = 0; i < mutableTask.numConstraints; ++i) {
            if (Ax(i) < mutableTask.lowerBound(i) - 1e-6 || Ax(i) > mutableTask.upperBound(i) + 1e-6) {
                feasible = false;
                break;
            }
        }
        
        if (!feasible) {
            // Simple projection: clip to bounds
            for (size_t i = 0; i < mutableTask.numConstraints; ++i) {
                if (Ax(i) < mutableTask.lowerBound(i)) {
                    mutableTask.b(i) = mutableTask.lowerBound(i);
                } else if (Ax(i) > mutableTask.upperBound(i)) {
                    mutableTask.b(i) = mutableTask.upperBound(i);
                }
            }
            solution = svd.solve(mutableTask.b);
        }
        
        return true;
    }
}

Task HoQp::projectToNullSpace(const Task& task) {
    Task projectedTask(task.numVariables, task.numConstraints);
    projectedTask.A = task.A * nullSpaceProjector_;
    projectedTask.b = task.b - task.A * (higherPriority_ ? higherPriority_->getSolution() : Eigen::VectorXd::Zero(task.numVariables));
    projectedTask.lowerBound = task.lowerBound;
    projectedTask.upperBound = task.upperBound;
    
    return projectedTask;
}

}  // namespace legged_wbc
