#pragma once

#include "Task.h"
#include <memory>
#include <vector>

namespace legged_wbc {

/**
 * @brief Hierarchical Quadratic Programming solver
 * 
 * Solves multiple QP problems in a hierarchical manner, where higher priority
 * tasks are satisfied first, and lower priority tasks are optimized in the
 * null space of higher priority tasks.
 */
class HoQp {
public:
    using Ptr = std::shared_ptr<HoQp>;
    using ConstPtr = std::shared_ptr<const HoQp>;
    
    /**
     * @brief Constructor
     * @param task The task to solve
     * @param higherPriority Higher priority HoQp (nullptr if this is the highest priority)
     */
    HoQp(const Task& task, HoQp::Ptr higherPriority = nullptr);
    
    /**
     * @brief Solve the hierarchical QP problem
     * @return true if solved successfully
     */
    bool solve();
    
    /**
     * @brief Get the solution
     * @return Solution vector
     */
    Eigen::VectorXd getSolution() const { return solution_; }
    
    /**
     * @brief Get the objective value
     * @return Objective value
     */
    double getObjectiveValue() const { return objectiveValue_; }
    
    /**
     * @brief Check if the problem is feasible
     * @return true if feasible
     */
    bool isFeasible() const { return feasible_; }

private:
    Task task_;
    HoQp::Ptr higherPriority_;
    
    Eigen::VectorXd solution_;
    double objectiveValue_;
    bool feasible_;
    
    // Null space projector from higher priority tasks
    Eigen::MatrixXd nullSpaceProjector_;
    
    /**
     * @brief Solve a single QP problem
     * @param task The task to solve
     * @param solution Output solution
     * @return true if solved successfully
     */
    bool solveQp(const Task& task, Eigen::VectorXd& solution);
    
    /**
     * @brief Project task into null space
     * @param task The task to project
     * @return Projected task
     */
    Task projectToNullSpace(const Task& task);
};

}  // namespace legged_wbc
