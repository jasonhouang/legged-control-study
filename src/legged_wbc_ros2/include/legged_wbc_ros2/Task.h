#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace legged_wbc {

/**
 * @brief Task representation for hierarchical optimization
 * 
 * Represents a task in the form: A*x = b (equality) or A*x <= b (inequality)
 */
struct Task {
    Eigen::MatrixXd A;  // Task matrix
    Eigen::VectorXd b;  // Task vector
    Eigen::VectorXd lowerBound;  // Lower bound for inequality constraints
    Eigen::VectorXd upperBound;  // Upper bound for inequality constraints
    
    size_t numVariables;  // Number of decision variables
    size_t numConstraints;  // Number of constraints
    
    Task() : numVariables(0), numConstraints(0) {}
    
    Task(size_t numVars, size_t numCons)
        : numVariables(numVars), numConstraints(numCons) {
        A = Eigen::MatrixXd::Zero(numCons, numVars);
        b = Eigen::VectorXd::Zero(numCons);
        lowerBound = Eigen::VectorXd::Constant(numCons, -1e10);
        upperBound = Eigen::VectorXd::Constant(numCons, 1e10);
    }
    
    // Equality task: A*x = b
    static Task createEquality(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
        Task task(A.cols(), A.rows());
        task.A = A;
        task.b = b;
        task.lowerBound = b;
        task.upperBound = b;
        return task;
    }
    
    // Inequality task: lowerBound <= A*x <= upperBound
    static Task createInequality(const Eigen::MatrixXd& A, 
                                  const Eigen::VectorXd& lower,
                                  const Eigen::VectorXd& upper) {
        Task task(A.cols(), A.rows());
        task.A = A;
        task.lowerBound = lower;
        task.upperBound = upper;
        return task;
    }
};

}  // namespace legged_wbc
