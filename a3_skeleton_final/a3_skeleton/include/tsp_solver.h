#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <vector>
#include <functional>
#include <chrono>
#include <limits>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

class TSPSolver {
public:
    struct TSPSolution {
        bool solution_found;
        std::vector<size_t> path;
        double total_cost;
        std::vector<std::vector<size_t>> all_valid_paths;
        std::vector<double> path_costs;
        std::chrono::milliseconds solve_time;
        
        TSPSolution() : solution_found(false), total_cost(std::numeric_limits<double>::infinity()) {}
    };
    
    explicit TSPSolver(const rclcpp::Logger& logger);
    
    TSPSolution solveTSP(
        const std::vector<geometry_msgs::msg::Pose>& goals,
        const geometry_msgs::msg::Point& start_position,
        std::function<double(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&)> cost_function,
        std::function<bool(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&)> traversability_function);

private:
    rclcpp::Logger logger_;
    
    // Helper function for progress calculation
    size_t factorial(size_t n);
};

#endif // TSP_SOLVER_H