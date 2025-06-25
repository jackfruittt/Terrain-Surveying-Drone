// Updated tsp_solver.cpp - based on your proven tsp.cpp implementation
#include "tsp_solver.h"
#include <chrono>
#include <algorithm>
#include <thread>
#include <cmath>

TSPSolver::TSPSolver(const rclcpp::Logger& logger) : logger_(logger) {}

TSPSolver::TSPSolution TSPSolver::solveTSP(
    const std::vector<geometry_msgs::msg::Pose>& goals,
    const geometry_msgs::msg::Point& start_position,
    std::function<double(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&)> cost_function,
    std::function<bool(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&)> traversability_function)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    TSPSolution solution;
    solution.solution_found = false;
    solution.total_cost = std::numeric_limits<double>::infinity();
    
    if (goals.empty()) {
        RCLCPP_WARN(logger_, "TSP: No goals provided");
        auto end_time = std::chrono::high_resolution_clock::now();
        solution.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        return solution;
    }
    
    if (goals.size() == 1) {
        // Single goal case
        solution.path = {0};
        solution.total_cost = cost_function(start_position, goals[0].position);
        solution.solution_found = true;
        solution.all_valid_paths.push_back({0});
        solution.path_costs.push_back(solution.total_cost);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        solution.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_INFO(logger_, "TSP: Single goal solution, cost: %.2fm", solution.total_cost);
        return solution;
    }
    
    RCLCPP_INFO(logger_, "TSP: Solving for %zu goals using std::next_permutation algorithm...", goals.size());
    
    // Use std::next_permutation to solve properly
    std::vector<size_t> indices;
    for (size_t i = 0; i < goals.size(); i++) {
        indices.push_back(i);
    }
    
    // Sort
    std::sort(indices.begin(), indices.end());
    
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<size_t> best_path;
    
    size_t permutations_checked = 0;
    size_t valid_paths_count = 0;
    
    do {
        permutations_checked++;
        
        // Add delay
        std::this_thread::sleep_for(std::chrono::microseconds(50)); // 50μs per permutation
        
        // Check if this path is traversable first
        bool is_traversable = true;
        geometry_msgs::msg::Point current_pos = start_position;
        
        // Check traversability from start to first goal
        if (!traversability_function(current_pos, goals[indices[0]].position)) {
            is_traversable = false;
        }
        
        // Check traversability between consecutive goals
        if (is_traversable) {
            for (size_t i = 0; i < indices.size() - 1; i++) {
                if (!traversability_function(goals[indices[i]].position, goals[indices[i+1]].position)) {
                    is_traversable = false;
                    break;
                }
            }
        }
        
        if (!is_traversable) {
            continue; // Skip this permutation
        }
        
        // Calculate path cost using path-dependent approach
        double total_cost = 0.0;
        current_pos = start_position;
        
        // Cost from start to first goal
        double leg_cost = cost_function(current_pos, goals[indices[0]].position);
        total_cost += leg_cost;
        current_pos = goals[indices[0]].position;
        
        // Cost between consecutive goals
        bool cost_calculation_valid = true;
        for (size_t i = 0; i < indices.size() - 1; i++) {
            leg_cost = cost_function(goals[indices[i]].position, goals[indices[i+1]].position);
            
            // Handle infinite costs (unreachable)
            if (std::isinf(leg_cost) || std::isnan(leg_cost)) {
                cost_calculation_valid = false;
                break;
            }
            
            total_cost += leg_cost;
            
            // Early termination if already worse than best
            if (total_cost > best_cost) {
                cost_calculation_valid = false;
                break;
            }
        }
        
        if (!cost_calculation_valid) {
            continue;
        }
        
        // Valid path
        valid_paths_count++;
        solution.all_valid_paths.push_back(indices);
        solution.path_costs.push_back(total_cost);
        
        // Check if it's the best
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_path = indices;
            
            RCLCPP_DEBUG(logger_, "TSP: New best path found with cost %.2fm (permutation %zu)", 
                        total_cost, permutations_checked);
        }
        
        // Log progress every 24 permutations (20%)
        if (permutations_checked % 24 == 0) {
            double progress = (static_cast<double>(permutations_checked) / 
                             static_cast<double>(factorial(goals.size()))) * 100.0;
            RCLCPP_DEBUG(logger_, "TSP: Progress %.1f%% (%zu/%zu permutations, %zu valid)", 
                        progress, permutations_checked, static_cast<size_t>(factorial(goals.size())), valid_paths_count);
        }
        
    } while (std::next_permutation(indices.begin(), indices.end()));
    
    auto end_time = std::chrono::high_resolution_clock::now();
    solution.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (valid_paths_count > 0) {
        solution.path = best_path;
        solution.total_cost = best_cost;
        solution.solution_found = true;
        
        RCLCPP_INFO(logger_, "TSP: Solution found! Best path cost: %.2fm", best_cost);
        RCLCPP_INFO(logger_, "TSP: Checked %zu permutations, found %zu valid paths", 
                    permutations_checked, valid_paths_count);
        RCLCPP_INFO(logger_, "TSP: Solve time: %ld ms (%.1f μs/permutation)", 
                    solution.solve_time.count(), 
                    (solution.solve_time.count() * 1000.0) / permutations_checked);
        
        // Log the optimal path
        std::string path_str = "TSP: Optimal path: START";
        for (size_t idx : solution.path) {
            path_str += " -> GOAL" + std::to_string(idx);
        }
        RCLCPP_INFO(logger_, "%s", path_str.c_str());
        
    } else {
        RCLCPP_ERROR(logger_, "TSP: No valid traversable paths found!");
        RCLCPP_ERROR(logger_, "TSP: Checked %zu permutations, all were untraversable", permutations_checked);
    }
    
    return solution;
}

// Helper function to calculate factorial for progress reporting
size_t TSPSolver::factorial(size_t n) {
    if (n <= 1) return 1;
    size_t result = 1;
    for (size_t i = 2; i <= n; i++) {
        result *= i;
    }
    return result;
}