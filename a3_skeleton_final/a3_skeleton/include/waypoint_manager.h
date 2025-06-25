#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include "common.h"

class WaypointManager {
public:
    /**
     * @brief Constructor for WaypointManager
     * 
     * @param logger ROS logger instance
     * @param max_gradient Maximum allowed gradient for traversable paths
     * @param waypoints_pub Publisher for waypoints
     * @param markers_pub Publisher for visualisation markers
     */
    WaypointManager(
        const rclcpp::Logger& logger,
        double max_gradient,
        const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr& waypoints_pub,
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& markers_pub);
    
    /**
     * @brief Check if path between two poses is traversable
     * 
     * @param start Starting pose
     * @param end Ending pose
     * @return true if traversable, false otherwise
     */
    bool isTraversable(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& end);
    
    /**
     * @brief Generate a sequence of waypoints between two poses
     * 
     * @param start Starting pose
     * @param end Ending pose
     * @param spacing Distance between generated waypoints
     * @return Vector of poses representing the path
     */
    std::vector<geometry_msgs::msg::Pose> generateGroundWaypoints(
        const geometry_msgs::msg::Pose& start, 
        const geometry_msgs::msg::Pose& end,
        double spacing = 1.0);
    
    /**
     * @brief Generate and publish waypoints for the given vehicle
     * 
     * @param vehicle Vehicle data containing goals
     * @param now Current time from ROS
     */
    void publishWaypoints(const VehicleData& vehicle, const rclcpp::Time& now);
    
    /**
     * @brief Generate and publish visualisation markers
     * 
     * @param vehicle Vehicle data containing goals and waypoints
     * @param now Current time from ROS
     */
    void publishMarkers(const VehicleData& vehicle, const rclcpp::Time& now);
    
    /**
     * @brief Set the function to get elevation at a point
     * 
     * @param elevationFn Function that takes a point and returns its elevation
     * @param gradientFn Function that takes two elevations and returns the gradient
     */
    void setElevationFunction(std::function<double(const geometry_msgs::msg::Point&)> elevationFn);
    void setGradientFunction(std::function<double(double, double)> gradientFn);

private:
    
    rclcpp::Logger logger_;
    double max_gradient_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    // Function pointers for elevation and gradient calculations
    std::function<double(double, double)> getGradientMagnitudeAtPoint_;
    std::function<double(const geometry_msgs::msg::Point&)> getElevationAtPoint_;
    
    // Method for drone waypoints
    std::vector<geometry_msgs::msg::Pose> generateDroneWaypoints(
        const geometry_msgs::msg::Pose& start, 
        const geometry_msgs::msg::Pose& end,
        double spacing = 1.0);  // Default 1m spacing

    /**
     * @brief Check if a single waypoint is traversable
     * 
     * @param waypoint The waypoint to check
     * @return true if the waypoint is traversable, false otherwise
     */
    bool isWaypointTraversable(const geometry_msgs::msg::Pose& waypoint);
};

#endif // WAYPOINT_MANAGER_H