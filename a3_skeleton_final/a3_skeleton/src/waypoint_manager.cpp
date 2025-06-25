#include "waypoint_manager.h"
#include <cmath>

WaypointManager::WaypointManager(
    const rclcpp::Logger& logger,
    double max_gradient,
    const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr& waypoints_pub,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& markers_pub)
    : logger_(logger), 
      max_gradient_(max_gradient),
      waypoints_pub_(waypoints_pub),
      markers_pub_(markers_pub),
      getElevationAtPoint_([](const geometry_msgs::msg::Point&){ return 0.0; }) // Default empty function
{
}

void WaypointManager::setElevationFunction(std::function<double(const geometry_msgs::msg::Point&)> elevationFn) {
    getElevationAtPoint_ = elevationFn;
}

void WaypointManager::setGradientFunction(std::function<double(double, double)> gradientFn) {
    getGradientMagnitudeAtPoint_ = gradientFn;
}

bool WaypointManager::isTraversable(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& end) {
    // Get the elevations at the start and end points
    double start_elevation = getElevationAtPoint_(start.position);
    double end_elevation = getElevationAtPoint_(end.position);
    
    // Calculate horizontal distance
    double dx = end.position.x - start.position.x;
    double dy = end.position.y - start.position.y;
    double horizontal_distance = std::sqrt(dx*dx + dy*dy);
    
    if (horizontal_distance < 0.001) {
        // Points are too close horizontally, consider it traversable
        return true;
    }
    
    // Calculate the elevation change
    double elevation_change = std::abs(end_elevation - start_elevation);
    
    // Calculate gradient as percentage
    double gradient = (elevation_change / horizontal_distance) * 100.0;
    
    RCLCPP_DEBUG(logger_, "Path from (%.2f,%.2f) to (%.2f,%.2f): gradient=%.2f%% (max=%.2f%%)", 
                start.position.x, start.position.y, 
                end.position.x, end.position.y, 
                gradient, max_gradient_);
    
    // Check intermediate points along the path for more accuracy
    int num_checks = static_cast<int>(horizontal_distance / 0.5) + 1; // Check every 0.5m
    for (int i = 1; i < num_checks; i++) {
        double t1 = static_cast<double>(i-1) / num_checks;
        double t2 = static_cast<double>(i) / num_checks;
        
        // Interpolate positions
        geometry_msgs::msg::Point p1, p2;
        p1.x = start.position.x + t1 * dx;
        p1.y = start.position.y + t1 * dy;
        p2.x = start.position.x + t2 * dx;
        p2.y = start.position.y + t2 * dy;
        
        // Get elevations
        double elev1 = getElevationAtPoint_(p1);
        double elev2 = getElevationAtPoint_(p2);
        
        // Calculate segment distance and gradient
        double segment_dist = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
        double segment_gradient = (std::abs(elev2 - elev1) / segment_dist) * 100.0;
        
        if (segment_gradient > max_gradient_) {
            RCLCPP_WARN(logger_, "Segment gradient %.2f%% exceeds max %.2f%%", 
                       segment_gradient, max_gradient_);
            return false;
        }
    }
    
    return gradient <= max_gradient_;
}

std::vector<geometry_msgs::msg::Pose> WaypointManager::generateGroundWaypoints(
    const geometry_msgs::msg::Pose& start, 
    const geometry_msgs::msg::Pose& end,
    double spacing)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    // Calculate horizontal distance between start and end
    double dx = end.position.x - start.position.x;
    double dy = end.position.y - start.position.y;
    double horizontal_distance = std::sqrt(dx*dx + dy*dy);
    
    // Calculate number of segments
    int num_segments = std::max(1, static_cast<int>(horizontal_distance / spacing));
    
    // Create waypoints
    for (int i = 0; i <= num_segments; i++) {
        geometry_msgs::msg::Pose waypoint;
        
        // Interpolate x and y position
        double t = static_cast<double>(i) / static_cast<double>(num_segments);
        waypoint.position.x = start.position.x + t * dx;
        waypoint.position.y = start.position.y + t * dy;
        
        // Get ground elevation at this point
        waypoint.position.z = getElevationAtPoint_(waypoint.position);

        // Safety margin for drone clearance
        waypoint.position.z += 2.0;
        
        // Calculate orientation to next waypoint
        if (i < num_segments) {
            // Calculate yaw angle to next point
            double next_t = static_cast<double>(i + 1) / static_cast<double>(num_segments);
            double next_x = start.position.x + next_t * dx;
            double next_y = start.position.y + next_t * dy;

            // Calculate yaw angle
            double yaw = std::atan2(next_y - waypoint.position.y, 
                next_x - waypoint.position.x);
            
            // Convert to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            waypoint.orientation = tf2::toMsg(q);
        } else {
            // For the last point, use the end pose orientation
            waypoint.orientation = end.orientation;
        }
        
        waypoints.push_back(waypoint);
    }
    
    return waypoints;
}

std::vector<geometry_msgs::msg::Pose> WaypointManager::generateDroneWaypoints(
    const geometry_msgs::msg::Pose& start, 
    const geometry_msgs::msg::Pose& end,
    double spacing)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    // Calculate 3D distance between start and end (including Z difference)
    double dx = end.position.x - start.position.x;
    double dy = end.position.y - start.position.y;
    double dz = end.position.z - start.position.z;
    double total_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Calculate number of segments for 1m spacing
    int num_segments = std::max(1, static_cast<int>(std::ceil(total_distance / spacing)));
    
    RCLCPP_DEBUG(logger_, "Generating drone waypoints: distance=%.2fm, segments=%d", 
                total_distance, num_segments);
    
    // Create intermediate waypoints every 1m along the 3D path
    for (int i = 0; i <= num_segments; i++) {
        geometry_msgs::msg::Pose waypoint;
        
        // Interpolate position along the 3D line (including Z)
        double t = static_cast<double>(i) / static_cast<double>(num_segments);
        waypoint.position.x = start.position.x + t * dx;
        waypoint.position.y = start.position.y + t * dy;
        waypoint.position.z = start.position.z + t * dz;  // Linear interpolation of altitude
        
        // Calculate orientation toward the end goal
        if (i < num_segments) {
            // Calculate direction to end goal
            double remaining_dx = end.position.x - waypoint.position.x;
            double remaining_dy = end.position.y - waypoint.position.y;
            
            // Calculate yaw angle
            double yaw = std::atan2(remaining_dy, remaining_dx);
            
            // Convert to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            waypoint.orientation = tf2::toMsg(q);
        } else {
            // For the last point, use the end pose orientation
            waypoint.orientation = end.orientation;
        }
        
        waypoints.push_back(waypoint);
    }
    
    return waypoints;
}

bool WaypointManager::isWaypointTraversable(const geometry_msgs::msg::Pose& waypoint) {
    
    double x = waypoint.position.x;
    double y = waypoint.position.y;
    
    // Use sensor node's gradient magnitude to determine traversability
    double gradient_magnitude = getGradientMagnitudeAtPoint_(x, y);
    
    // Convert from slope to percentage gradient
    double gradient_percent = gradient_magnitude * 100.0;
    
    // Check if gradient exceeds the maximum allowed (3% by default)
    bool is_traversable = gradient_percent <= max_gradient_;
    
    RCLCPP_DEBUG(logger_, "Waypoint at (%.2f, %.2f): gradient=%.2f%%, traversable=%s",
                x, y, gradient_percent, is_traversable ? "YES" : "NO");
    
    return is_traversable;
}

void WaypointManager::publishWaypoints(const VehicleData& vehicle, const rclcpp::Time& now) {
    // Create a pose array message for the DRONE flight path
    geometry_msgs::msg::PoseArray waypoints_msg;
    waypoints_msg.header.frame_id = "world";
    waypoints_msg.header.stamp = now;
    
    // Create a local copy of the goals
    std::vector<geometry_msgs::msg::Pose> goals;
    {
        auto& non_const_vehicle = const_cast<VehicleData&>(vehicle);
        std::lock_guard<std::mutex> lock(non_const_vehicle.goals_mutex);
        goals = vehicle.goals;
    }
    
    // Always publish, even if empty
    if (goals.empty()) {
        waypoints_pub_->publish(waypoints_msg);
        return;
    }

    RCLCPP_INFO(logger_, "Generating waypoints for %zu goals with adaptive altitude control", goals.size());
    
    // Generate waypoints based on expected path from goal to goal
    std::vector<geometry_msgs::msg::Pose> drone_waypoints;
    
    if (goals.size() == 1) {
        // Single goal case - create waypoints from current position to goal
        geometry_msgs::msg::Pose current_pos;
        {
            auto& non_const_vehicle = const_cast<VehicleData&>(vehicle);
            std::lock_guard<std::mutex> lock(non_const_vehicle.odom_mutex);
            current_pos.position = vehicle.current_odom.pose.pose.position;
            current_pos.orientation = vehicle.current_odom.pose.pose.orientation;
        }
        
        // Adjust altitudes based on goal type
        geometry_msgs::msg::Pose adjusted_start = current_pos;
        geometry_msgs::msg::Pose adjusted_goal = goals[0];
        
        // For terrain following goals, ensure waypoints are at least 2m high
        if (adjusted_start.position.z < 2.0 && goals[0].position.z <= 0.1) {
            adjusted_start.position.z = 2.0;
        }
        if (adjusted_goal.position.z <= 0.1) {
            adjusted_goal.position.z = 2.0;  // Visualization at 2m for terrain following
        }
        
        // Generate waypoints from current position to single goal
        auto segment_waypoints = generateDroneWaypoints(adjusted_start, adjusted_goal, 1.0);
        drone_waypoints = segment_waypoints;
        
    } else {
        // Multiple goals case - generate waypoints between consecutive goals
        for (size_t i = 0; i < goals.size() - 1; i++) {
            geometry_msgs::msg::Pose start_goal = goals[i];
            geometry_msgs::msg::Pose end_goal = goals[i + 1];
            
            // Adjust altitudes based on goal types
            if (start_goal.position.z <= 0.1) {
                start_goal.position.z = 2.0;  // Terrain following -> 2m visualization
            }
            if (end_goal.position.z <= 0.1) {
                end_goal.position.z = 2.0;    // Terrain following -> 2m visualization
            }
            
            // Generate 1m waypoints along the 3D path
            auto segment_waypoints = generateDroneWaypoints(start_goal, end_goal, 1.0);
            
            // Add all waypoints from this segment (excluding the last one to avoid duplicates)
            for (size_t j = 0; j < segment_waypoints.size() - 1; j++) {
                drone_waypoints.push_back(segment_waypoints[j]);
            }
        }
        
        // Add the final goal with proper altitude
        geometry_msgs::msg::Pose final_goal = goals.back();
        if (final_goal.position.z <= 0.1) {
            final_goal.position.z = 2.0;  // Visualization at 2m for terrain following
        }
        drone_waypoints.push_back(final_goal);
    }
    
    // Add all waypoints to the message
    for (const auto& waypoint : drone_waypoints) {
        waypoints_msg.poses.push_back(waypoint);
    }
    
    // Publish waypoints
    waypoints_pub_->publish(waypoints_msg);
    
    // Store the waypoints in the vehicle (for markers)
    {
        auto& non_const_vehicle = const_cast<VehicleData&>(vehicle);
        std::lock_guard<std::mutex> lock(non_const_vehicle.goals_mutex);
        non_const_vehicle.traversable_waypoints = drone_waypoints;
    }
    
    RCLCPP_INFO(logger_, "Published %zu adaptive waypoints (terrain following at 2m, manual altitudes preserved)", 
                drone_waypoints.size());
}

void WaypointManager::publishMarkers(const VehicleData& vehicle, const rclcpp::Time& now) {
    // Create marker array for visualisation
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Get drone waypoints and goals
    std::vector<geometry_msgs::msg::Pose> waypoints;
    std::vector<geometry_msgs::msg::Pose> goals;
    {
        auto& non_const_vehicle = const_cast<VehicleData&>(vehicle);
        std::lock_guard<std::mutex> lock(non_const_vehicle.goals_mutex);
        waypoints = vehicle.traversable_waypoints;  // 1m intermediate waypoints
        goals = vehicle.goals;  // original goal poses
    }
    
    // Markers for intermediate 1m waypoints - CYLINDER with radius 0.2m, height 0.5m
    int traversable_count = 0;
    int non_traversable_count = 0;
    
    for (size_t i = 0; i < waypoints.size(); i++) {
        // Check if waypoint is over traversable terrain
        bool is_traversable = isWaypointTraversable(waypoints[i]);
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now;
        marker.ns = "waypoint";  // Namespace "waypoint" as required
        marker.id = static_cast<int>(i);
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position at waypoint location
        marker.pose = waypoints[i];
        
        if (is_traversable) {
            // CYLINDER for traversable terrain (gradient ≤3%)
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            
            // Size as per requirements - radius 0.2m, height 0.5m
            marker.scale.x = 0.4;  // 0.4m diameter = 0.2m radius
            marker.scale.y = 0.4;  // 0.4m diameter = 0.2m radius  
            marker.scale.z = 0.5;  // 0.5m height
            
            // Green color for traversable waypoints
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;
            
            traversable_count++;
            
        } else {
            // SPHERE for non-traversable terrain (gradient >3%)
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            
            // Smaller size to distinguish from goals
            marker.scale.x = 0.25;
            marker.scale.y = 0.25;
            marker.scale.z = 0.25;
            
            // Red color for non-traversable waypoints
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;
            
            non_traversable_count++;
        }
        
        // Lifetime - NEVER DELETE waypoints
        marker.lifetime = rclcpp::Duration(0, 0);  // Persistent forever
        
        marker_array.markers.push_back(marker);
    }
    
    // Arrow markers for original goals (pointing to next goal)
    for (size_t i = 0; i < goals.size(); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now;
        marker.ns = "goal";  // Different namespace for goals
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::ARROW;  // ARROW for goals
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position at goal location
        marker.pose.position = goals[i].position;
        
        // Calculate orientation pointing to next goal
        if (i < goals.size() - 1) {
            // Calculate direction to next goal
            double dx = goals[i + 1].position.x - goals[i].position.x;
            double dy = goals[i + 1].position.y - goals[i].position.y;
            
            // Calculate yaw angle
            double yaw = std::atan2(dy, dx);
            
            // Convert to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            marker.pose.orientation = tf2::toMsg(q);
        } else {
            // For the last goal, point forward (0 degrees)
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            marker.pose.orientation = tf2::toMsg(q);
        }
        
        // Arrow size
        marker.scale.x = 0.8;  
        marker.scale.y = 0.2; 
        marker.scale.z = 0.2;  
        
        // Blue color for goals
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.8;
        
        // Lifetime - NEVER DELETE
        marker.lifetime = rclcpp::Duration(0, 0);
        
        marker_array.markers.push_back(marker);
    }
    
    // Publish the complete marker array to /visualization_marker
    markers_pub_->publish(marker_array);
    
    RCLCPP_INFO(logger_, "Published markers: %d traversable (green cylinders), %d non-traversable (red spheres), %zu goals (blue arrows)", 
                traversable_count, non_traversable_count, goals.size());
}