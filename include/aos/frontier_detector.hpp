/**
 * @file frontier_detector.hpp
 * @brief Frontier detection for autonomous exploration
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This class implements frontier-based exploration by detecting boundaries
 * between known and unknown areas in the occupancy grid.
 */

#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <memory>
#include <unordered_set>

namespace orbit_planner {

struct Frontier {
    geometry_msgs::msg::Point center;
    std::vector<geometry_msgs::msg::Point> points;
    double size;
    double distance_to_robot;
    double utility;
    bool visited;
    int cluster_id;
};

class FrontierDetector {
public:
    FrontierDetector();
    ~FrontierDetector() = default;

    // Parameter setting
    void setParameters(double robot_radius, int min_cluster_size, 
                      double max_planning_distance, double frontier_gain_weight);

    // Main detection function
    std::vector<Frontier> detectFrontiers(
        const nav_msgs::msg::OccupancyGrid& occupancy_grid,
        const geometry_msgs::msg::PoseStamped& robot_pose);

    // Utility functions
    void markFrontierAsVisited(const Frontier& frontier);
    void markAreaAsVisited(const geometry_msgs::msg::Point& center, double radius);
    bool isFrontierVisited(const Frontier& frontier);
    void resetVisitedMarkers();

    // Getters
    const std::vector<Frontier>& getLastFrontiers() const { return last_frontiers_; }
    const std::unordered_set<int>& getVisitedFrontiers() const { return visited_frontiers_; }

private:
    // Parameters
    double robot_radius_;
    int min_cluster_size_;
    double max_planning_distance_;
    double frontier_gain_weight_;

    // Results
    std::vector<Frontier> last_frontiers_;
    std::unordered_set<int> visited_frontiers_;

    // Internal processing functions
    std::vector<geometry_msgs::msg::Point> findFrontierCells(
        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    std::vector<Frontier> clusterFrontierPoints(
        const std::vector<geometry_msgs::msg::Point>& frontier_points);
    
    double calculateFrontierUtility(const Frontier& frontier,
                                   const geometry_msgs::msg::PoseStamped& robot_pose);
    
    double calculateDistanceToRobot(const Frontier& frontier,
                                   const geometry_msgs::msg::PoseStamped& robot_pose);
    
    bool isFrontierReachable(const Frontier& frontier,
                            const geometry_msgs::msg::PoseStamped& robot_pose);
    
    bool isPointInBounds(const geometry_msgs::msg::Point& point,
                        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    int pointToIndex(const geometry_msgs::msg::Point& point,
                    const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    geometry_msgs::msg::Point indexToPoint(int index,
                                          const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    // Utility functions
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
    
    bool isAdjacentToUnknown(int x, int y, 
                            const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    bool isFreeSpace(int x, int y, 
                    const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    bool isUnknownSpace(int x, int y, 
                       const nav_msgs::msg::OccupancyGrid& occupancy_grid);
};

} // namespace orbit_planner
