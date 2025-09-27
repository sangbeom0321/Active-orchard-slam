/**
 * @file path_planner.hpp
 * @brief ESDF-based path planning for autonomous exploration
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This class implements path planning algorithms using occupancy grids
 * and distance fields for safe navigation in orchard environments.
 */

#pragma once

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

// #include <opencv2/core.hpp>
// #include <opencv2/imgproc.hpp>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <memory>

namespace orbit_planner {

struct PathNode {
    geometry_msgs::msg::Point position;
    double g_cost;  // cost from start
    double h_cost;  // heuristic cost to goal
    double f_cost;  // total cost
    std::shared_ptr<PathNode> parent;
    bool visited;
    
    PathNode() : g_cost(0.0), h_cost(0.0), f_cost(0.0), visited(false) {}
};

class PathPlanner {
public:
    PathPlanner();
    ~PathPlanner() = default;

    // Parameter setting
    void setParameters(double robot_radius, double safety_margin,
                      double resolution, double max_planning_distance);

    // Main planning function
    nav_msgs::msg::Path planPath(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const nav_msgs::msg::OccupancyGrid& occupancy_grid);

    // Alternative planning with ESDF
    nav_msgs::msg::Path planPathWithESDF(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const std::function<double(const geometry_msgs::msg::Point&)>& esdf_query);

    // Path smoothing
    nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path,
                                  const nav_msgs::msg::OccupancyGrid& occupancy_grid);

    // Utility functions
    bool isPathValid(const nav_msgs::msg::Path& path,
                    const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    double calculatePathLength(const nav_msgs::msg::Path& path);
    
    double calculatePathCurvature(const nav_msgs::msg::Path& path);

private:
    // Parameters
    double robot_radius_;
    double safety_margin_;
    double resolution_;
    double max_planning_distance_;

    // A* planning
    nav_msgs::msg::Path planAStar(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    // RRT* planning
    nav_msgs::msg::Path planRRTStar(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const nav_msgs::msg::OccupancyGrid& occupancy_grid);

    // Utility functions
    std::vector<geometry_msgs::msg::Point> getNeighbors(
        const geometry_msgs::msg::Point& point,
        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    double calculateHeuristic(const geometry_msgs::msg::Point& current,
                             const geometry_msgs::msg::Point& goal);
    
    double calculateCost(const geometry_msgs::msg::Point& from,
                        const geometry_msgs::msg::Point& to);
    
    bool isCollisionFree(const geometry_msgs::msg::Point& point,
                        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    bool isLineCollisionFree(const geometry_msgs::msg::Point& start,
                            const geometry_msgs::msg::Point& end,
                            const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    geometry_msgs::msg::Point worldToGrid(const geometry_msgs::msg::Point& world_point,
                                         const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    geometry_msgs::msg::Point gridToWorld(const geometry_msgs::msg::Point& grid_point,
                                         const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    int pointToIndex(const geometry_msgs::msg::Point& point,
                    const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    geometry_msgs::msg::Point indexToPoint(int index,
                                          const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    // Path smoothing
    nav_msgs::msg::Path shortcutPath(const nav_msgs::msg::Path& path,
                                    const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    nav_msgs::msg::Path splineSmoothPath(const nav_msgs::msg::Path& path);
    
    // Utility functions
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
    
    double calculateAngle(const geometry_msgs::msg::Point& p1, 
                         const geometry_msgs::msg::Point& p2, 
                         const geometry_msgs::msg::Point& p3);
    
    geometry_msgs::msg::PoseStamped createPoseStamped(
        const geometry_msgs::msg::Point& position,
        double yaw = 0.0);
};

} // namespace orbit_planner
