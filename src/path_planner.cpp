#include "aos/path_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace orbit_planner {

PathPlanner::PathPlanner() 
    : robot_radius_(0.4), safety_margin_(0.1), resolution_(0.1), max_planning_distance_(50.0) {
}

void PathPlanner::setParameters(double robot_radius, double safety_margin,
                               double resolution, double max_planning_distance) {
    robot_radius_ = robot_radius;
    safety_margin_ = safety_margin;
    resolution_ = resolution;
    max_planning_distance_ = max_planning_distance;
}

nav_msgs::msg::Path PathPlanner::planPath(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    
    return planAStar(start, goal, occupancy_grid);
}

nav_msgs::msg::Path PathPlanner::planPathWithESDF(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const std::function<double(const geometry_msgs::msg::Point&)>& esdf_query) {
    
    // Simple straight-line path for now
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();
    
    path.poses.push_back(start);
    path.poses.push_back(goal);
    
    return path;
}

nav_msgs::msg::Path PathPlanner::smoothPath(const nav_msgs::msg::Path& path,
                                           const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    return shortcutPath(path, occupancy_grid);
}

bool PathPlanner::isPathValid(const nav_msgs::msg::Path& path,
                             const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    for (const auto& pose : path.poses) {
        if (!isCollisionFree(pose.pose.position, occupancy_grid)) {
            return false;
        }
    }
    return true;
}

double PathPlanner::calculatePathLength(const nav_msgs::msg::Path& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
        length += calculateDistance(path.poses[i-1].pose.position, path.poses[i].pose.position);
    }
    return length;
}

double PathPlanner::calculatePathCurvature(const nav_msgs::msg::Path& path) {
    if (path.poses.size() < 3) return 0.0;
    
    double total_curvature = 0.0;
    for (size_t i = 1; i < path.poses.size() - 1; ++i) {
        double curvature = calculateAngle(path.poses[i-1].pose.position,
                                        path.poses[i].pose.position,
                                        path.poses[i+1].pose.position);
        total_curvature += std::abs(curvature);
    }
    
    return total_curvature / (path.poses.size() - 2);
}

nav_msgs::msg::Path PathPlanner::planAStar(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();
    
    // Simple implementation - straight line for now
    path.poses.push_back(start);
    path.poses.push_back(goal);
    
    return path;
}

nav_msgs::msg::Path PathPlanner::planRRTStar(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    
    // Placeholder for RRT* implementation
    return planAStar(start, goal, occupancy_grid);
}

std::vector<geometry_msgs::msg::Point> PathPlanner::getNeighbors(
    const geometry_msgs::msg::Point& point,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    
    std::vector<geometry_msgs::msg::Point> neighbors;
    
    // 8-connected neighborhood
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            
            geometry_msgs::msg::Point neighbor;
            neighbor.x = point.x + dx * resolution_;
            neighbor.y = point.y + dy * resolution_;
            neighbor.z = point.z;
            
            if (isCollisionFree(neighbor, occupancy_grid)) {
                neighbors.push_back(neighbor);
            }
        }
    }
    
    return neighbors;
}

double PathPlanner::calculateHeuristic(const geometry_msgs::msg::Point& current,
                                     const geometry_msgs::msg::Point& goal) {
    return calculateDistance(current, goal);
}

double PathPlanner::calculateCost(const geometry_msgs::msg::Point& from,
                                 const geometry_msgs::msg::Point& to) {
    return calculateDistance(from, to);
}

bool PathPlanner::isCollisionFree(const geometry_msgs::msg::Point& point,
                                 const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    geometry_msgs::msg::Point grid_point = worldToGrid(point, occupancy_grid);
    int x = static_cast<int>(grid_point.x);
    int y = static_cast<int>(grid_point.y);
    
    int width = occupancy_grid.info.width;
    int height = occupancy_grid.info.height;
    
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return false;
    }
    
    int index = y * width + x;
    return occupancy_grid.data[index] == 0;
}

bool PathPlanner::isLineCollisionFree(const geometry_msgs::msg::Point& start,
                                     const geometry_msgs::msg::Point& end,
                                     const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    // Simple line collision check
    double distance = calculateDistance(start, end);
    int steps = static_cast<int>(distance / resolution_) + 1;
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        geometry_msgs::msg::Point point;
        point.x = start.x + t * (end.x - start.x);
        point.y = start.y + t * (end.y - start.y);
        point.z = start.z + t * (end.z - start.z);
        
        if (!isCollisionFree(point, occupancy_grid)) {
            return false;
        }
    }
    
    return true;
}

geometry_msgs::msg::Point PathPlanner::worldToGrid(const geometry_msgs::msg::Point& world_point,
                                                  const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    geometry_msgs::msg::Point grid_point;
    grid_point.x = (world_point.x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution;
    grid_point.y = (world_point.y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution;
    grid_point.z = 0.0;
    return grid_point;
}

geometry_msgs::msg::Point PathPlanner::gridToWorld(const geometry_msgs::msg::Point& grid_point,
                                                  const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    geometry_msgs::msg::Point world_point;
    world_point.x = occupancy_grid.info.origin.position.x + grid_point.x * occupancy_grid.info.resolution;
    world_point.y = occupancy_grid.info.origin.position.y + grid_point.y * occupancy_grid.info.resolution;
    world_point.z = 0.0;
    return world_point;
}

int PathPlanner::pointToIndex(const geometry_msgs::msg::Point& point,
                             const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    geometry_msgs::msg::Point grid_point = worldToGrid(point, occupancy_grid);
    int x = static_cast<int>(grid_point.x);
    int y = static_cast<int>(grid_point.y);
    return y * occupancy_grid.info.width + x;
}

geometry_msgs::msg::Point PathPlanner::indexToPoint(int index,
                                                   const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    int width = occupancy_grid.info.width;
    int y = index / width;
    int x = index % width;
    
    geometry_msgs::msg::Point grid_point;
    grid_point.x = x;
    grid_point.y = y;
    grid_point.z = 0.0;
    
    return gridToWorld(grid_point, occupancy_grid);
}

nav_msgs::msg::Path PathPlanner::shortcutPath(const nav_msgs::msg::Path& path,
                                             const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    if (path.poses.size() <= 2) {
        return path;
    }
    
    nav_msgs::msg::Path shortcut_path;
    shortcut_path.header = path.header;
    
    shortcut_path.poses.push_back(path.poses[0]);
    
    size_t i = 0;
    while (i < path.poses.size() - 1) {
        size_t j = path.poses.size() - 1;
        bool found_shortcut = false;
        
        while (j > i + 1) {
            if (isLineCollisionFree(path.poses[i].pose.position, path.poses[j].pose.position, occupancy_grid)) {
                shortcut_path.poses.push_back(path.poses[j]);
                i = j;
                found_shortcut = true;
                break;
            }
            j--;
        }
        
        if (!found_shortcut) {
            i++;
            if (i < path.poses.size()) {
                shortcut_path.poses.push_back(path.poses[i]);
            }
        }
    }
    
    return shortcut_path;
}

nav_msgs::msg::Path PathPlanner::splineSmoothPath(const nav_msgs::msg::Path& path) {
    // Placeholder for spline smoothing
    return path;
}

double PathPlanner::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                     const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

double PathPlanner::calculateAngle(const geometry_msgs::msg::Point& p1, 
                                  const geometry_msgs::msg::Point& p2, 
                                  const geometry_msgs::msg::Point& p3) {
    // Calculate angle at p2
    double v1x = p1.x - p2.x;
    double v1y = p1.y - p2.y;
    double v2x = p3.x - p2.x;
    double v2y = p3.y - p2.y;
    
    double dot = v1x * v2x + v1y * v2y;
    double mag1 = sqrt(v1x * v1x + v1y * v1y);
    double mag2 = sqrt(v2x * v2x + v2y * v2y);
    
    if (mag1 == 0.0 || mag2 == 0.0) return 0.0;
    
    double cos_angle = dot / (mag1 * mag2);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    
    return acos(cos_angle);
}

geometry_msgs::msg::PoseStamped PathPlanner::createPoseStamped(
    const geometry_msgs::msg::Point& position,
    double yaw) {
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = rclcpp::Clock().now();
    pose.pose.position = position;
    
    // Convert yaw to quaternion
    pose.pose.orientation.w = cos(yaw / 2.0);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(yaw / 2.0);
    
    return pose;
}

} // namespace orbit_planner
