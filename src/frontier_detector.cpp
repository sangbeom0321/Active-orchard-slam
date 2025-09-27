#include "aos/frontier_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace orbit_planner {

FrontierDetector::FrontierDetector() 
    : robot_radius_(0.4), min_cluster_size_(5), max_planning_distance_(50.0),
      frontier_gain_weight_(1.0) {
}

void FrontierDetector::setParameters(double robot_radius, int min_cluster_size, 
                                    double max_planning_distance, double frontier_gain_weight) {
    robot_radius_ = robot_radius;
    min_cluster_size_ = min_cluster_size;
    max_planning_distance_ = max_planning_distance;
    frontier_gain_weight_ = frontier_gain_weight;
}

std::vector<Frontier> FrontierDetector::detectFrontiers(
    const nav_msgs::msg::OccupancyGrid& occupancy_grid,
    const geometry_msgs::msg::PoseStamped& robot_pose) {
    
    // Find frontier cells
    auto frontier_points = findFrontierCells(occupancy_grid);
    
    // Cluster frontier points
    auto frontiers = clusterFrontierPoints(frontier_points);
    
    // Calculate utilities
    for (auto& frontier : frontiers) {
        frontier.distance_to_robot = calculateDistanceToRobot(frontier, robot_pose);
        frontier.utility = calculateFrontierUtility(frontier, robot_pose);
        frontier.visited = false;
    }
    
    last_frontiers_ = frontiers;
    return frontiers;
}

void FrontierDetector::markFrontierAsVisited(const Frontier& frontier) {
    visited_frontiers_.insert(frontier.cluster_id);
}

void FrontierDetector::markAreaAsVisited(const geometry_msgs::msg::Point& center, double radius) {
    // Mark all frontiers within radius as visited
    for (const auto& frontier : last_frontiers_) {
        if (calculateDistance(frontier.center, center) <= radius) {
            visited_frontiers_.insert(frontier.cluster_id);
        }
    }
}

bool FrontierDetector::isFrontierVisited(const Frontier& frontier) {
    return visited_frontiers_.find(frontier.cluster_id) != visited_frontiers_.end();
}

void FrontierDetector::resetVisitedMarkers() {
    visited_frontiers_.clear();
}

std::vector<geometry_msgs::msg::Point> FrontierDetector::findFrontierCells(
    const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    
    std::vector<geometry_msgs::msg::Point> frontier_points;
    
    int width = occupancy_grid.info.width;
    int height = occupancy_grid.info.height;
    double resolution = occupancy_grid.info.resolution;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (isFreeSpace(x, y, occupancy_grid) && isAdjacentToUnknown(x, y, occupancy_grid)) {
                geometry_msgs::msg::Point point;
                point.x = occupancy_grid.info.origin.position.x + x * resolution;
                point.y = occupancy_grid.info.origin.position.y + y * resolution;
                point.z = 0.0;
                frontier_points.push_back(point);
            }
        }
    }
    
    return frontier_points;
}

std::vector<Frontier> FrontierDetector::clusterFrontierPoints(
    const std::vector<geometry_msgs::msg::Point>& frontier_points) {
    
    std::vector<Frontier> frontiers;
    
    if (frontier_points.empty()) {
        return frontiers;
    }
    
    // Simple clustering - group nearby points
    std::vector<bool> processed(frontier_points.size(), false);
    
    for (size_t i = 0; i < frontier_points.size(); ++i) {
        if (processed[i]) continue;
        
        Frontier frontier;
        frontier.cluster_id = frontiers.size();
        frontier.points.push_back(frontier_points[i]);
        processed[i] = true;
        
        // Find nearby points
        for (size_t j = i + 1; j < frontier_points.size(); ++j) {
            if (processed[j]) continue;
            
            double distance = calculateDistance(frontier_points[i], frontier_points[j]);
            if (distance < 2.0) { // 2 meter clustering distance
                frontier.points.push_back(frontier_points[j]);
                processed[j] = true;
            }
        }
        
        // Calculate center and size
        if (frontier.points.size() >= min_cluster_size_) {
            double sum_x = 0, sum_y = 0;
            for (const auto& point : frontier.points) {
                sum_x += point.x;
                sum_y += point.y;
            }
            
            frontier.center.x = sum_x / frontier.points.size();
            frontier.center.y = sum_y / frontier.points.size();
            frontier.center.z = 0.0;
            frontier.size = frontier.points.size();
            
            frontiers.push_back(frontier);
        }
    }
    
    return frontiers;
}

double FrontierDetector::calculateFrontierUtility(const Frontier& frontier,
                                                 const geometry_msgs::msg::PoseStamped& robot_pose) {
    double distance = calculateDistanceToRobot(frontier, robot_pose);
    double size_factor = frontier.size / 100.0; // Normalize size
    
    return frontier_gain_weight_ * size_factor - distance / max_planning_distance_;
}

double FrontierDetector::calculateDistanceToRobot(const Frontier& frontier,
                                                 const geometry_msgs::msg::PoseStamped& robot_pose) {
    return calculateDistance(frontier.center, robot_pose.pose.position);
}

bool FrontierDetector::isFrontierReachable(const Frontier& frontier,
                                          const geometry_msgs::msg::PoseStamped& robot_pose) {
    double distance = calculateDistanceToRobot(frontier, robot_pose);
    return distance <= max_planning_distance_;
}

bool FrontierDetector::isPointInBounds(const geometry_msgs::msg::Point& point,
                                      const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    double x = point.x - occupancy_grid.info.origin.position.x;
    double y = point.y - occupancy_grid.info.origin.position.y;
    
    int grid_x = static_cast<int>(x / occupancy_grid.info.resolution);
    int grid_y = static_cast<int>(y / occupancy_grid.info.resolution);
    
    return grid_x >= 0 && grid_x < occupancy_grid.info.width &&
           grid_y >= 0 && grid_y < occupancy_grid.info.height;
}

int FrontierDetector::pointToIndex(const geometry_msgs::msg::Point& point,
                                  const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    double x = point.x - occupancy_grid.info.origin.position.x;
    double y = point.y - occupancy_grid.info.origin.position.y;
    
    int grid_x = static_cast<int>(x / occupancy_grid.info.resolution);
    int grid_y = static_cast<int>(y / occupancy_grid.info.resolution);
    
    return grid_y * occupancy_grid.info.width + grid_x;
}

geometry_msgs::msg::Point FrontierDetector::indexToPoint(int index,
                                                        const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    int width = occupancy_grid.info.width;
    int y = index / width;
    int x = index % width;
    
    geometry_msgs::msg::Point point;
    point.x = occupancy_grid.info.origin.position.x + x * occupancy_grid.info.resolution;
    point.y = occupancy_grid.info.origin.position.y + y * occupancy_grid.info.resolution;
    point.z = 0.0;
    
    return point;
}

double FrontierDetector::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                          const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

bool FrontierDetector::isAdjacentToUnknown(int x, int y, 
                                          const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    int width = occupancy_grid.info.width;
    int height = occupancy_grid.info.height;
    
    // Check 8-neighborhood
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = x + dx;
            int ny = y + dy;
            
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                int index = ny * width + nx;
                if (isUnknownSpace(nx, ny, occupancy_grid)) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool FrontierDetector::isFreeSpace(int x, int y, 
                                  const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    int width = occupancy_grid.info.width;
    int height = occupancy_grid.info.height;
    
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return false;
    }
    
    int index = y * width + x;
    return occupancy_grid.data[index] == 0;
}

bool FrontierDetector::isUnknownSpace(int x, int y, 
                                     const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    int width = occupancy_grid.info.width;
    int height = occupancy_grid.info.height;
    
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return true; // Outside bounds is unknown
    }
    
    int index = y * width + x;
    return occupancy_grid.data[index] == -1;
}

} // namespace orbit_planner
