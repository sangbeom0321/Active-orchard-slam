/**
 * @file orbit_voxblox_interface.hpp
 * @brief Voxblox interface for TSDF/ESDF mapping
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This class provides an interface to Voxblox for TSDF and ESDF mapping.
 * It handles point cloud integration and provides distance queries for path planning.
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <mutex>
#include <vector>

namespace orbit_planner {

class OrbitVoxbloxInterface {
public:
    OrbitVoxbloxInterface();
    ~OrbitVoxbloxInterface() = default;

    // Initialization
    void initialize(double voxel_size = 0.1, double truncation_distance = 0.2);
    
    // Point cloud integration
    void integratePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           const geometry_msgs::msg::PoseStamped& pose);
    
    // ESDF update
    void updateESDF();
    
    // Distance queries
    double getDistance(const geometry_msgs::msg::Point& point);
    bool isFree(const geometry_msgs::msg::Point& point, double radius = 0.5);
    
    // Occupancy grid generation
    nav_msgs::msg::OccupancyGrid generateOccupancyGrid(
        const geometry_msgs::msg::Point& origin,
        double resolution = 0.1,
        int width = 1000,
        int height = 1000);
    
    // Map bounds
    void getMapBounds(geometry_msgs::msg::Point& min_point, 
                     geometry_msgs::msg::Point& max_point);
    
    // Parameters
    void setVoxelSize(double voxel_size);
    void setTruncationDistance(double truncation_distance);
    void setMaxWeight(double max_weight);
    
private:
    // Voxblox layers (simplified implementation)
    struct Voxel {
        double tsdf_value = 0.0;
        double weight = 0.0;
        bool is_occupied = false;
    };
    
    // Map parameters
    double voxel_size_;
    double truncation_distance_;
    double max_weight_;
    
    // Map data
    std::unordered_map<std::string, Voxel> tsdf_map_;
    std::unordered_map<std::string, double> esdf_map_;
    
    // Thread safety
    std::mutex map_mutex_;
    
    // Map bounds
    geometry_msgs::msg::Point min_bounds_;
    geometry_msgs::msg::Point max_bounds_;
    bool bounds_initialized_;
    
    // Utility functions
    std::string pointToKey(const geometry_msgs::msg::Point& point);
    geometry_msgs::msg::Point keyToPoint(const std::string& key);
    void updateBounds(const geometry_msgs::msg::Point& point);
    void computeESDF();
    void propagateDistance(const std::string& source_key, double source_distance);
    bool propagateDistanceFromOccupied(const std::string& source_key);
    bool propagateDistanceFromFree(const std::string& source_key);
    double interpolateDistance(const geometry_msgs::msg::Point& point);
};

} // namespace orbit_planner