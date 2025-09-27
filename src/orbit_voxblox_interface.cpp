#include "aos/orbit_voxblox_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace orbit_planner {

OrbitVoxbloxInterface::OrbitVoxbloxInterface() 
    : voxel_size_(0.1), truncation_distance_(0.2), max_weight_(100.0),
      bounds_initialized_(false) {
    min_bounds_.x = min_bounds_.y = min_bounds_.z = std::numeric_limits<double>::max();
    max_bounds_.x = max_bounds_.y = max_bounds_.z = std::numeric_limits<double>::lowest();
}

void OrbitVoxbloxInterface::initialize(double voxel_size, double truncation_distance) {
    voxel_size_ = voxel_size;
    truncation_distance_ = truncation_distance;
}

void OrbitVoxbloxInterface::integratePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const geometry_msgs::msg::PoseStamped& pose) {
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (const auto& point : cloud->points) {
        geometry_msgs::msg::Point world_point;
        world_point.x = point.x;
        world_point.y = point.y;
        world_point.z = point.z;
        
        updateBounds(world_point);
        
        std::string key = pointToKey(world_point);
        auto& voxel = tsdf_map_[key];
        
        // Simple TSDF integration
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        voxel.tsdf_value = std::min(voxel.tsdf_value, distance);
        voxel.weight = std::min(voxel.weight + 1.0, max_weight_);
        voxel.is_occupied = (distance < truncation_distance_);
    }
}

void OrbitVoxbloxInterface::updateESDF() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    computeESDF();
}

double OrbitVoxbloxInterface::getDistance(const geometry_msgs::msg::Point& point) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return interpolateDistance(point);
}

bool OrbitVoxbloxInterface::isFree(const geometry_msgs::msg::Point& point, double radius) {
    return getDistance(point) >= radius;
}

nav_msgs::msg::OccupancyGrid OrbitVoxbloxInterface::generateOccupancyGrid(
    const geometry_msgs::msg::Point& origin,
    double resolution,
    int width,
    int height) {
    
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = rclcpp::Clock().now();
    
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position = origin;
    grid.info.origin.orientation.w = 1.0;
    
    grid.data.resize(width * height, -1);
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            geometry_msgs::msg::Point world_point;
            world_point.x = origin.x + x * resolution;
            world_point.y = origin.y + y * resolution;
            world_point.z = 0.0;
            
            double distance = interpolateDistance(world_point);
            int index = y * width + x;
            
            if (distance < 0.5) {
                grid.data[index] = 100; // Occupied
            } else if (distance < 2.0) {
                grid.data[index] = 0;   // Free
            } else {
                grid.data[index] = -1;  // Unknown
            }
        }
    }
    
    return grid;
}

void OrbitVoxbloxInterface::getMapBounds(geometry_msgs::msg::Point& min_point, 
                                        geometry_msgs::msg::Point& max_point) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    min_point = min_bounds_;
    max_point = max_bounds_;
}

void OrbitVoxbloxInterface::setVoxelSize(double voxel_size) {
    voxel_size_ = voxel_size;
}

void OrbitVoxbloxInterface::setTruncationDistance(double truncation_distance) {
    truncation_distance_ = truncation_distance;
}

void OrbitVoxbloxInterface::setMaxWeight(double max_weight) {
    max_weight_ = max_weight;
}

std::string OrbitVoxbloxInterface::pointToKey(const geometry_msgs::msg::Point& point) {
    int x = static_cast<int>(std::floor(point.x / voxel_size_));
    int y = static_cast<int>(std::floor(point.y / voxel_size_));
    int z = static_cast<int>(std::floor(point.z / voxel_size_));
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
}

geometry_msgs::msg::Point OrbitVoxbloxInterface::keyToPoint(const std::string& key) {
    geometry_msgs::msg::Point point;
    // Simple parsing - in real implementation, use proper parsing
    point.x = point.y = point.z = 0.0;
    return point;
}

void OrbitVoxbloxInterface::updateBounds(const geometry_msgs::msg::Point& point) {
    if (!bounds_initialized_) {
        min_bounds_ = max_bounds_ = point;
        bounds_initialized_ = true;
    } else {
        min_bounds_.x = std::min(min_bounds_.x, point.x);
        min_bounds_.y = std::min(min_bounds_.y, point.y);
        min_bounds_.z = std::min(min_bounds_.z, point.z);
        max_bounds_.x = std::max(max_bounds_.x, point.x);
        max_bounds_.y = std::max(max_bounds_.y, point.y);
        max_bounds_.z = std::max(max_bounds_.z, point.z);
    }
}

void OrbitVoxbloxInterface::computeESDF() {
    // Voxblox-style ESDF computation with proper distance propagation
    // This implements a simplified version of the algorithm described in the Voxblox paper
    
    // Clear previous ESDF
    esdf_map_.clear();
    
    // Step 1: Initialize ESDF with TSDF information
    // In Voxblox, this is done by converting TSDF values to distance values
    for (const auto& [key, voxel] : tsdf_map_) {
        if (voxel.is_occupied) {
            esdf_map_[key] = 0.0;  // Occupied cells have distance 0
        } else if (voxel.tsdf_value < truncation_distance_) {
            // Near surface: use TSDF value as distance estimate
            esdf_map_[key] = voxel.tsdf_value;
        } else {
            // Far from surface: initialize with large distance
            esdf_map_[key] = truncation_distance_;
        }
    }
    
    // Step 2: Multi-pass distance propagation (simplified Fast Marching Method)
    // This is where Voxblox excels - efficient global distance propagation
    bool converged = false;
    int max_iterations = 10;  // Prevent infinite loops
    int iteration = 0;
    
    while (!converged && iteration < max_iterations) {
        converged = true;
        iteration++;
        
        // Forward pass: propagate distances from occupied cells outward
        for (const auto& [key, voxel] : tsdf_map_) {
            if (voxel.is_occupied) {
                if (propagateDistanceFromOccupied(key)) {
                    converged = false;
                }
            }
        }
        
        // Backward pass: ensure consistency
        for (const auto& [key, voxel] : tsdf_map_) {
            if (!voxel.is_occupied && esdf_map_.find(key) != esdf_map_.end()) {
                if (propagateDistanceFromFree(key)) {
                    converged = false;
                }
            }
        }
    }
}

bool OrbitVoxbloxInterface::propagateDistanceFromOccupied(const std::string& source_key) {
    // Parse source key to get coordinates
    size_t comma1 = source_key.find(',');
    size_t comma2 = source_key.find(',', comma1 + 1);
    
    if (comma1 == std::string::npos || comma2 == std::string::npos) {
        return false;
    }
    
    int source_x = std::stoi(source_key.substr(0, comma1));
    int source_y = std::stoi(source_key.substr(comma1 + 1, comma2 - comma1 - 1));
    int source_z = std::stoi(source_key.substr(comma2 + 1));
    
    bool updated = false;
    double source_distance = 0.0;  // Occupied cells have distance 0
    
    // 6-connected neighborhood (more efficient than 26-connected)
    int neighbors[6][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
    
    for (int i = 0; i < 6; ++i) {
        int target_x = source_x + neighbors[i][0];
        int target_y = source_y + neighbors[i][1];
        int target_z = source_z + neighbors[i][2];
        
        std::string target_key = std::to_string(target_x) + "," + 
                               std::to_string(target_y) + "," + 
                               std::to_string(target_z);
        
        double new_distance = source_distance + voxel_size_;
        
        // Update if this is a better (shorter) distance
        if (esdf_map_.find(target_key) != esdf_map_.end()) {
            if (new_distance < esdf_map_[target_key]) {
                esdf_map_[target_key] = new_distance;
                updated = true;
            }
        } else {
            // Create new entry for this cell
            esdf_map_[target_key] = new_distance;
            updated = true;
        }
    }
    
    return updated;
}

bool OrbitVoxbloxInterface::propagateDistanceFromFree(const std::string& source_key) {
    // Parse source key to get coordinates
    size_t comma1 = source_key.find(',');
    size_t comma2 = source_key.find(',', comma1 + 1);
    
    if (comma1 == std::string::npos || comma2 == std::string::npos) {
        return false;
    }
    
    int source_x = std::stoi(source_key.substr(0, comma1));
    int source_y = std::stoi(source_key.substr(comma1 + 1, comma2 - comma1 - 1));
    int source_z = std::stoi(source_key.substr(comma2 + 1));
    
    if (esdf_map_.find(source_key) == esdf_map_.end()) {
        return false;
    }
    
    double current_distance = esdf_map_[source_key];
    bool updated = false;
    
    // 6-connected neighborhood
    int neighbors[6][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
    
    for (int i = 0; i < 6; ++i) {
        int target_x = source_x + neighbors[i][0];
        int target_y = source_y + neighbors[i][1];
        int target_z = source_z + neighbors[i][2];
        
        std::string target_key = std::to_string(target_x) + "," + 
                               std::to_string(target_y) + "," + 
                               std::to_string(target_z);
        
        if (esdf_map_.find(target_key) != esdf_map_.end()) {
            double neighbor_distance = esdf_map_[target_key];
            double new_distance = neighbor_distance + voxel_size_;
            
            if (new_distance < current_distance) {
                esdf_map_[source_key] = new_distance;
                updated = true;
                break;  // Only update once per iteration
            }
        }
    }
    
    return updated;
}

void OrbitVoxbloxInterface::propagateDistance(const std::string& source_key, double source_distance) {
    // Parse source key to get coordinates
    size_t comma1 = source_key.find(',');
    size_t comma2 = source_key.find(',', comma1 + 1);
    
    if (comma1 == std::string::npos || comma2 == std::string::npos) {
        return;
    }
    
    int source_x = std::stoi(source_key.substr(0, comma1));
    int source_y = std::stoi(source_key.substr(comma1 + 1, comma2 - comma1 - 1));
    int source_z = std::stoi(source_key.substr(comma2 + 1));
    
    // Define search radius (in voxel units)
    int search_radius = static_cast<int>(10.0 / voxel_size_);  // 10m search radius
    
    // Propagate distance to nearby cells
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
            for (int dz = -search_radius; dz <= search_radius; ++dz) {
                int target_x = source_x + dx;
                int target_y = source_y + dy;
                int target_z = source_z + dz;
                
                std::string target_key = std::to_string(target_x) + "," + 
                                       std::to_string(target_y) + "," + 
                                       std::to_string(target_z);
                
                // Calculate Euclidean distance
                double distance = std::sqrt(dx * dx + dy * dy + dz * dz) * voxel_size_;
                double new_distance = source_distance + distance;
                
                // Update ESDF if this is a better (shorter) distance
                if (esdf_map_.find(target_key) != esdf_map_.end()) {
                    if (new_distance < esdf_map_[target_key]) {
                        esdf_map_[target_key] = new_distance;
                    }
                } else {
                    // Create new entry for this cell
                    esdf_map_[target_key] = new_distance;
                }
            }
        }
    }
}

double OrbitVoxbloxInterface::interpolateDistance(const geometry_msgs::msg::Point& point) {
    std::string key = pointToKey(point);
    
    if (esdf_map_.find(key) != esdf_map_.end()) {
        return esdf_map_[key];
    }
    
    // If exact cell not found, try to find nearby cells for interpolation
    // Parse key to get coordinates
    size_t comma1 = key.find(',');
    size_t comma2 = key.find(',', comma1 + 1);
    
    if (comma1 == std::string::npos || comma2 == std::string::npos) {
        return 5.0; // Default distance for unknown areas
    }
    
    int center_x = std::stoi(key.substr(0, comma1));
    int center_y = std::stoi(key.substr(comma1 + 1, comma2 - comma1 - 1));
    int center_z = std::stoi(key.substr(comma2 + 1));
    
    // Search in a small neighborhood for interpolation
    double min_distance = std::numeric_limits<double>::max();
    bool found_nearby = false;
    
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dz = -2; dz <= 2; ++dz) {
                int search_x = center_x + dx;
                int search_y = center_y + dy;
                int search_z = center_z + dz;
                
                std::string search_key = std::to_string(search_x) + "," + 
                                       std::to_string(search_y) + "," + 
                                       std::to_string(search_z);
                
                if (esdf_map_.find(search_key) != esdf_map_.end()) {
                    // Calculate distance to this cell
                    double cell_distance = std::sqrt(dx * dx + dy * dy + dz * dz) * voxel_size_;
                    double total_distance = esdf_map_[search_key] + cell_distance;
                    
                    if (total_distance < min_distance) {
                        min_distance = total_distance;
                        found_nearby = true;
                    }
                }
            }
        }
    }
    
    if (found_nearby) {
        return min_distance;
    }
    
    return 5.0; // Default distance for unknown areas
}

} // namespace orbit_planner