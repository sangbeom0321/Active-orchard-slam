#include "aos/orbit_anchor_generator.hpp"

namespace orbit_planner {

OrbitAnchorGenerator::OrbitAnchorGenerator() 
    : orbit_radius_(2.0), orbit_spacing_(1.0), min_row_length_(5.0), max_orbit_distance_(50.0) {
}

void OrbitAnchorGenerator::setParameters(double orbit_radius, double orbit_spacing,
                                        double min_row_length, double max_orbit_distance) {
    orbit_radius_ = orbit_radius;
    orbit_spacing_ = orbit_spacing;
    min_row_length_ = min_row_length;
    max_orbit_distance_ = max_orbit_distance;
}

std::vector<OrbitAnchor> OrbitAnchorGenerator::generateOrbitAnchors(
    const std::vector<TreeRow>& tree_rows,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    
    std::vector<OrbitAnchor> anchors;
    
    for (const auto& row : tree_rows) {
        auto row_anchors = generateAnchorsForRow(row, occupancy_grid);
        anchors.insert(anchors.end(), row_anchors.begin(), row_anchors.end());
    }
    
    last_anchors_ = anchors;
    return anchors;
}

std::vector<OrbitSequence> OrbitAnchorGenerator::generateOrbitSequences(
    const std::vector<OrbitAnchor>& anchors,
    const geometry_msgs::msg::PoseStamped& robot_pose) {
    
    std::vector<OrbitSequence> sequences;
    
    // Group anchors by row
    std::unordered_map<int, std::vector<OrbitAnchor>> anchors_by_row;
    for (const auto& anchor : anchors) {
        anchors_by_row[anchor.row_id].push_back(anchor);
    }
    
    // Create sequences for each row
    for (const auto& [row_id, row_anchors] : anchors_by_row) {
        OrbitSequence sequence;
        sequence.row_id = row_id;
        sequence.anchors = solveTSP(row_anchors, robot_pose);
        sequence.total_cost = calculateTSPCost(sequence.anchors, robot_pose);
        sequence.total_utility = 0.0; // Calculate utility
        
        for (const auto& anchor : sequence.anchors) {
            sequence.total_utility += anchor.utility;
        }
        
        sequences.push_back(sequence);
    }
    
    last_sequences_ = sequences;
    return sequences;
}

std::vector<OrbitAnchor> OrbitAnchorGenerator::solveTSP(
    const std::vector<OrbitAnchor>& anchors,
    const geometry_msgs::msg::PoseStamped& start_pose) {
    
    if (anchors.empty()) {
        return {};
    }
    
    // Use nearest neighbor heuristic
    return nearestNeighborTSP(anchors, start_pose);
}

void OrbitAnchorGenerator::markAnchorAsVisited(const OrbitAnchor& anchor) {
    if (anchor.cluster_id >= 0 && anchor.cluster_id < visited_anchors_.size()) {
        visited_anchors_[anchor.cluster_id] = true;
    }
}

bool OrbitAnchorGenerator::isAnchorVisited(const OrbitAnchor& anchor) {
    if (anchor.cluster_id >= 0 && anchor.cluster_id < visited_anchors_.size()) {
        return visited_anchors_[anchor.cluster_id];
    }
    return false;
}

void OrbitAnchorGenerator::resetVisitedMarkers() {
    visited_anchors_.clear();
    visited_anchors_.resize(last_anchors_.size(), false);
}

std::vector<OrbitAnchor> OrbitAnchorGenerator::generateAnchorsForRow(
    const TreeRow& row,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    
    std::vector<OrbitAnchor> anchors;
    
    if (row.trees.empty()) {
        return anchors;
    }
    
    // Calculate row center and orientation
    geometry_msgs::msg::Point row_center = calculateRowCenter(row);
    double orientation = calculateRowOrientation(row);
    
    // Generate four orbit anchors around the row
    std::vector<OrbitType> types = {
        OrbitType::FRONT, OrbitType::BACK,
        OrbitType::LEFT, OrbitType::RIGHT
    };
    
    for (auto type : types) {
        geometry_msgs::msg::Point position;
        double yaw = orientation;
        
        switch (type) {
            case OrbitType::FRONT:
                position.x = row_center.x + orbit_radius_ * std::cos(orientation);
                position.y = row_center.y + orbit_radius_ * std::sin(orientation);
                break;
            case OrbitType::BACK:
                position.x = row_center.x - orbit_radius_ * std::cos(orientation);
                position.y = row_center.y - orbit_radius_ * std::sin(orientation);
                yaw += M_PI;
                break;
            case OrbitType::LEFT:
                position.x = row_center.x + orbit_radius_ * std::cos(orientation + M_PI/2);
                position.y = row_center.y + orbit_radius_ * std::sin(orientation + M_PI/2);
                yaw += M_PI/2;
                break;
            case OrbitType::RIGHT:
                position.x = row_center.x + orbit_radius_ * std::cos(orientation - M_PI/2);
                position.y = row_center.y + orbit_radius_ * std::sin(orientation - M_PI/2);
                yaw -= M_PI/2;
                break;
        }
        
        position.z = 0.0;
        
        if (isValidAnchorPosition(position, occupancy_grid)) {
            OrbitAnchor anchor = createOrbitAnchor(position, yaw, type, row.tree_count);
            anchor.utility = calculateAnchorUtility(anchor, row, geometry_msgs::msg::PoseStamped());
            anchors.push_back(anchor);
        }
    }
    
    return anchors;
}

OrbitAnchor OrbitAnchorGenerator::createOrbitAnchor(
    const geometry_msgs::msg::Point& position,
    double yaw,
    OrbitType type,
    int row_id) {
    
    OrbitAnchor anchor;
    anchor.position = position;
    anchor.yaw = yaw;
    anchor.type = type;
    anchor.row_id = row_id;
    anchor.utility = 0.0;
    anchor.visited = false;
    anchor.cluster_id = last_anchors_.size();
    
    return anchor;
}

bool OrbitAnchorGenerator::isValidAnchorPosition(
    const geometry_msgs::msg::Point& position,
    const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    
    if (!isPointInBounds(position, occupancy_grid)) {
        return false;
    }
    
    return isPointFree(position, occupancy_grid);
}

double OrbitAnchorGenerator::calculateAnchorUtility(
    const OrbitAnchor& anchor,
    const TreeRow& row,
    const geometry_msgs::msg::PoseStamped& robot_pose) {
    
    // Simple utility calculation based on row length and distance
    double row_utility = row.length / 10.0; // Normalize row length
    double distance_utility = 1.0 / (1.0 + calculateDistance(anchor.position, robot_pose.pose.position));
    
    return row_utility + distance_utility;
}

std::vector<OrbitAnchor> OrbitAnchorGenerator::nearestNeighborTSP(
    const std::vector<OrbitAnchor>& anchors,
    const geometry_msgs::msg::PoseStamped& start_pose) {
    
    if (anchors.empty()) {
        return {};
    }
    
    std::vector<OrbitAnchor> tour;
    std::vector<bool> visited(anchors.size(), false);
    
    // Start from the anchor closest to robot
    geometry_msgs::msg::Point current_pos = start_pose.pose.position;
    int current_index = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < anchors.size(); ++i) {
        double distance = calculateDistance(current_pos, anchors[i].position);
        if (distance < min_distance) {
            min_distance = distance;
            current_index = i;
        }
    }
    
    tour.push_back(anchors[current_index]);
    visited[current_index] = true;
    
    // Greedily select nearest unvisited anchor
    for (size_t i = 1; i < anchors.size(); ++i) {
        int next_index = -1;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t j = 0; j < anchors.size(); ++j) {
            if (visited[j]) continue;
            
            double distance = calculateDistance(tour.back().position, anchors[j].position);
            if (distance < min_distance) {
                min_distance = distance;
                next_index = j;
            }
        }
        
        if (next_index >= 0) {
            tour.push_back(anchors[next_index]);
            visited[next_index] = true;
        }
    }
    
    return tour;
}

std::vector<OrbitAnchor> OrbitAnchorGenerator::twoOptTSP(
    const std::vector<OrbitAnchor>& anchors,
    const geometry_msgs::msg::PoseStamped& start_pose) {
    
    // Placeholder for 2-opt improvement
    return nearestNeighborTSP(anchors, start_pose);
}

double OrbitAnchorGenerator::calculateTSPCost(
    const std::vector<OrbitAnchor>& sequence,
    const geometry_msgs::msg::PoseStamped& start_pose) {
    
    if (sequence.empty()) {
        return 0.0;
    }
    
    double total_cost = calculateDistance(start_pose.pose.position, sequence[0].position);
    
    for (size_t i = 1; i < sequence.size(); ++i) {
        total_cost += calculateDistance(sequence[i-1].position, sequence[i].position);
    }
    
    return total_cost;
}

double OrbitAnchorGenerator::calculateDistance(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double OrbitAnchorGenerator::calculateYawChange(
    const geometry_msgs::msg::PoseStamped& from,
    const geometry_msgs::msg::PoseStamped& to) {
    
    double yaw_from = std::atan2(2.0 * (from.pose.orientation.w * from.pose.orientation.z + 
                                       from.pose.orientation.x * from.pose.orientation.y),
                                1.0 - 2.0 * (from.pose.orientation.y * from.pose.orientation.y + 
                                           from.pose.orientation.z * from.pose.orientation.z));
    
    double yaw_to = std::atan2(2.0 * (to.pose.orientation.w * to.pose.orientation.z + 
                                     to.pose.orientation.x * to.pose.orientation.y),
                              1.0 - 2.0 * (to.pose.orientation.y * to.pose.orientation.y + 
                                         to.pose.orientation.z * to.pose.orientation.z));
    
    double yaw_diff = yaw_to - yaw_from;
    
    // Normalize to [-π, π]
    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    
    return std::abs(yaw_diff);
}

geometry_msgs::msg::Point OrbitAnchorGenerator::calculateRowCenter(const TreeRow& row) {
    if (row.trees.empty()) {
        return {};
    }
    
    double sum_x = 0, sum_y = 0;
    for (const auto& tree : row.trees) {
        sum_x += tree.center.x;
        sum_y += tree.center.y;
    }
    
    geometry_msgs::msg::Point center;
    center.x = sum_x / row.trees.size();
    center.y = sum_y / row.trees.size();
    center.z = 0.0;
    
    return center;
}

double OrbitAnchorGenerator::calculateRowOrientation(const TreeRow& row) {
    if (row.trees.size() < 2) {
        return 0.0;
    }
    
    // Use first and last trees to determine orientation
    const auto& first = row.trees.front().center;
    const auto& last = row.trees.back().center;
    
    return std::atan2(last.y - first.y, last.x - first.x);
}

geometry_msgs::msg::Point OrbitAnchorGenerator::calculateRowStart(const TreeRow& row) {
    if (row.trees.empty()) {
        return {};
    }
    
    geometry_msgs::msg::Point start = row.trees[0].center;
    for (const auto& tree : row.trees) {
        if (tree.center.x < start.x) {
            start = tree.center;
        }
    }
    return start;
}

geometry_msgs::msg::Point OrbitAnchorGenerator::calculateRowEnd(const TreeRow& row) {
    if (row.trees.empty()) {
        return {};
    }
    
    geometry_msgs::msg::Point end = row.trees[0].center;
    for (const auto& tree : row.trees) {
        if (tree.center.x > end.x) {
            end = tree.center;
        }
    }
    return end;
}

bool OrbitAnchorGenerator::isPointInBounds(const geometry_msgs::msg::Point& point,
                                          const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    double x = point.x - occupancy_grid.info.origin.position.x;
    double y = point.y - occupancy_grid.info.origin.position.y;
    
    int grid_x = static_cast<int>(x / occupancy_grid.info.resolution);
    int grid_y = static_cast<int>(y / occupancy_grid.info.resolution);
    
    return grid_x >= 0 && grid_x < occupancy_grid.info.width &&
           grid_y >= 0 && grid_y < occupancy_grid.info.height;
}

bool OrbitAnchorGenerator::isPointFree(const geometry_msgs::msg::Point& point,
                                      const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    double x = point.x - occupancy_grid.info.origin.position.x;
    double y = point.y - occupancy_grid.info.origin.position.y;
    
    int grid_x = static_cast<int>(x / occupancy_grid.info.resolution);
    int grid_y = static_cast<int>(y / occupancy_grid.info.resolution);
    
    if (grid_x < 0 || grid_x >= occupancy_grid.info.width ||
        grid_y < 0 || grid_y >= occupancy_grid.info.height) {
        return false;
    }
    
    int index = grid_y * occupancy_grid.info.width + grid_x;
    return occupancy_grid.data[index] == 0;
}

} // namespace orbit_planner
