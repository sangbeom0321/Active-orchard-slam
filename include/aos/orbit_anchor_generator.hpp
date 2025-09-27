/**
 * @file orbit_anchor_generator.hpp
 * @brief Orbit anchor generation for systematic row-by-row monitoring
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This class generates orbit anchors around detected tree rows for systematic
 * monitoring, ensuring both sides of every tree are observed.
 */

#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>
#include <memory>

#include "tree_clusterer.hpp"

namespace orbit_planner {

enum class OrbitType {
    FRONT = 0,
    BACK = 1,
    LEFT = 2,
    RIGHT = 3
};

struct OrbitAnchor {
    geometry_msgs::msg::Point position;
    double yaw;  // orientation to face the row
    OrbitType type;
    int row_id;
    int cluster_id;
    double utility;
    bool visited;
};

struct OrbitSequence {
    std::vector<OrbitAnchor> anchors;
    double total_cost;
    double total_utility;
    int row_id;
};

class OrbitAnchorGenerator {
public:
    OrbitAnchorGenerator();
    ~OrbitAnchorGenerator() = default;

    // Parameter setting
    void setParameters(double orbit_radius, double orbit_spacing,
                      double min_row_length, double max_orbit_distance);

    // Main generation functions
    std::vector<OrbitAnchor> generateOrbitAnchors(
        const std::vector<TreeRow>& tree_rows,
        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    std::vector<OrbitSequence> generateOrbitSequences(
        const std::vector<OrbitAnchor>& anchors,
        const geometry_msgs::msg::PoseStamped& robot_pose);

    // TSP-based sequencing
    std::vector<OrbitAnchor> solveTSP(
        const std::vector<OrbitAnchor>& anchors,
        const geometry_msgs::msg::PoseStamped& start_pose);

    // Utility functions
    void markAnchorAsVisited(const OrbitAnchor& anchor);
    bool isAnchorVisited(const OrbitAnchor& anchor);
    void resetVisitedMarkers();

    // Getters
    const std::vector<OrbitAnchor>& getLastAnchors() const { return last_anchors_; }
    const std::vector<OrbitSequence>& getLastSequences() const { return last_sequences_; }

private:
    // Parameters
    double orbit_radius_;
    double orbit_spacing_;
    double min_row_length_;
    double max_orbit_distance_;

    // Results
    std::vector<OrbitAnchor> last_anchors_;
    std::vector<OrbitSequence> last_sequences_;
    std::vector<bool> visited_anchors_;

    // Internal generation functions
    std::vector<OrbitAnchor> generateAnchorsForRow(
        const TreeRow& row,
        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    OrbitAnchor createOrbitAnchor(
        const geometry_msgs::msg::Point& position,
        double yaw,
        OrbitType type,
        int row_id);
    
    bool isValidAnchorPosition(
        const geometry_msgs::msg::Point& position,
        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    double calculateAnchorUtility(
        const OrbitAnchor& anchor,
        const TreeRow& row,
        const geometry_msgs::msg::PoseStamped& robot_pose);

    // TSP solving
    std::vector<OrbitAnchor> nearestNeighborTSP(
        const std::vector<OrbitAnchor>& anchors,
        const geometry_msgs::msg::PoseStamped& start_pose);
    
    std::vector<OrbitAnchor> twoOptTSP(
        const std::vector<OrbitAnchor>& anchors,
        const geometry_msgs::msg::PoseStamped& start_pose);
    
    double calculateTSPCost(
        const std::vector<OrbitAnchor>& sequence,
        const geometry_msgs::msg::PoseStamped& start_pose);
    
    double calculateDistance(
        const geometry_msgs::msg::Point& p1,
        const geometry_msgs::msg::Point& p2);
    
    double calculateYawChange(
        const geometry_msgs::msg::PoseStamped& from,
        const geometry_msgs::msg::PoseStamped& to);

    // Utility functions
    geometry_msgs::msg::Point calculateRowCenter(const TreeRow& row);
    double calculateRowOrientation(const TreeRow& row);
    geometry_msgs::msg::Point calculateRowStart(const TreeRow& row);
    geometry_msgs::msg::Point calculateRowEnd(const TreeRow& row);
    
    bool isPointInBounds(const geometry_msgs::msg::Point& point,
                        const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    bool isPointFree(const geometry_msgs::msg::Point& point,
                    const nav_msgs::msg::OccupancyGrid& occupancy_grid);
};

} // namespace orbit_planner
