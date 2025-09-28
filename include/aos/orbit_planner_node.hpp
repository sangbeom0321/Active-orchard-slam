/**
 * @file orbit_planner_node.hpp
 * @brief Main planner node for autonomous orchard exploration
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This node implements autonomous exploration for orchard environments using
 * frontier-based exploration with tree row detection and ESDF-based path planning.
 * 
 * Key features:
 * - Tree clustering and row detection
 * - Frontier detection and candidate generation
 * - Orbit anchor generation for systematic monitoring
 * - Receding-horizon scheduling
 * - ESDF-based path planning
 * - RViz integration for area specification
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include "orbit_voxblox_interface.hpp"
#include "tree_clusterer.hpp"
#include "frontier_detector.hpp"
#include "path_planner.hpp"
#include "orbit_anchor_generator.hpp"

namespace orbit_planner {

struct ExplorationState {
    bool is_exploring = false;
    bool area_defined = false;
    bool start_pose_defined = false;
    bool robot_pose_defined = false;
    geometry_msgs::msg::PolygonStamped exploration_area;
    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped current_robot_pose;
    std::vector<geometry_msgs::msg::Point> visited_goals;
    std::vector<geometry_msgs::msg::Point> current_frontiers;
    std::vector<geometry_msgs::msg::Point> current_orbit_anchors;
    geometry_msgs::msg::PoseStamped current_goal;
    nav_msgs::msg::Path current_path;
};

class OrbitPlannerNode : public rclcpp::Node {
public:
    OrbitPlannerNode();
    ~OrbitPlannerNode();

private:
    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr area_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_height_band_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parameter_update_sub_;
    
    // ROS2 Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontiers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr orbit_anchors_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visited_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trees_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_rows_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_height_band_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pcd_occupancy_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr z_filtered_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensor_marker_pub_;
    
    // ROS2 Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_exploration_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_exploration_srv_;
    
    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Core Components
    std::unique_ptr<OrbitVoxbloxInterface> voxblox_interface_;
    std::unique_ptr<TreeClusterer> tree_clusterer_;
    std::unique_ptr<FrontierDetector> frontier_detector_;
    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<OrbitAnchorGenerator> orbit_anchor_generator_;
    
    // State Management
    ExplorationState state_;
    std::vector<TreeRow> current_tree_rows_;
    std::mutex state_mutex_;
    
    // Threading
    std::thread exploration_thread_;
    std::atomic<bool> should_explore_;
    
    // Parameters
    struct Parameters {
        // Update rates
        double map_update_rate = 2.0;
        double planning_rate = 1.0;
        
        // Robot parameters
        double robot_radius = 0.4;
        double safety_margin = 0.1;
        
        // Exploration parameters
        double max_planning_distance = 50.0;
        double frontier_cluster_min_size = 5.0;
        double frontier_cluster_max_distance = 10.0;
        double goal_tolerance = 1.0;
        
        // Cost weights
        double yaw_change_weight = 0.5;
        double frontier_gain_weight = 1.0;
        double distance_weight = 1.0;
        
        // Tree detection parameters
        double tree_height_min = 0.4;
        double tree_height_max = 0.7;
        double tree_cluster_tolerance = 0.5;
        int tree_min_cluster_size = 10;
        // Row detection parameters
        double row_tolerance = 2.0;          // neighbor linking threshold (m)
        double row_min_length = 5.0;         // minimum row length (m)
        int row_min_trees = 3;               // minimum trees per row
        
        // Radius search clustering parameters
        double center_search_radius = 0.15;  // radius for finding tree centers (m)
        double clustering_radius = 0.8;      // radius for clustering filtering (m)
        int min_neighbors_in_radius = 0;     // minimum neighbors for clustering
        
        // Orbit anchor parameters
        double orbit_radius = 2.0;
        double orbit_spacing = 1.0;
        
        // Path planning parameters
        double path_resolution = 0.1;
        double path_smoothing_factor = 0.5;
    } params_;
    
    // Callbacks
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void areaCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void startPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void filteredHeightBandCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void parameterUpdateCallback(const std_msgs::msg::String::SharedPtr msg);
    
    void startExplorationCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void stopExplorationCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    // Core Functions
    void explorationLoop();
    void updateMap();
    void generateCandidates();
    void selectNextGoal();
    void planPath();
    void publishVisualization();
    
    // Utility Functions
    bool isPointInPolygon(const geometry_msgs::msg::Point& point, 
                         const geometry_msgs::msg::PolygonStamped& polygon);
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
    double calculateYawChange(const geometry_msgs::msg::PoseStamped& from, 
                            const geometry_msgs::msg::PoseStamped& to);
    
    // Visualization
    void publishFrontiers();
    void publishOrbitAnchors();
    void publishVisitedGoals();
    void publishOccupancyGrid();
    void publishTreeVisualization(const std::vector<TreeCluster>& tree_clusters);
    void publishTreeRowsVisualization(const std::vector<TreeRow>& tree_rows);
    void publishFilteredHeightBandVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud);
    
    // PCD to Occupancy Grid conversion
    nav_msgs::msg::OccupancyGrid convertPCDToOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    nav_msgs::msg::OccupancyGrid convertPCDToOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<TreeRow>& tree_rows);
    
    // 2D Ray casting for occupancy mapping
    std::vector<std::pair<int, int>> raycast2D(int x0, int y0, int x1, int y1);
    void addTreeToOccupancyGrid(nav_msgs::msg::OccupancyGrid& occupancy_grid, const TreeCluster& tree, double resolution);
    void addRowToOccupancyGrid(nav_msgs::msg::OccupancyGrid& occupancy_grid, const TreeRow& row, double resolution);
    
    // Distance calculation utilities
    double distancePointToLineSegment(double px, double py, double x1, double y1, double x2, double y2);
    
    // Visualization utilities
    void publishSensorMarker(const geometry_msgs::msg::PoseStamped& sensor_pose);
    
    // Parameter Loading
    void loadParameters();
};

} // namespace orbit_planner