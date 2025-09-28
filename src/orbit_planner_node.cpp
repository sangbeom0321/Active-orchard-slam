#include "aos/orbit_planner_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace orbit_planner {

OrbitPlannerNode::OrbitPlannerNode() 
    : Node("orbit_planner_node"),
      should_explore_(false) {
    
    // Load parameters
    loadParameters();
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize core components
    voxblox_interface_ = std::make_unique<OrbitVoxbloxInterface>();
    tree_clusterer_ = std::make_unique<TreeClusterer>();
    frontier_detector_ = std::make_unique<FrontierDetector>();
    path_planner_ = std::make_unique<PathPlanner>();
    orbit_anchor_generator_ = std::make_unique<OrbitAnchorGenerator>();
    
    // Initialize voxblox interface
    voxblox_interface_->initialize(params_.robot_radius, params_.safety_margin);
    
    // Set parameters for components
    tree_clusterer_->setParameters(
        params_.tree_height_min, params_.tree_height_max,
        params_.tree_cluster_tolerance, params_.tree_min_cluster_size,
        0.1, params_.row_tolerance, params_.row_min_length, params_.row_min_trees,
        params_.center_search_radius, params_.clustering_radius, params_.min_neighbors_in_radius);
    
    frontier_detector_->setParameters(
        params_.robot_radius, static_cast<int>(params_.frontier_cluster_min_size),
        params_.max_planning_distance, params_.frontier_gain_weight);
    
    path_planner_->setParameters(
        params_.robot_radius, params_.safety_margin,
        params_.path_resolution, params_.max_planning_distance);
    
    orbit_anchor_generator_->setParameters(
        params_.orbit_radius, params_.orbit_spacing,
        5.0, params_.max_planning_distance);
    
    // Create subscribers
    // Initialize subscribers with appropriate QoS
    
    // Create QoS profile for point cloud data (reliable, keep last 10 messages)
    rclcpp::QoS pointcloud_qos(10);
    pointcloud_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    pointcloud_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    pointcloud_qos.history(rclcpp::HistoryPolicy::KeepLast);
    
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        // "/lio_sam/mapping/map_global", pointcloud_qos,
        "/accumulated_map", pointcloud_qos,
        std::bind(&OrbitPlannerNode::pointCloudCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Point cloud subscriber created for topic: /accumulated_map");
    
    // Create QoS profile for odometry (best effort, keep last 10 messages)
    rclcpp::QoS odometry_qos(10);
    odometry_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    odometry_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    odometry_qos.history(rclcpp::HistoryPolicy::KeepLast);
    
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/lio_sam/mapping/odometry", odometry_qos,
        std::bind(&OrbitPlannerNode::poseCallback, this, std::placeholders::_1));
    
    area_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/orbit_planner/exploration_area", 10,
        std::bind(&OrbitPlannerNode::areaCallback, this, std::placeholders::_1));
    
    // Subscribe to filtered height band point cloud
    filtered_height_band_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/orbit_planner/z_filtered", pointcloud_qos,
        std::bind(&OrbitPlannerNode::filteredHeightBandCallback, this, std::placeholders::_1));
    
    // Subscribe to parameter updates from panel
    parameter_update_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/orbit_planner/parameter_update", 10,
        std::bind(&OrbitPlannerNode::parameterUpdateCallback, this, std::placeholders::_1));
    
  // Create publishers
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/orbit_planner/trajectory", 10);
    
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/orbit_planner/goal", 10);
    
    frontiers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/frontiers", 10);
    
    orbit_anchors_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/orbit_anchors", 10);
    
    visited_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/visited", 10);
    
    occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/orbit_planner/occupancy", 10);
    
    trees_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/trees", 10);
    
    tree_rows_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/tree_rows", 10);
    
    filtered_height_band_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orbit_planner/filtered_height_band", 10);
    
    pcd_occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/orbit_planner/pcd_occupancy", 10);
    
    z_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orbit_planner/z_filtered", 10);
    
    sensor_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/orbit_planner/sensor_marker", 10);

    // Create services
    start_exploration_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/orbit_planner/start_exploration",
            std::bind(&OrbitPlannerNode::startExplorationCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
    
    stop_exploration_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/orbit_planner/stop_exploration",
            std::bind(&OrbitPlannerNode::stopExplorationCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
    
    // Start exploration thread (but exploration is disabled by default)
    should_explore_ = true;
    exploration_thread_ = std::thread(&OrbitPlannerNode::explorationLoop, this);
    
    RCLCPP_INFO(this->get_logger(), "Orbit Planner Node initialized");
}

OrbitPlannerNode::~OrbitPlannerNode() {
    should_explore_ = false;
    if (exploration_thread_.joinable()) {
        exploration_thread_.join();
    }
}

void OrbitPlannerNode::loadParameters() {
    // Update rates
    this->declare_parameter("map_update_rate", params_.map_update_rate);
    this->declare_parameter("planning_rate", params_.planning_rate);
    
    // Robot parameters
    this->declare_parameter("robot_radius", params_.robot_radius);
    this->declare_parameter("safety_margin", params_.safety_margin);
    
    // Exploration parameters
    this->declare_parameter("max_planning_distance", params_.max_planning_distance);
    this->declare_parameter("frontier_cluster_min_size", params_.frontier_cluster_min_size);
    this->declare_parameter("frontier_cluster_max_distance", params_.frontier_cluster_max_distance);
    this->declare_parameter("goal_tolerance", params_.goal_tolerance);
    
    // Cost weights
    this->declare_parameter("yaw_change_weight", params_.yaw_change_weight);
    this->declare_parameter("frontier_gain_weight", params_.frontier_gain_weight);
    this->declare_parameter("distance_weight", params_.distance_weight);
    
    // Tree detection parameters
    this->declare_parameter("tree_height_min", params_.tree_height_min);
    this->declare_parameter("tree_height_max", params_.tree_height_max);
    this->declare_parameter("tree_cluster_tolerance", params_.tree_cluster_tolerance);
    this->declare_parameter("tree_min_cluster_size", params_.tree_min_cluster_size);
    // Row detection parameters
    this->declare_parameter("row_tolerance", params_.row_tolerance);
    this->declare_parameter("row_min_length", params_.row_min_length);
    this->declare_parameter("row_min_trees", params_.row_min_trees);
    
    // Radius search clustering parameters
    this->declare_parameter("center_search_radius", params_.center_search_radius);
    this->declare_parameter("clustering_radius", params_.clustering_radius);
    this->declare_parameter("min_neighbors_in_radius", params_.min_neighbors_in_radius);
    
    // Orbit anchor parameters
    this->declare_parameter("orbit_radius", params_.orbit_radius);
    this->declare_parameter("orbit_spacing", params_.orbit_spacing);
    
    // Path planning parameters
    this->declare_parameter("path_resolution", params_.path_resolution);
    this->declare_parameter("path_smoothing_factor", params_.path_smoothing_factor);
    
    // Load values
    params_.map_update_rate = this->get_parameter("map_update_rate").as_double();
    params_.planning_rate = this->get_parameter("planning_rate").as_double();
    params_.robot_radius = this->get_parameter("robot_radius").as_double();
    params_.safety_margin = this->get_parameter("safety_margin").as_double();
    params_.max_planning_distance = this->get_parameter("max_planning_distance").as_double();
    params_.frontier_cluster_min_size = this->get_parameter("frontier_cluster_min_size").as_double();
    params_.frontier_cluster_max_distance = this->get_parameter("frontier_cluster_max_distance").as_double();
    params_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    params_.yaw_change_weight = this->get_parameter("yaw_change_weight").as_double();
    params_.frontier_gain_weight = this->get_parameter("frontier_gain_weight").as_double();
    params_.distance_weight = this->get_parameter("distance_weight").as_double();
    params_.tree_height_min = this->get_parameter("tree_height_min").as_double();
    params_.tree_height_max = this->get_parameter("tree_height_max").as_double();
    params_.tree_cluster_tolerance = this->get_parameter("tree_cluster_tolerance").as_double();
    params_.tree_min_cluster_size = this->get_parameter("tree_min_cluster_size").as_int();
    params_.row_tolerance = this->get_parameter("row_tolerance").as_double();
    params_.row_min_length = this->get_parameter("row_min_length").as_double();
    params_.row_min_trees = this->get_parameter("row_min_trees").as_int();
    params_.center_search_radius = this->get_parameter("center_search_radius").as_double();
    params_.clustering_radius = this->get_parameter("clustering_radius").as_double();
    params_.min_neighbors_in_radius = this->get_parameter("min_neighbors_in_radius").as_int();
    params_.orbit_radius = this->get_parameter("orbit_radius").as_double();
    params_.orbit_spacing = this->get_parameter("orbit_spacing").as_double();
    params_.path_resolution = this->get_parameter("path_resolution").as_double();
    params_.path_smoothing_factor = this->get_parameter("path_smoothing_factor").as_double();
}

void OrbitPlannerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Check if exploration is active
    {
        if (!state_.is_exploring) {
            return; // Skip processing if not exploring
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Point cloud callback triggered - received %u points", msg->width * msg->height);
    
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    RCLCPP_INFO(this->get_logger(), "Point cloud received with %zu points", cloud->size());

    // Z-band filtering (independent of exploration area)
    pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    double z_min = 0.2; // Trunk band minimum height
    double z_max = 1.2; // Trunk band maximum height
    
    for (const auto& point : cloud->points) {
        if (point.z >= z_min && point.z <= z_max) {
            pcl::PointXYZ filtered_point;
            filtered_point.x = point.x;
            filtered_point.y = point.y;
            filtered_point.z = 0.0; // Project to 2D plane
            z_filtered_cloud->points.push_back(filtered_point);
        }
    }
    z_filtered_cloud->width = z_filtered_cloud->points.size();
    z_filtered_cloud->height = 1;
    z_filtered_cloud->is_dense = true;
    
    // Publish Z-filtered point cloud
    if (!z_filtered_cloud->empty()) {
        sensor_msgs::msg::PointCloud2 z_filtered_msg;
        pcl::toROSMsg(*z_filtered_cloud, z_filtered_msg);
        z_filtered_msg.header = msg->header; // Keep original header
        z_filtered_pub_->publish(z_filtered_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published Z-filtered cloud with %zu points", z_filtered_cloud->size());
    } else {
        RCLCPP_WARN(this->get_logger(), "Z-filtered cloud is empty");
    }

    // Filter points within exploration area
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    {
    
        if (state_.area_defined) {
            for (const auto& point : cloud->points) {
                geometry_msgs::msg::Point geo_point;
                geo_point.x = point.x;
                geo_point.y = point.y;
                geo_point.z = point.z;
                
                if (isPointInPolygon(geo_point, state_.exploration_area)) {
                    filtered_cloud->points.push_back(point);
                }
            }
            filtered_cloud->width = filtered_cloud->points.size();
            filtered_cloud->height = 1;
            filtered_cloud->is_dense = true;
            
            RCLCPP_INFO(this->get_logger(), "Filtered to %zu points within exploration area", filtered_cloud->size());
        } else {
            RCLCPP_WARN(this->get_logger(), "Exploration area not defined, using all points");
            filtered_cloud = cloud;
        }
    }
    auto tree_clusters = tree_clusterer_->detectTrees(filtered_cloud);

    // Integrate point cloud
    // voxblox_interface_->integratePointCloud(filtered_cloud, robot_pose);
    
    if (!tree_clusters.empty()) {
        RCLCPP_INFO(this->get_logger(), "Detected %zu trees", tree_clusters.size());
        
        // Publish tree visualization
        publishTreeVisualization(tree_clusters);
        
        // Detect tree rows
        auto tree_rows = tree_clusterer_->detectRows(tree_clusters);
        if (!tree_rows.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected %zu tree rows", tree_rows.size());
            
            // Store tree rows for occupancy grid generation
            current_tree_rows_ = tree_rows;
            
            // Publish tree rows visualization
            publishTreeRowsVisualization(tree_rows);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Tree detection complete");
}

void OrbitPlannerNode::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Check if exploration is active
    {
        if (!state_.is_exploring) {
            return; // Skip processing if not exploring
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Odometry callback triggered");
    
    // Convert Odometry to PoseStamped
    {
        state_.current_robot_pose.header = msg->header;
        state_.current_robot_pose.pose = msg->pose.pose;
        state_.robot_pose_defined = true;
    }
    
    RCLCPP_INFO(this->get_logger(), "Robot pose updated: (%.2f, %.2f, %.2f)", 
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void OrbitPlannerNode::areaCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {

    state_.exploration_area = *msg;
    state_.area_defined = true;
    RCLCPP_INFO(this->get_logger(), "Exploration area received");
}

void OrbitPlannerNode::startExplorationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    //
    
    if (!state_.area_defined) {
        RCLCPP_WARN(this->get_logger(), "Exploration area not defined");
        return;
    }

    state_.is_exploring = true;
    RCLCPP_INFO(this->get_logger(), "Exploration started with start pose: (%.2f, %.2f)", 
                state_.start_pose.pose.position.x, state_.start_pose.pose.position.y);
}

void OrbitPlannerNode::stopExplorationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    

    state_.is_exploring = false;
    RCLCPP_INFO(this->get_logger(), "Exploration stopped");
}

void OrbitPlannerNode::explorationLoop() {
    rclcpp::Rate rate(params_.planning_rate);
    
    while (rclcpp::ok() && should_explore_) {
        {
        
            if (!state_.is_exploring || !state_.area_defined) {
                rate.sleep();
                continue;
            }
        }
        
        // Tree detection will be performed in pointCloudCallback
        // This ensures we have the latest point cloud data
        
        // Update map
        // updateMap();
        
        // // Generate candidates
        // generateCandidates();
        
        // // Select next goal
        // selectNextGoal();
        
        // // Plan path
        // planPath();
        
        // // Publish visualization
        // publishVisualization();
        
        rate.sleep();
    }
}

void OrbitPlannerNode::updateMap() {
    // Update ESDF
    voxblox_interface_->updateESDF();
    
    // Generate occupancy grid
    geometry_msgs::msg::Point origin;
    origin.x = -50.0;
    origin.y = -50.0;
    origin.z = 0.0;
    
    auto occupancy_grid = voxblox_interface_->generateOccupancyGrid(origin, 0.1, 1000, 1000);
    occupancy_pub_->publish(occupancy_grid);
}

void OrbitPlannerNode::generateCandidates() {
    // Get current robot pose from state
    geometry_msgs::msg::PoseStamped robot_pose;
    {
    
        if (!state_.robot_pose_defined) {
            RCLCPP_WARN(this->get_logger(), "Robot pose not available for candidate generation");
            return;
        }
        robot_pose = state_.current_robot_pose;
        robot_pose.header.stamp = this->now();
    }
    
    // Generate occupancy grid for frontier detection
    geometry_msgs::msg::Point origin;
    origin.x = -50.0;
    origin.y = -50.0;
    origin.z = 0.0;
    
    auto occupancy_grid = voxblox_interface_->generateOccupancyGrid(origin, 0.1, 1000, 1000);
    
    // Detect frontiers
    auto frontiers = frontier_detector_->detectFrontiers(occupancy_grid, robot_pose);
    
    // Filter frontiers within exploration area
    std::vector<Frontier> valid_frontiers;
    for (const auto& frontier : frontiers) {
        if (isPointInPolygon(frontier.center, state_.exploration_area)) {
            valid_frontiers.push_back(frontier);
        }
    }
    
    // Update state

    state_.current_frontiers.clear();
    for (const auto& frontier : valid_frontiers) {
        state_.current_frontiers.push_back(frontier.center);
    }
}

void OrbitPlannerNode::selectNextGoal() {

    
    if (state_.current_frontiers.empty()) {
        RCLCPP_INFO(this->get_logger(), "No more frontiers to explore");
        state_.is_exploring = false;
        return;
    }
    
    // Simple selection: choose closest frontier
    geometry_msgs::msg::PoseStamped robot_pose;
    {
    
        if (!state_.robot_pose_defined) {
            RCLCPP_WARN(this->get_logger(), "Robot pose not available for goal selection");
            return;
        }
        robot_pose = state_.current_robot_pose;
        robot_pose.header.stamp = this->now();
    }
    
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point best_goal;
    
    for (const auto& frontier : state_.current_frontiers) {
        double distance = calculateDistance(robot_pose.pose.position, frontier);
        if (distance < min_distance) {
            min_distance = distance;
            best_goal = frontier;
        }
    }
    
    // Create goal pose
    state_.current_goal.header.frame_id = "map";
    state_.current_goal.header.stamp = this->now();
    state_.current_goal.pose.position = best_goal;
    state_.current_goal.pose.orientation.w = 1.0;
    
    // Mark as visited
    state_.visited_goals.push_back(best_goal);
}

void OrbitPlannerNode::planPath() {

    
    if (state_.current_goal.pose.position.x == 0.0 && 
        state_.current_goal.pose.position.y == 0.0) {
        return;
    }
    
    // Get robot pose from state
    geometry_msgs::msg::PoseStamped robot_pose;
    {
    
        if (!state_.robot_pose_defined) {
            RCLCPP_WARN(this->get_logger(), "Robot pose not available for path planning");
            return;
        }
        robot_pose = state_.current_robot_pose;
        robot_pose.header.stamp = this->now();
    }
    
    // Generate occupancy grid for path planning
    geometry_msgs::msg::Point origin;
    origin.x = -50.0;
    origin.y = -50.0;
    origin.z = 0.0;
    
    auto occupancy_grid = voxblox_interface_->generateOccupancyGrid(origin, 0.1, 1000, 1000);
    
    // Plan path
    auto path = path_planner_->planPath(robot_pose, state_.current_goal, occupancy_grid);
    
    if (!path.poses.empty()) {
        state_.current_path = path;
        trajectory_pub_->publish(path);
        goal_pub_->publish(state_.current_goal);
    }
}

void OrbitPlannerNode::publishVisualization() {
    publishFrontiers();
    publishVisitedGoals();
}

void OrbitPlannerNode::publishFrontiers() {
    visualization_msgs::msg::MarkerArray marker_array;
    

    
    for (size_t i = 0; i < state_.current_frontiers.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "frontiers";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = state_.current_frontiers[i];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker_array.markers.push_back(marker);
    }
    
    frontiers_pub_->publish(marker_array);
}

void OrbitPlannerNode::publishVisitedGoals() {
    visualization_msgs::msg::MarkerArray marker_array;
    

    
    for (size_t i = 0; i < state_.visited_goals.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "visited";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = state_.visited_goals[i];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.6;
        
        marker_array.markers.push_back(marker);
    }
    
    visited_pub_->publish(marker_array);
}

bool OrbitPlannerNode::isPointInPolygon(const geometry_msgs::msg::Point& point, 
                                       const geometry_msgs::msg::PolygonStamped& polygon) {
    // Simple point-in-polygon test using ray casting
  int crossings = 0;
    const auto& points = polygon.polygon.points;
    
    for (size_t i = 0; i < points.size(); ++i) {
        size_t j = (i + 1) % points.size();
        
        if (((points[i].y <= point.y) && (point.y < points[j].y)) ||
            ((points[j].y <= point.y) && (point.y < points[i].y))) {
            
            double x = points[i].x + (point.y - points[i].y) / (points[j].y - points[i].y) * 
                      (points[j].x - points[i].x);
            
            if (point.x < x) {
        crossings++;
      }
    }
  }
  
  return (crossings % 2) == 1;
}

double OrbitPlannerNode::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                          const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double OrbitPlannerNode::calculateYawChange(const geometry_msgs::msg::PoseStamped& from, 
                                           const geometry_msgs::msg::PoseStamped& to) {
    // Calculate yaw change between two poses
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

void OrbitPlannerNode::publishTreeVisualization(const std::vector<TreeCluster>& tree_clusters) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Create tree markers
    for (size_t i = 0; i < tree_clusters.size(); ++i) {
        const auto& cluster = tree_clusters[i];
        
        // Tree trunk marker (cylinder)
        visualization_msgs::msg::Marker trunk_marker;
        trunk_marker.header.frame_id = "map";
        trunk_marker.header.stamp = this->get_clock()->now();
        trunk_marker.ns = "trees";
        trunk_marker.id = i * 2; // Even IDs for trunks
        trunk_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        trunk_marker.action = visualization_msgs::msg::Marker::ADD;
        
        trunk_marker.pose.position.x = cluster.center.x;
        trunk_marker.pose.position.y = cluster.center.y;
        trunk_marker.pose.position.z = cluster.center.z + cluster.height / 2.0;
        trunk_marker.pose.orientation.w = 1.0;
        
        trunk_marker.scale.x = 0.3; // Fixed small diameter
        trunk_marker.scale.y = 0.3; // Fixed small diameter
        trunk_marker.scale.z = 0.5; // Fixed small height
        
        trunk_marker.color.r = 0.4; // Brown color
        trunk_marker.color.g = 0.2;
        trunk_marker.color.b = 0.0;
        trunk_marker.color.a = 0.8;
        
        marker_array.markers.push_back(trunk_marker);
        
        // Tree center marker (sphere)
        visualization_msgs::msg::Marker center_marker;
        center_marker.header.frame_id = "map";
        center_marker.header.stamp = this->get_clock()->now();
        center_marker.ns = "tree_centers";
        center_marker.id = i * 2 + 1; // Odd IDs for centers
        center_marker.type = visualization_msgs::msg::Marker::SPHERE;
        center_marker.action = visualization_msgs::msg::Marker::ADD;
        
        center_marker.pose.position.x = cluster.center.x;
        center_marker.pose.position.y = cluster.center.y;
        center_marker.pose.position.z = cluster.center.z;
        center_marker.pose.orientation.w = 1.0;
        
        center_marker.scale.x = 0.2;
        center_marker.scale.y = 0.2;
        center_marker.scale.z = 0.2;
        
        center_marker.color.r = 1.0; // Red color
        center_marker.color.g = 0.0;
        center_marker.color.b = 0.0;
        center_marker.color.a = 1.0;
        
        marker_array.markers.push_back(center_marker);
    }
    
    trees_pub_->publish(marker_array);
}

void OrbitPlannerNode::publishTreeRowsVisualization(const std::vector<TreeRow>& tree_rows) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Color palette for different rows
    std::vector<std::vector<float>> row_colors = {
        {1.0, 0.0, 0.0}, // Red
        {0.0, 1.0, 0.0}, // Green
        {0.0, 0.0, 1.0}, // Blue
        {1.0, 1.0, 0.0}, // Yellow
        {1.0, 0.0, 1.0}, // Magenta
        {0.0, 1.0, 1.0}, // Cyan
        {1.0, 0.5, 0.0}, // Orange
        {0.5, 0.0, 1.0}, // Purple
        {0.0, 0.5, 0.0}, // Dark Green
        {0.5, 0.5, 0.5}  // Gray
    };
    
    // Create tree row markers
    for (size_t i = 0; i < tree_rows.size(); ++i) {
        const auto& row = tree_rows[i];
        
        if (row.trees.size() < 2) {
            continue; // Need at least 2 trees to form a line
        }
        
        // Get color for this row (cycle through palette)
        const auto& color = row_colors[i % row_colors.size()];
        
        // Create markers for each tree in this row
        for (size_t j = 0; j < row.trees.size(); ++j) {
            const auto& tree = row.trees[j];
            
            // Tree trunk marker (cylinder) - same color as row
            visualization_msgs::msg::Marker trunk_marker;
            trunk_marker.header.frame_id = "map";
            trunk_marker.header.stamp = this->get_clock()->now();
            trunk_marker.ns = "row_trees";
            trunk_marker.id = i * 1000 + j * 2; // Unique ID for each tree
            trunk_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            trunk_marker.action = visualization_msgs::msg::Marker::ADD;
            
            trunk_marker.pose.position.x = tree.center.x;
            trunk_marker.pose.position.y = tree.center.y;
            trunk_marker.pose.position.z = tree.center.z + tree.height / 2.0;
            trunk_marker.pose.orientation.w = 1.0;
            
            trunk_marker.scale.x = 0.3; // Fixed small diameter
            trunk_marker.scale.y = 0.3; // Fixed small diameter
            trunk_marker.scale.z = 0.5; // Fixed small height
            
            trunk_marker.color.r = color[0];
            trunk_marker.color.g = color[1];
            trunk_marker.color.b = color[2];
            trunk_marker.color.a = 0.8;
            
            marker_array.markers.push_back(trunk_marker);
            
            // Tree center marker (sphere) - brighter color
            visualization_msgs::msg::Marker center_marker;
            center_marker.header.frame_id = "map";
            center_marker.header.stamp = this->get_clock()->now();
            center_marker.ns = "row_tree_centers";
            center_marker.id = i * 1000 + j * 2 + 1; // Unique ID for each center
            center_marker.type = visualization_msgs::msg::Marker::SPHERE;
            center_marker.action = visualization_msgs::msg::Marker::ADD;
            
            center_marker.pose.position.x = tree.center.x;
            center_marker.pose.position.y = tree.center.y;
            center_marker.pose.position.z = tree.center.z;
            center_marker.pose.orientation.w = 1.0;
            
            center_marker.scale.x = 0.2;
            center_marker.scale.y = 0.2;
            center_marker.scale.z = 0.2;
            
            center_marker.color.r = color[0];
            center_marker.color.g = color[1];
            center_marker.color.b = color[2];
            center_marker.color.a = 1.0;
            
            marker_array.markers.push_back(center_marker);
        }
        
        // Create line marker for the tree row (using PCA-calculated endpoints)
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.ns = "tree_rows";
        line_marker.id = i * 2; // Even IDs for lines
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set line points using PCA-calculated endpoints
        line_marker.points.push_back(row.start_point);
        line_marker.points.push_back(row.end_point);
        
        // Set line properties
        line_marker.scale.x = 0.3; // Line width
        line_marker.color.r = color[0];
        line_marker.color.g = color[1];
        line_marker.color.b = color[2];
        line_marker.color.a = 0.8;
        
        marker_array.markers.push_back(line_marker);
        
        // Create arrow marker to show direction
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = this->get_clock()->now();
        arrow_marker.ns = "tree_row_directions";
        arrow_marker.id = i * 2 + 1; // Odd IDs for arrows
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set arrow position (middle of the line)
        arrow_marker.pose.position.x = (row.start_point.x + row.end_point.x) / 2.0;
        arrow_marker.pose.position.y = (row.start_point.y + row.end_point.y) / 2.0;
        arrow_marker.pose.position.z = (row.start_point.z + row.end_point.z) / 2.0;
        
        // Calculate orientation from PCA direction
        double dx = row.end_point.x - row.start_point.x;
        double dy = row.end_point.y - row.start_point.y;
        double yaw = std::atan2(dy, dx);
        
        // Convert yaw to quaternion
        arrow_marker.pose.orientation.x = 0.0;
        arrow_marker.pose.orientation.y = 0.0;
        arrow_marker.pose.orientation.z = std::sin(yaw / 2.0);
        arrow_marker.pose.orientation.w = std::cos(yaw / 2.0);
        
        // Set arrow properties
        arrow_marker.scale.x = std::sqrt(dx * dx + dy * dy); // Arrow length
        arrow_marker.scale.y = 0.1; // Arrow width
        arrow_marker.scale.z = 0.1; // Arrow height
        arrow_marker.color.r = color[0];
        arrow_marker.color.g = color[1];
        arrow_marker.color.b = color[2];
        arrow_marker.color.a = 0.8;
        
        marker_array.markers.push_back(arrow_marker);
        
        // Create text marker for row ID and tree count
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.ns = "tree_row_labels";
        text_marker.id = i;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        text_marker.pose.position.x = arrow_marker.pose.position.x;
        text_marker.pose.position.y = arrow_marker.pose.position.y;
        text_marker.pose.position.z = arrow_marker.pose.position.z + 0.5; // Above the arrow
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.3; // Text size
        text_marker.color.r = 1.0; // White color
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        text_marker.text = "Row " + std::to_string(i + 1) + " (" + std::to_string(row.tree_count) + " trees)";
        
        marker_array.markers.push_back(text_marker);
    }
    
    tree_rows_pub_->publish(marker_array);
}

void OrbitPlannerNode::publishFilteredHeightBandVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {
    // Convert PCL to ROS message
    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "map";
    filtered_cloud_msg.header.stamp = this->get_clock()->now();
    
    // Publish the filtered point cloud
    filtered_height_band_pub_->publish(filtered_cloud_msg);
}

void OrbitPlannerNode::filteredHeightBandCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Filtered height band callback triggered - received %u points", 
                msg->width * msg->height);
    
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud in filtered height band callback");
        return;
    }
    
    // Get current tree rows (already detected in pointCloudCallback)
    std::vector<TreeRow> tree_rows = current_tree_rows_;
    
    // Convert PCD to Occupancy Grid with row information
    auto occupancy_grid = convertPCDToOccupancyGrid(cloud, tree_rows);
    
    // Publish the occupancy grid
    pcd_occupancy_pub_->publish(occupancy_grid);
    
    // Publish sensor marker for visualization
    if (state_.robot_pose_defined) {
        publishSensorMarker(state_.current_robot_pose);
    }
    
    RCLCPP_INFO(this->get_logger(), "Published PCD occupancy grid with resolution %.3f", 
                occupancy_grid.info.resolution);
}

nav_msgs::msg::OccupancyGrid OrbitPlannerNode::convertPCDToOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<TreeRow>& tree_rows) {
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    
    if (cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot convert empty point cloud to occupancy grid");
        return occupancy_grid;
    }
    
    // Grid parameters
    double resolution = 0.1; // 10cm resolution
    
    // Calculate bounds based on exploration area if available, otherwise use point cloud bounds
    double min_x, max_x, min_y, max_y;
    
    if (state_.area_defined && !state_.exploration_area.polygon.points.empty()) {
        // Use exploration area bounds
        min_x = std::numeric_limits<double>::max();
        max_x = std::numeric_limits<double>::lowest();
        min_y = std::numeric_limits<double>::max();
        max_y = std::numeric_limits<double>::lowest();
        
        for (const auto& point : state_.exploration_area.polygon.points) {
            min_x = std::min(min_x, static_cast<double>(point.x));
            max_x = std::max(max_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            max_y = std::max(max_y, static_cast<double>(point.y));
        }
        
        RCLCPP_INFO(this->get_logger(), "Using exploration area bounds: (%.2f, %.2f) to (%.2f, %.2f)", 
                    min_x, min_y, max_x, max_y);
    } else {
        // Fallback to point cloud bounds
        min_x = std::numeric_limits<double>::max();
        max_x = std::numeric_limits<double>::lowest();
        min_y = std::numeric_limits<double>::max();
        max_y = std::numeric_limits<double>::lowest();
        
        for (const auto& point : cloud->points) {
            min_x = std::min(min_x, static_cast<double>(point.x));
            max_x = std::max(max_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            max_y = std::max(max_y, static_cast<double>(point.y));
        }
        
        RCLCPP_INFO(this->get_logger(), "Using point cloud bounds: (%.2f, %.2f) to (%.2f, %.2f)", 
                    min_x, min_y, max_x, max_y);
    }
    
    // Add some margin around the bounds
    double margin = 10.0; // 10m margin
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;
    
    // Calculate grid dimensions (use ceil to ensure boundary inclusion)
    double grid_width_m = max_x - min_x;
    double grid_height_m = max_y - min_y;
    int grid_width = std::max(1, static_cast<int>(std::ceil(grid_width_m / resolution)));
    int grid_height = std::max(1, static_cast<int>(std::ceil(grid_height_m / resolution)));
    
    // Validate grid size
    if (grid_width <= 0 || grid_height <= 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid grid size: %dx%d", grid_width, grid_height);
        return occupancy_grid;
    }
    
    RCLCPP_INFO(this->get_logger(), "Grid dimensions: %.2fm x %.2fm (%dx%d cells)", 
                grid_width_m, grid_height_m, grid_width, grid_height);
    
    // Set up occupancy grid
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.header.stamp = this->get_clock()->now();
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.width = grid_width;
    occupancy_grid.info.height = grid_height;
    occupancy_grid.info.origin.position.x = min_x;
    occupancy_grid.info.origin.position.y = min_y;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    
    // Initialize occupancy grid data - start with all free
    occupancy_grid.data.assign(grid_width * grid_height, 0);
    
    // Add individual trees as occupied areas with width
    for (const auto& row : tree_rows) {
        for (const auto& tree : row.trees) {
            addTreeToOccupancyGrid(occupancy_grid, tree, resolution);
        }
    }
    
    // Add tree rows as occupied areas with width
    for (const auto& row : tree_rows) {
        addRowToOccupancyGrid(occupancy_grid, row, resolution);
    }
    
    // Post-process: Set exploration area boundary as occupied and area outside as unknown
    if (state_.area_defined && !state_.exploration_area.polygon.points.empty()) {
        for (int y = 0; y < grid_height; ++y) {
            for (int x = 0; x < grid_width; ++x) {
                int cell_index = y * grid_width + x;
                
                // Convert grid coordinates back to world coordinates
                double world_x = occupancy_grid.info.origin.position.x + x * resolution;
                double world_y = occupancy_grid.info.origin.position.y + y * resolution;
                
                geometry_msgs::msg::Point cell_point;
                cell_point.x = world_x;
                cell_point.y = world_y;
                cell_point.z = 0.0;
                
                // Check if cell is inside exploration area
                bool is_inside = isPointInPolygon(cell_point, state_.exploration_area);
                
                if (!is_inside) {
                    // Outside exploration area: set as unknown
                    occupancy_grid.data[cell_index] = -1;
                } else {
                    // Inside exploration area: check if it's on the boundary
                    bool is_boundary = false;
                    
                    // Check if this cell is close to any polygon edge
                    const auto& points = state_.exploration_area.polygon.points;
                    for (size_t i = 0; i < points.size(); ++i) {
                        size_t j = (i + 1) % points.size();
                        
                        // Calculate distance from cell to polygon edge
                        double dist_to_edge = distancePointToLineSegment(
                            world_x, world_y,
                            points[i].x, points[i].y,
                            points[j].x, points[j].y
                        );
                        
                        // If within 2 grid cells of the boundary, mark as occupied
                        if (dist_to_edge <= 2.0 * resolution) {
                            is_boundary = true;
                            break;
                        }
                    }
                    
                    if (is_boundary) {
                        occupancy_grid.data[cell_index] = 100; // Occupied boundary
                    } else if (occupancy_grid.data[cell_index] != 100) {
                        // Inside area and not occupied by obstacle: set as free
                        occupancy_grid.data[cell_index] = 0; // Free
                    }
                    // Otherwise keep the original occupancy value (occupied by obstacle)
                }
            }
        }
    } else {
        // No exploration area defined: set all non-occupied cells as free
        for (int y = 0; y < grid_height; ++y) {
            for (int x = 0; x < grid_width; ++x) {
                int cell_index = y * grid_width + x;
                if (occupancy_grid.data[cell_index] != 100) {
                    occupancy_grid.data[cell_index] = 0; // Free
                }
            }
        }
    }
    
    // Count occupied cells for logging
    int occupied_count = 0;
    for (int val : occupancy_grid.data) {
        if (val == 100) occupied_count++;
    }
    
    RCLCPP_INFO(this->get_logger(), "Converted PCD to occupancy grid: %dx%d cells, %d occupied cells", 
                grid_width, grid_height, occupied_count);
    
    return occupancy_grid;
}

std::vector<std::pair<int, int>> OrbitPlannerNode::raycast2D(int x0, int y0, int x1, int y1) {
    std::vector<std::pair<int, int>> cells;
    
    // Bresenham's line algorithm for 2D ray casting
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int x_inc = (x0 < x1) ? 1 : -1;
    int y_inc = (y0 < y1) ? 1 : -1;
    int error = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
        cells.push_back({x, y});
        
        if (x == x1 && y == y1) {
            break;
        }
        
        int error2 = 2 * error;
        
        if (error2 > -dy) {
            error -= dy;
            x += x_inc;
        }
        
        if (error2 < dx) {
            error += dx;
            y += y_inc;
        }
    }
    
    return cells;
}

double OrbitPlannerNode::distancePointToLineSegment(double px, double py, double x1, double y1, double x2, double y2) {
    // Calculate the distance from point (px, py) to line segment (x1, y1) to (x2, y2)
    double A = px - x1;
    double B = py - y1;
    double C = x2 - x1;
    double D = y2 - y1;
    
    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    
    if (len_sq == 0) {
        // Line segment is actually a point
        return std::sqrt(A * A + B * B);
    }
    
    double param = dot / len_sq;
    
    double xx, yy;
    if (param < 0) {
        xx = x1;
        yy = y1;
    } else if (param > 1) {
        xx = x2;
        yy = y2;
    } else {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }
    
    double dx = px - xx;
    double dy = py - yy;
    return std::sqrt(dx * dx + dy * dy);
}

void OrbitPlannerNode::publishSensorMarker(const geometry_msgs::msg::PoseStamped& sensor_pose) {
    visualization_msgs::msg::Marker sensor_marker;
    
    sensor_marker.header.frame_id = "map";
    sensor_marker.header.stamp = this->get_clock()->now();
    sensor_marker.ns = "sensor_position";
    sensor_marker.id = 0;
    sensor_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sensor_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position
    sensor_marker.pose.position.x = sensor_pose.pose.position.x;
    sensor_marker.pose.position.y = sensor_pose.pose.position.y;
    sensor_marker.pose.position.z = sensor_pose.pose.position.z;
    sensor_marker.pose.orientation = sensor_pose.pose.orientation;
    
    // Scale (0.3m radius sphere)
    sensor_marker.scale.x = 0.6;
    sensor_marker.scale.y = 0.6;
    sensor_marker.scale.z = 0.6;
    
    // Color (bright blue)
    sensor_marker.color.r = 0.0;
    sensor_marker.color.g = 0.0;
    sensor_marker.color.b = 1.0;
    sensor_marker.color.a = 0.8;
    
    // Lifetime (auto-delete after 1 second if not updated)
    sensor_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    
    sensor_marker_pub_->publish(sensor_marker);
    
    // Also publish a text marker with grid coordinates
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = this->get_clock()->now();
    text_marker.ns = "sensor_text";
    text_marker.id = 1;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    text_marker.pose.position.x = sensor_pose.pose.position.x;
    text_marker.pose.position.y = sensor_pose.pose.position.y;
    text_marker.pose.position.z = sensor_pose.pose.position.z + 1.0; // Above the sphere
    text_marker.pose.orientation.w = 1.0;
    
    text_marker.scale.z = 0.5; // Text size
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    
    // Create text with world and grid coordinates
    std::ostringstream oss;
    oss << "Sensor\n";
    oss << "World: (" << std::fixed << std::setprecision(2) 
        << sensor_pose.pose.position.x << ", " 
        << sensor_pose.pose.position.y << ", " 
        << sensor_pose.pose.position.z << ")\n";
    
    // Calculate grid coordinates (assuming 0.1m resolution and origin at (0,0,0))
    int grid_x = static_cast<int>(sensor_pose.pose.position.x / 0.1);
    int grid_y = static_cast<int>(sensor_pose.pose.position.y / 0.1);
    oss << "Grid: (" << grid_x << ", " << grid_y << ")";
    
    text_marker.text = oss.str();
    
    sensor_marker_pub_->publish(text_marker);
}

void OrbitPlannerNode::addTreeToOccupancyGrid(nav_msgs::msg::OccupancyGrid& occupancy_grid, const TreeCluster& tree, double resolution) {
    // Get grid dimensions
    int grid_width = occupancy_grid.info.width;
    int grid_height = occupancy_grid.info.height;
    
    // Convert tree center to grid coordinates
    int center_x = static_cast<int>((tree.center.x - occupancy_grid.info.origin.position.x) / resolution);
    int center_y = static_cast<int>((tree.center.y - occupancy_grid.info.origin.position.y) / resolution);
    
    // Calculate tree radius in grid cells
    double tree_radius = tree.radius; // Use the tree's radius
    int radius_cells = static_cast<int>(std::ceil(tree_radius / resolution));
    
    // Mark cells within tree radius as occupied
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            int x = center_x + dx;
            int y = center_y + dy;
            
            // Check bounds
            if (x >= 0 && x < grid_width && y >= 0 && y < grid_height) {
                // Calculate distance from tree center
                double dist = std::sqrt(dx * dx + dy * dy) * resolution;
                
                // If within tree radius, mark as occupied
                if (dist <= tree_radius) {
                    int cell_index = y * grid_width + x;
                    occupancy_grid.data[cell_index] = 100; // Occupied
                }
            }
        }
    }
}

void OrbitPlannerNode::addRowToOccupancyGrid(nav_msgs::msg::OccupancyGrid& occupancy_grid, const TreeRow& row, double resolution) {
    // Convert row start and end points to grid coordinates
    int start_x = static_cast<int>((row.start_point.x - occupancy_grid.info.origin.position.x) / resolution);
    int start_y = static_cast<int>((row.start_point.y - occupancy_grid.info.origin.position.y) / resolution);
    int end_x = static_cast<int>((row.end_point.x - occupancy_grid.info.origin.position.x) / resolution);
    int end_y = static_cast<int>((row.end_point.y - occupancy_grid.info.origin.position.y) / resolution);
    
    // Get grid dimensions
    int grid_width = occupancy_grid.info.width;
    int grid_height = occupancy_grid.info.height;
    
    // Use raycast2D to get all cells along the line segment
    auto line_cells = raycast2D(start_x, start_y, end_x, end_y);
    
    // Mark all cells along the line as occupied
    for (const auto& cell : line_cells) {
        int x = cell.first;
        int y = cell.second;
        
        // Check bounds
        if (x >= 0 && x < grid_width && y >= 0 && y < grid_height) {
            int cell_index = y * grid_width + x;
            occupancy_grid.data[cell_index] = 100; // Occupied
        }
    }
    
    // Add some width to the row by marking nearby cells as occupied
    // This creates a thicker line to represent the tree row
    double row_width = 0.5; // 50cm width for the row
    int width_cells = static_cast<int>(std::ceil(row_width / resolution));
    
    for (const auto& cell : line_cells) {
        int x = cell.first;
        int y = cell.second;
        
        // Mark cells within row_width distance as occupied
        for (int dx = -width_cells; dx <= width_cells; ++dx) {
            for (int dy = -width_cells; dy <= width_cells; ++dy) {
                int nx = x + dx;
                int ny = y + dy;
                
                // Check bounds
                if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height) {
                    // Calculate distance from the line
                    double world_x = occupancy_grid.info.origin.position.x + nx * resolution;
                    double world_y = occupancy_grid.info.origin.position.y + ny * resolution;
                    
                    double dist_to_line = distancePointToLineSegment(
                        world_x, world_y,
                        row.start_point.x, row.start_point.y,
                        row.end_point.x, row.end_point.y
                    );
                    
                    // If within row width, mark as occupied
                    if (dist_to_line <= row_width) {
                        int cell_index = ny * grid_width + nx;
                        occupancy_grid.data[cell_index] = 100; // Occupied
                    }
                }
            }
        }
    }
}

void OrbitPlannerNode::parameterUpdateCallback(const std_msgs::msg::String::SharedPtr msg) {
    try {
        // Parse parameter string (format: "key1=value1;key2=value2;...")
        std::string param_str = msg->data;
        std::istringstream iss(param_str);
        std::string param_pair;
        
        bool parameters_updated = false;
        
        while (std::getline(iss, param_pair, ';')) {
            size_t equal_pos = param_pair.find('=');
            if (equal_pos != std::string::npos) {
                std::string key = param_pair.substr(0, equal_pos);
                std::string value_str = param_pair.substr(equal_pos + 1);
                
                // Update parameters based on key
                if (key == "map_update_rate") {
                    params_.map_update_rate = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("map_update_rate", params_.map_update_rate));
                    parameters_updated = true;
                } else if (key == "planning_rate") {
                    params_.planning_rate = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("planning_rate", params_.planning_rate));
                    parameters_updated = true;
                } else if (key == "robot_radius") {
                    params_.robot_radius = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("robot_radius", params_.robot_radius));
                    parameters_updated = true;
                } else if (key == "safety_margin") {
                    params_.safety_margin = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("safety_margin", params_.safety_margin));
                    parameters_updated = true;
                } else if (key == "max_planning_distance") {
                    params_.max_planning_distance = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("max_planning_distance", params_.max_planning_distance));
                    parameters_updated = true;
                } else if (key == "frontier_cluster_min_size") {
                    params_.frontier_cluster_min_size = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("frontier_cluster_min_size", params_.frontier_cluster_min_size));
                    parameters_updated = true;
                } else if (key == "frontier_cluster_max_distance") {
                    params_.frontier_cluster_max_distance = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("frontier_cluster_max_distance", params_.frontier_cluster_max_distance));
                    parameters_updated = true;
                } else if (key == "goal_tolerance") {
                    params_.goal_tolerance = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("goal_tolerance", params_.goal_tolerance));
                    parameters_updated = true;
                } else if (key == "yaw_change_weight") {
                    params_.yaw_change_weight = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("yaw_change_weight", params_.yaw_change_weight));
                    parameters_updated = true;
                } else if (key == "frontier_gain_weight") {
                    params_.frontier_gain_weight = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("frontier_gain_weight", params_.frontier_gain_weight));
                    parameters_updated = true;
                } else if (key == "distance_weight") {
                    params_.distance_weight = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("distance_weight", params_.distance_weight));
                    parameters_updated = true;
                } else if (key == "tree_height_min") {
                    params_.tree_height_min = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("tree_height_min", params_.tree_height_min));
                    parameters_updated = true;
                } else if (key == "tree_height_max") {
                    params_.tree_height_max = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("tree_height_max", params_.tree_height_max));
                    parameters_updated = true;
                } else if (key == "tree_cluster_tolerance") {
                    params_.tree_cluster_tolerance = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("tree_cluster_tolerance", params_.tree_cluster_tolerance));
                    parameters_updated = true;
                } else if (key == "tree_min_cluster_size") {
                    params_.tree_min_cluster_size = std::stoi(value_str);
                    this->set_parameter(rclcpp::Parameter("tree_min_cluster_size", params_.tree_min_cluster_size));
                    parameters_updated = true;
                } else if (key == "row_tolerance") {
                    params_.row_tolerance = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("row_tolerance", params_.row_tolerance));
                    parameters_updated = true;
                } else if (key == "row_min_length") {
                    params_.row_min_length = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("row_min_length", params_.row_min_length));
                    parameters_updated = true;
                } else if (key == "row_min_trees") {
                    params_.row_min_trees = std::stoi(value_str);
                    this->set_parameter(rclcpp::Parameter("row_min_trees", params_.row_min_trees));
                    parameters_updated = true;
                } else if (key == "center_search_radius") {
                    params_.center_search_radius = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("center_search_radius", params_.center_search_radius));
                    parameters_updated = true;
                } else if (key == "clustering_radius") {
                    params_.clustering_radius = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("clustering_radius", params_.clustering_radius));
                    parameters_updated = true;
                } else if (key == "min_neighbors_in_radius") {
                    params_.min_neighbors_in_radius = std::stoi(value_str);
                    this->set_parameter(rclcpp::Parameter("min_neighbors_in_radius", params_.min_neighbors_in_radius));
                    parameters_updated = true;
                } else if (key == "orbit_radius") {
                    params_.orbit_radius = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("orbit_radius", params_.orbit_radius));
                    parameters_updated = true;
                } else if (key == "orbit_spacing") {
                    params_.orbit_spacing = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("orbit_spacing", params_.orbit_spacing));
                    parameters_updated = true;
                } else if (key == "path_resolution") {
                    params_.path_resolution = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("path_resolution", params_.path_resolution));
                    parameters_updated = true;
                } else if (key == "path_smoothing_factor") {
                    params_.path_smoothing_factor = std::stod(value_str);
                    this->set_parameter(rclcpp::Parameter("path_smoothing_factor", params_.path_smoothing_factor));
                    parameters_updated = true;
                }
            }
        }
        
        if (parameters_updated) {
            // Update component parameters
            tree_clusterer_->setParameters(
                params_.tree_height_min, params_.tree_height_max,
                params_.tree_cluster_tolerance, params_.tree_min_cluster_size,
                0.1, params_.row_tolerance, params_.row_min_length, params_.row_min_trees,
                params_.center_search_radius, params_.clustering_radius, params_.min_neighbors_in_radius);
            
            frontier_detector_->setParameters(
                params_.robot_radius, static_cast<int>(params_.frontier_cluster_min_size),
                params_.max_planning_distance, params_.frontier_gain_weight);
            
            path_planner_->setParameters(
                params_.robot_radius, params_.safety_margin,
                params_.path_resolution, params_.max_planning_distance);
            
            orbit_anchor_generator_->setParameters(
                params_.orbit_radius, params_.orbit_spacing,
                5.0, params_.max_planning_distance);
            
            RCLCPP_INFO(this->get_logger(), "Parameters updated successfully from panel");
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse parameter update: %s", e.what());
    }
}

} // namespace orbit_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<orbit_planner::OrbitPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
