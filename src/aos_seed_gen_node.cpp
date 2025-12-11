#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <queue>
#include <unordered_set>
#include <numeric>
#include <limits>
#include <Eigen/Dense>

// Structure to represent a cluster of occupied cells
struct Cluster {
  std::vector<std::pair<int, int>> cells;  // Grid coordinates
  float center_x = 0.0f;
  float center_y = 0.0f;
  int size = 0;
  float length = 0.0f;  // Maximum length of the cluster in meters
};

// Structure to represent a tree row from skeleton cluster
struct TreeRowFromCluster {
  Eigen::Vector2d center;
  Eigen::Vector2d start_point;
  Eigen::Vector2d end_point;
  double length;
};

// Structure to represent a virtual seed
struct VirtualSeed {
  geometry_msgs::msg::Point position;
  int tree_row_index;  // Which tree row this seed belongs to
  bool is_converted;    // Whether this seed has been converted to real seed
};

// Structure to represent a ray cast from a seed
struct RayCast {
  geometry_msgs::msg::Point start_point;  // Start point (base seed position)
  geometry_msgs::msg::Point end_point;   // Collision point or maximum distance point
  bool hit;                              // Whether collision occurred
};

class AosSeedGenNode : public rclcpp::Node {
public:
  AosSeedGenNode() : Node("aos_seed_gen_node") {
    // Parameters
    this->declare_parameter<float>("clipping_minz", -0.4); 
    this->declare_parameter<float>("clipping_maxz", 0.5);
    this->declare_parameter<float>("clipping_minx", -5.0);
    this->declare_parameter<float>("clipping_maxx", 72.0);
    this->declare_parameter<float>("clipping_miny", -10.0);
    this->declare_parameter<float>("clipping_maxy", 20.0);
    this->declare_parameter<float>("grid_resolution", 0.05);
    this->declare_parameter<float>("inflation_radius", 0.8);
    this->declare_parameter<float>("waypoint_offset_distance", 2.4);
    this->declare_parameter<float>("waypoint_offset_distance_x", 2.0);
    this->declare_parameter<float>("waypoint_offset_distance_y", 2.4);
    this->declare_parameter<bool>("publish_cluster_axes", true);
    this->declare_parameter<double>("cluster_axis_scale", 1.0);
    this->declare_parameter<double>("cluster_axis_width", 0.08);
    this->declare_parameter<double>("cluster_min_length", 2.0);

    // Get parameters
    clipping_minz = this->get_parameter("clipping_minz").as_double();
    clipping_maxz = this->get_parameter("clipping_maxz").as_double();
    clipping_minx = this->get_parameter("clipping_minx").as_double();
    clipping_maxx = this->get_parameter("clipping_maxx").as_double();
    clipping_miny = this->get_parameter("clipping_miny").as_double();
    clipping_maxy = this->get_parameter("clipping_maxy").as_double();
    grid_resolution = this->get_parameter("grid_resolution").as_double();
    inflation_radius = this->get_parameter("inflation_radius").as_double();
    waypoint_offset_distance = this->get_parameter("waypoint_offset_distance").as_double();
    waypoint_offset_distance_x = this->get_parameter("waypoint_offset_distance_x").as_double();
    waypoint_offset_distance_y = this->get_parameter("waypoint_offset_distance_y").as_double();
    publish_cluster_axes_ = this->get_parameter("publish_cluster_axes").as_bool();
    cluster_axis_scale_ = this->get_parameter("cluster_axis_scale").as_double();
    cluster_axis_width_ = this->get_parameter("cluster_axis_width").as_double();
    cluster_min_length_m_ = this->get_parameter("cluster_min_length").as_double();

    if (cluster_axis_scale_ < 0.0) {
      cluster_axis_scale_ = 0.0;
    }
    if (cluster_axis_width_ <= 0.0) {
      cluster_axis_width_ = 0.05;
    }
    if (cluster_min_length_m_ < 0.0) {
      cluster_min_length_m_ = 0.0;
    }
    if (waypoint_offset_distance_x < 0.0) {
      waypoint_offset_distance_x = 0.0;
    }
    if (waypoint_offset_distance_y < 0.0) {
      waypoint_offset_distance_y = 0.0;
    }


    // QoS settings: Real-time essential data uses Reliable, visualization uses BestEffort
    rclcpp::QoS reliable_qos(10);
    reliable_qos.reliable();
    reliable_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Keep latest data
    reliable_qos.history(rclcpp::HistoryPolicy::KeepLast);
    
    rclcpp::QoS best_effort_qos(10);
    best_effort_qos.best_effort();  // For visualization
    best_effort_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    best_effort_qos.keep_last(1);  // Memory saving: keep only last 1

    // Publisher for occupancy grid - important data, reliable
    publisher_occupancy_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", reliable_qos);
    
    // Publisher for skeletonized occupancy grid - important data, reliable  
    publisher_skeletonized_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("skeletonized_occupancy_grid", reliable_qos);
    
    // Publisher for cluster visualization - for visualization, changed to best_effort
    publisher_cluster_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("skeleton_clusters", reliable_qos);
    
    // Publisher for cluster information - important data but frequent updates, best_effort
    publisher_cluster_info = this->create_publisher<geometry_msgs::msg::PoseArray>("cluster_info", reliable_qos);
    
    // Publisher for exploration area visualization - for visualization, best_effort
    publisher_exploration_area_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("gvd/exploration_area", reliable_qos);
    
    // Publisher for filtered confirmed trees
    publisher_filtered_trees = this->create_publisher<geometry_msgs::msg::PoseArray>("filtered_confirmed_trees", reliable_qos);
    
    // Publisher for filtered trees visualization
    publisher_filtered_trees_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("filtered_confirmed_trees_markers", reliable_qos);
    
    // Publisher for tree rows (all and exploration area)
    publisher_tree_rows_all = this->create_publisher<visualization_msgs::msg::MarkerArray>("tree_rows_all", reliable_qos);
    publisher_tree_rows_exploration = this->create_publisher<visualization_msgs::msg::MarkerArray>("tree_rows_exploration", reliable_qos);
    
    // Publisher for voronoi seeds (start and end points of all tree rows, positions of filtered trees)
    publisher_voronoi_seeds = this->create_publisher<geometry_msgs::msg::PoseArray>("voronoi_seeds", reliable_qos);
    
    // Publisher for voronoi seeds visualization
    publisher_voronoi_seeds_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("voronoi_seeds_markers", reliable_qos);
    
    // Publisher for exploration tree rows info (tree row information in area: start and end points)
    publisher_exploration_tree_rows_info = this->create_publisher<geometry_msgs::msg::PoseArray>("exploration_tree_rows_info", reliable_qos);
    
    // Publisher for virtual and real seeds visualization
    publisher_virtual_real_seeds_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("virtual_real_seeds_markers", reliable_qos);
    
    // Input topic parameters
    this->declare_parameter<std::string>("global_map_topic", "/lio_sam/mapping/global_map");
    this->declare_parameter<std::string>("exploration_area_topic", "/aos_planner/exploration_area");
    this->declare_parameter<std::string>("robot_position_topic", "/Local/utm");
    
    std::string global_map_topic, exploration_area_topic, robot_position_topic;
    this->get_parameter("global_map_topic", global_map_topic);
    this->get_parameter("exploration_area_topic", exploration_area_topic);
    this->get_parameter("robot_position_topic", robot_position_topic);
    
    // Subscriber for global map
    subscriber_global_map = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        global_map_topic, 10,
        std::bind(&AosSeedGenNode::globalMapCallback, this, std::placeholders::_1));
    
    // Subscriber for exploration area polygon
    subscriber_exploration_area = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        exploration_area_topic, 10,
        std::bind(&AosSeedGenNode::explorationAreaCallback, this, std::placeholders::_1));
    
    // Subscriber for robot position
    subscriber_robot_position_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        robot_position_topic, 10,
        std::bind(&AosSeedGenNode::robotPositionCallback, this, std::placeholders::_1));
    
    // Set hardcoded exploration area polygon (taken from aos_gvd_node.cpp)
    {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      hardcoded_polygon_points_.clear();
      hardcoded_polygon_points_.push_back({-1.972916603088379, 7.9420671463012695});
      hardcoded_polygon_points_.push_back({-2.0726776123046875, 0.022441387176513672});
      hardcoded_polygon_points_.push_back({70.22465515136719, 2.102720260620117});
      hardcoded_polygon_points_.push_back({69.48777770996094, 9.786612510681152});
      
      // Calculate bounding box
      hardcoded_minx_ = hardcoded_polygon_points_[0].first;
      hardcoded_maxx_ = hardcoded_polygon_points_[0].first;
      hardcoded_miny_ = hardcoded_polygon_points_[0].second;
      hardcoded_maxy_ = hardcoded_polygon_points_[0].second;
      
      for (const auto& pt : hardcoded_polygon_points_) {
        hardcoded_minx_ = std::min(hardcoded_minx_, pt.first);
        hardcoded_maxx_ = std::max(hardcoded_maxx_, pt.first);
        hardcoded_miny_ = std::min(hardcoded_miny_, pt.second);
        hardcoded_maxy_ = std::max(hardcoded_maxy_, pt.second);
      }
      
      use_hardcoded_polygon_ = true;
    }
    
    // Exploration area visualization timer (1Hz)
    exploration_area_viz_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&AosSeedGenNode::publishExplorationAreaViz, this)
    );
    
  }
  
  ~AosSeedGenNode() {
    // Cleanup if needed
  }

private:
  void globalMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert ROS message to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Radius Outlier Removal (no explicit search method)
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.2);          // tune for your point spacing
    ror.setMinNeighborsInRadius(2);    // tune for desired strictness

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    ror.filter(*cloud_out);

    last_cloud = cloud_out;

    // Continue your pipeline
    processPointCloud(cloud_out);
  }

  void explorationAreaCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    
    // Use polygon received from topic (instead of hardcoded values)
    if (!msg->polygon.points.empty() && msg->polygon.points.size() >= 3) {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      
      // Update with polygon received from topic
      hardcoded_polygon_points_.clear();
      for (const auto& pt : msg->polygon.points) {
        hardcoded_polygon_points_.push_back({pt.x, pt.y});
      }
      
      // Recalculate bounding box
      hardcoded_minx_ = hardcoded_polygon_points_[0].first;
      hardcoded_maxx_ = hardcoded_polygon_points_[0].first;
      hardcoded_miny_ = hardcoded_polygon_points_[0].second;
      hardcoded_maxy_ = hardcoded_polygon_points_[0].second;
      
      for (const auto& pt : hardcoded_polygon_points_) {
        hardcoded_minx_ = std::min(hardcoded_minx_, pt.first);
        hardcoded_maxx_ = std::max(hardcoded_maxx_, pt.first);
        hardcoded_miny_ = std::min(hardcoded_miny_, pt.second);
        hardcoded_maxy_ = std::max(hardcoded_maxy_, pt.second);
      }
      
      // Don't use hardcoded area if topic is received
      use_hardcoded_polygon_ = false;
    }
    
    // Republish /gvd/exploration_area immediately since polygon changed
    publishExplorationAreaViz();
    
    // Regenerate 2D OccupancyGrid since polygon changed (reprocess last point cloud)
    if (last_cloud && !last_cloud->points.empty()) {
      processPointCloud(last_cloud);
    }
  }
  
  void robotPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
      std::lock_guard<std::mutex> lock(seeds_mutex_);
      current_robot_position_.x = msg->data[0];
      current_robot_position_.y = msg->data[1];
      current_robot_position_.z = 0.0;
      robot_position_received_ = true;
      
      // Convert confirmed tree positions near virtual seeds to real seeds when robot passes near virtual seeds (4m)
      convertVirtualSeedsToReal();
    }
  }

  void publishFilteredTreesVisualization(const geometry_msgs::msg::PoseArray& trees) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete existing markers
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = trees.header.frame_id.empty() ? "map" : trees.header.frame_id;
    delete_all.header.stamp = this->get_clock()->now();
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);
    
    if (trees.poses.empty()) {
      publisher_filtered_trees_viz->publish(marker_array);
      return;
    }
    
    // Display trees as spheres
    visualization_msgs::msg::Marker trees_marker;
    trees_marker.header = trees.header;
    trees_marker.ns = "filtered_confirmed_trees";
    trees_marker.id = 0;
    trees_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    trees_marker.action = visualization_msgs::msg::Marker::ADD;
    trees_marker.scale.x = 0.4;  // Tree size
    trees_marker.scale.y = 0.4;
    trees_marker.scale.z = 0.4;
    trees_marker.color.r = 0.0;  // Green
    trees_marker.color.g = 1.0;
    trees_marker.color.b = 0.0;
    trees_marker.color.a = 1.0;
    
    // Add each tree position as point
    for (const auto& pose : trees.poses) {
      geometry_msgs::msg::Point p;
      p.x = pose.position.x;
      p.y = pose.position.y;
      p.z = 0.0;
      trees_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(trees_marker);
    
    // Publish
    publisher_filtered_trees_viz->publish(marker_array);
  }
  
  void publishExplorationAreaViz() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete existing markers
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = "map";
    delete_all.header.stamp = this->get_clock()->now();
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);
    
    // Safely read polygon data
    std::vector<std::pair<double, double>> polygon_copy;
    {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      polygon_copy = hardcoded_polygon_points_;
    }
    
    if (polygon_copy.size() < 3) {
      publisher_exploration_area_viz->publish(marker_array);
      return;
    }
    
    rclcpp::Time now = this->get_clock()->now();
    
    // Display polygon border (LINE_STRIP)
    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.stamp = now;
    polygon_marker.header.frame_id = "map";
    polygon_marker.ns = "exploration_area";
    polygon_marker.id = 0;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.scale.x = 0.15;  // Line thickness
    polygon_marker.color.r = 0.0;
    polygon_marker.color.g = 0.0;
    polygon_marker.color.b = 1.0;  // Blue
    polygon_marker.color.a = 0.8;
    
    // Add polygon points in order
    for (const auto& pt : polygon_copy) {
      geometry_msgs::msg::Point p;
      p.x = pt.first;
      p.y = pt.second;
      p.z = 0.0;
      polygon_marker.points.push_back(p);
    }
    
    // Add first point at the end to close polygon
    if (!polygon_copy.empty()) {
      geometry_msgs::msg::Point p;
      p.x = polygon_copy[0].first;
      p.y = polygon_copy[0].second;
      p.z = 0.0;
      polygon_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(polygon_marker);
    
    // Display polygon points as spheres
    for (size_t i = 0; i < polygon_copy.size(); ++i) {
      visualization_msgs::msg::Marker point_marker;
      point_marker.header.stamp = now;
      point_marker.header.frame_id = "map";
      point_marker.ns = "exploration_area_points";
      point_marker.id = static_cast<int>(i);
      point_marker.type = visualization_msgs::msg::Marker::SPHERE;
      point_marker.action = visualization_msgs::msg::Marker::ADD;
      point_marker.pose.position.x = polygon_copy[i].first;
      point_marker.pose.position.y = polygon_copy[i].second;
      point_marker.pose.position.z = 0.0;
      point_marker.pose.orientation.w = 1.0;
      point_marker.scale.x = 0.3;
      point_marker.scale.y = 0.3;
      point_marker.scale.z = 0.3;
      point_marker.color.r = 1.0;
      point_marker.color.g = 0.0;
      point_marker.color.b = 0.0;  // Red
      point_marker.color.a = 1.0;
      marker_array.markers.push_back(point_marker);
      
      // Display point number text
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.stamp = now;
      text_marker.header.frame_id = "map";
      text_marker.ns = "exploration_area_labels";
      text_marker.id = static_cast<int>(i);
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position.x = polygon_copy[i].first;
      text_marker.pose.position.y = polygon_copy[i].second;
      text_marker.pose.position.z = 0.5;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.4;
      text_marker.color.r = 1.0;
      text_marker.color.g = 0.0;
      text_marker.color.b = 0.0;
      text_marker.color.a = 1.0;
      text_marker.text = "P" + std::to_string(i);
      text_marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker_array.markers.push_back(text_marker);
    }
    
    // Publish
    publisher_exploration_area_viz->publish(marker_array);
  }

  void processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float use_minx, use_maxx, use_miny, use_maxy;
    getActiveBounds(use_minx, use_maxx, use_miny, use_maxy);

    // Step 1: Apply clipping filters
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    
    // Z-axis filtering (keep only points within specified height range)
    pass.setFilterFieldName("z");
    pass.setFilterLimits(clipping_minz, clipping_maxz);
    pass.filter(*cloud_filtered);

    // X-axis filtering according to active area bounds
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(use_minx, use_maxx);
    pass.filter(*cloud_filtered);

    // Y-axis filtering according to active area bounds
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(use_miny, use_maxy);
    pass.filter(*cloud_filtered);


    // Step 1.5: Remove points within 1m radius of specified positions (temporary)
    {
      // Specified positions (set in rviz2)
      struct ExclusionPoint {
        float x, y;
        float radius;
      };
      std::vector<ExclusionPoint> exclusion_points = {
        {0.646417f, 3.83918f, 1.0f},
        {2.0405f, 3.62485f, 1.0f},
        {65.3711f, 2.09755f, 1.0f},
        {66.9094f, 2.07515f, 1.0f},
        {-1.61309f, 5.69933f, 1.0f},
        {-1.97349f, 4.77329f, 1.0f},
        {-2.11365f, 3.74464f, 1.0f},
        {-2.26381f, 2.70848f, 1.0f},
        {-2.66426f, 1.72738f, 1.0f},
        {68.0229f, 2.31687f, 1.0f},
        {65.4647f, 2.18653f, 1.0f}
      };
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_exclusions(new pcl::PointCloud<pcl::PointXYZ>);
      cloud_without_exclusions->points.reserve(cloud_filtered->points.size());
      
      for (const auto& point : cloud_filtered->points) {
        bool exclude = false;
        for (const auto& excl : exclusion_points) {
          float dx = point.x - excl.x;
          float dy = point.y - excl.y;
          float dist_sq = dx * dx + dy * dy;
          if (dist_sq <= excl.radius * excl.radius) {
            exclude = true;
            break;
          }
        }
        if (!exclude) {
          cloud_without_exclusions->points.push_back(point);
        }
      }
      
      cloud_without_exclusions->width = cloud_without_exclusions->points.size();
      cloud_without_exclusions->height = 1;
      cloud_without_exclusions->is_dense = true;
      
      cloud_filtered = cloud_without_exclusions;
    }

    // Step 2: Compress to XY plane (set all Z to 0)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy_compressed(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud_filtered->points) {
      pcl::PointXYZ compressed_point;
      compressed_point.x = point.x;
      compressed_point.y = point.y;
      compressed_point.z = 0.0;  // Compress to XY plane
      cloud_xy_compressed->points.push_back(compressed_point);
    }
    cloud_xy_compressed->width = cloud_xy_compressed->points.size();
    cloud_xy_compressed->height = 1;
    cloud_xy_compressed->is_dense = true;

    // Step 3: Generate occupancy grid
    nav_msgs::msg::OccupancyGrid occupancy_grid = generateOccupancyGrid(cloud_xy_compressed);
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.header.stamp = this->get_clock()->now();

    // Step 4: (Disabled) Previously removed occupied cells near waypoints. Now keep original grid.
    nav_msgs::msg::OccupancyGrid grid_without_waypoints = occupancy_grid;

    // Step 5: Apply inflation to remaining occupied cells
    nav_msgs::msg::OccupancyGrid inflated_grid = applyInflation(grid_without_waypoints);
    
    // Step 5.5: Mark grid boundaries as occupied
    nav_msgs::msg::OccupancyGrid inflated_grid_with_boundaries = markBoundariesAsOccupied(inflated_grid);

    // Step 6: Publish inflated grid with boundaries to /occupancy_grid
    inflated_grid_with_boundaries.header.frame_id = "map";
    inflated_grid_with_boundaries.header.stamp = this->get_clock()->now();
    publisher_occupancy_grid->publish(inflated_grid_with_boundaries);

    // Step 7: Apply skeletonization to inflated grid (without boundaries)
    nav_msgs::msg::OccupancyGrid skeletonized_grid = skeletonizeOccupancyGrid(inflated_grid);
    
    // Store skeletonized grid (for ray casting)
    {
      std::lock_guard<std::mutex> lock(skeletonized_grid_mutex_);
      last_skeletonized_grid_ = skeletonized_grid;
    }

    // Step 8: Cluster and visualize occupied cells in skeletonized grid (without boundaries)
    clusterAndVisualizeSkeletonizedGrid(skeletonized_grid);

    // Step 9: Mark polygon boundary after clustering (fallback to grid borders if no polygon)
    nav_msgs::msg::OccupancyGrid skeletonized_grid_with_boundaries = markPolygonBoundaryAsOccupied(skeletonized_grid);

    // Step 10: Publish final skeletonized occupancy grid with boundaries
    skeletonized_grid_with_boundaries.header.frame_id = "map";
    skeletonized_grid_with_boundaries.header.stamp = this->get_clock()->now();
    publisher_skeletonized_grid->publish(skeletonized_grid_with_boundaries);

  }

  nav_msgs::msg::OccupancyGrid generateOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    nav_msgs::msg::OccupancyGrid grid;
    
    // Calculate grid dimensions using active bounds (polygon bbox if available)
    float use_minx, use_maxx, use_miny, use_maxy;
    getActiveBounds(use_minx, use_maxx, use_miny, use_maxy);
    float width = std::max(0.0f, use_maxx - use_minx);
    float height = std::max(0.0f, use_maxy - use_miny);
    
    grid.info.resolution = grid_resolution;
    unsigned int w_cells = static_cast<unsigned int>(std::ceil(width / grid_resolution));
    unsigned int h_cells = static_cast<unsigned int>(std::ceil(height / grid_resolution));
    if (w_cells == 0) w_cells = 1;
    if (h_cells == 0) h_cells = 1;
    grid.info.width = w_cells;
    grid.info.height = h_cells;
    grid.info.origin.position.x = use_minx;
    grid.info.origin.position.y = use_miny;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    // Initialize grid data
    grid.data.resize(grid.info.width * grid.info.height, 0);  // 0 = free space


    // Mark occupied cells
    for (const auto& point : cloud->points) {
      // Convert world coordinates to grid coordinates
      int grid_x = static_cast<int>((point.x - grid.info.origin.position.x) / grid_resolution);
      int grid_y = static_cast<int>((point.y - grid.info.origin.position.y) / grid_resolution);
      
      // Check bounds
      if (grid_x >= 0 && grid_x < static_cast<int>(grid.info.width) && 
          grid_y >= 0 && grid_y < static_cast<int>(grid.info.height)) {
        
        int index = grid_x + grid_y * grid.info.width;
        grid.data[index] = 100;  // Mark as occupied
      }
    }

    return grid;
  }


  // Convert occupancy grid to OpenCV Mat
  cv::Mat occupancyGridToMat(const nav_msgs::msg::OccupancyGrid& grid) {
    cv::Mat mat(grid.info.height, grid.info.width, CV_8UC1);
    
    for (unsigned int y = 0; y < grid.info.height; y++) {
      for (unsigned int x = 0; x < grid.info.width; x++) {
        int index = x + y * grid.info.width;
        int8_t value = grid.data[index];
        
        // Convert occupancy values: -1(unknown) -> 0, 0(free) -> 0, 100(occupied) -> 255
        if (value == 100) {
          mat.at<uchar>(y, x) = 255;  // White for occupied
        } else {
          mat.at<uchar>(y, x) = 0;    // Black for free/unknown
        }
      }
    }
    
    return mat;
  }

  // Convert OpenCV Mat back to occupancy grid
  nav_msgs::msg::OccupancyGrid matToOccupancyGrid(const cv::Mat& mat, const nav_msgs::msg::OccupancyGrid& original_grid) {
    nav_msgs::msg::OccupancyGrid grid = original_grid;
    
    // Clear the data
    grid.data.clear();
    grid.data.resize(grid.info.width * grid.info.height, 0);
    
    for (int y = 0; y < mat.rows; y++) {
      for (int x = 0; x < mat.cols; x++) {
        int index = x + y * grid.info.width;
        uchar value = mat.at<uchar>(y, x);
        
        // Convert back: 255(white) -> 100(occupied), 0(black) -> 0(free)
        if (value == 255) {
          grid.data[index] = 100;
        } else {
          grid.data[index] = 0;
        }
      }
    }
    
    return grid;
  }

  // Apply skeletonization to occupancy grid
  nav_msgs::msg::OccupancyGrid skeletonizeOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid) {
    
    // Convert to OpenCV Mat
    cv::Mat mat = occupancyGridToMat(grid);
    
    // Apply morphological operations for better skeletonization
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat opened;
    cv::morphologyEx(mat, opened, cv::MORPH_OPEN, kernel);
    
    // Apply skeletonization using Zhang-Suen algorithm
    cv::Mat skeleton;
    cv::ximgproc::thinning(opened, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);
    
    // If ximgproc is not available, use alternative method
    if (skeleton.empty()) {
      
      // Alternative: Use distance transform and watershed
      cv::Mat dist;
      cv::distanceTransform(opened, dist, cv::DIST_L2, 5);
      
      // Normalize distance transform
      cv::Mat dist_norm;
      cv::normalize(dist, dist_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
      
      // Threshold to get skeleton-like structure
      cv::threshold(dist_norm, skeleton, 200, 255, cv::THRESH_BINARY);
    }
    
    // Convert back to occupancy grid
    nav_msgs::msg::OccupancyGrid skeletonized_grid = matToOccupancyGrid(skeleton, grid);
    
    return skeletonized_grid;
  }

  // Mark occupancy grid borders as occupied with thick borders
  nav_msgs::msg::OccupancyGrid markBoundariesAsOccupied(const nav_msgs::msg::OccupancyGrid& grid) {
    nav_msgs::msg::OccupancyGrid result_grid = grid;
    
    // Use thicker borders to survive skeletonization (at least 5 cells)
    int border_thickness = 5;
    
    // Mark top border (thick)
    for (int y = 0; y < border_thickness && y < static_cast<int>(grid.info.height); y++) {
      for (int x = 0; x < static_cast<int>(grid.info.width); x++) {
        int index = x + y * grid.info.width;
        result_grid.data[index] = 100;
      }
    }
    
    // Mark bottom border (thick)
    for (int y = std::max(0, static_cast<int>(grid.info.height) - border_thickness); 
         y < static_cast<int>(grid.info.height); y++) {
      for (int x = 0; x < static_cast<int>(grid.info.width); x++) {
        int index = x + y * grid.info.width;
        result_grid.data[index] = 100;
      }
    }
    
    // Mark left border (thick)
    for (int x = 0; x < border_thickness && x < static_cast<int>(grid.info.width); x++) {
      for (int y = 0; y < static_cast<int>(grid.info.height); y++) {
        int index = x + y * grid.info.width;
        result_grid.data[index] = 100;
      }
    }
    
    // Mark right border (thick)
    for (int x = std::max(0, static_cast<int>(grid.info.width) - border_thickness); 
         x < static_cast<int>(grid.info.width); x++) {
      for (int y = 0; y < static_cast<int>(grid.info.height); y++) {
        int index = x + y * grid.info.width;
        result_grid.data[index] = 100;
      }
    }
    
    // Count marked cells for verification
    int marked_cells = 0;
    for (int8_t value : result_grid.data) {
      if (value == 100) marked_cells++;
    }
    
    // RCLCPP_INFO(this->get_logger(), "Grid borders marked as occupied: %d cells marked (border thickness: %d)", 
    //             marked_cells, border_thickness);
    return result_grid;
  }

  // Convert world point to grid pixel coordinates
  bool worldToGrid(const nav_msgs::msg::OccupancyGrid& grid, float wx, float wy, int& gx, int& gy) {
    float rel_x = (wx - grid.info.origin.position.x) / grid_resolution;
    float rel_y = (wy - grid.info.origin.position.y) / grid_resolution;
    // Use floor and clamp to ensure max-bound vertices map inside the grid
    gx = static_cast<int>(std::floor(rel_x));
    gy = static_cast<int>(std::floor(rel_y));
    if (gx < 0) gx = 0; else if (gx >= static_cast<int>(grid.info.width)) gx = static_cast<int>(grid.info.width) - 1;
    if (gy < 0) gy = 0; else if (gy >= static_cast<int>(grid.info.height)) gy = static_cast<int>(grid.info.height) - 1;
    return true;
  }

  // Mark polygon boundary as occupied: draw rectangle boundary with 2.5m margin around polygon bbox
  nav_msgs::msg::OccupancyGrid markPolygonBoundaryAsOccupied(const nav_msgs::msg::OccupancyGrid& grid) {
    nav_msgs::msg::OccupancyGrid result_grid = grid;
    
    std::lock_guard<std::mutex> lock(area_polygon_mutex_);
    
    if (!hardcoded_polygon_points_.empty()) {
      // Mark rectangular boundary with 2.5m margin added to polygon's bounding box as occupied
      const double margin = 2.5;
      double rect_minx = hardcoded_minx_ - margin;
      double rect_maxx = hardcoded_maxx_ + margin;
      double rect_miny = hardcoded_miny_ - margin;
      double rect_maxy = hardcoded_maxy_ + margin;
      
      // Convert four corner coordinates of rectangle to grid coordinates
      int gx_min, gy_min, gx_max, gy_max;
      if (worldToGrid(result_grid, static_cast<float>(rect_minx), static_cast<float>(rect_miny), gx_min, gy_min) &&
          worldToGrid(result_grid, static_cast<float>(rect_maxx), static_cast<float>(rect_maxy), gx_max, gy_max)) {
        // Draw rectangular boundary (using Bresenham algorithm)
        // Top boundary
        drawLineInGrid(result_grid, gx_min, gy_min, gx_max, gy_min);
        // Bottom boundary
        drawLineInGrid(result_grid, gx_min, gy_max, gx_max, gy_max);
        // Left boundary
        drawLineInGrid(result_grid, gx_min, gy_min, gx_min, gy_max);
        // Right boundary
        drawLineInGrid(result_grid, gx_max, gy_min, gx_max, gy_max);
      }
    } else if (!area_polygon_received || area_polygon_points.empty()) {
      // Mark grid boundary as occupied if no polygon
      return markBoundariesAsOccupied(grid);
    } else {
      // Display existing polygon boundary (for compatibility)
      cv::Mat mat = occupancyGridToMat(grid);
      std::vector<cv::Point> pts;
      pts.reserve(area_polygon_points.size());
      
      for (const auto& p : area_polygon_points) {
        int gx, gy;
        if (worldToGrid(grid, p.x, p.y, gx, gy)) {
          pts.emplace_back(gx, gy);
        }
      }
      
      if (pts.size() >= 2) {
        const cv::Point* p_data = pts.data();
        int npts = static_cast<int>(pts.size());
        cv::polylines(mat, &p_data, &npts, 1, true, cv::Scalar(255), 1, cv::LINE_8);
      }
      
      return matToOccupancyGrid(mat, grid);
    }
    
    return result_grid;
  }

  // Draw straight line between two grid coordinates using Bresenham algorithm
  void drawLineInGrid(nav_msgs::msg::OccupancyGrid& grid, int x0, int y0, int x1, int y1) {
    // Check grid bounds
    int width = static_cast<int>(grid.info.width);
    int height = static_cast<int>(grid.info.height);
    
    // Clip if out of bounds
    x0 = std::max(0, std::min(width - 1, x0));
    y0 = std::max(0, std::min(height - 1, y0));
    x1 = std::max(0, std::min(width - 1, x1));
    y1 = std::max(0, std::min(height - 1, y1));
    
    // Bresenham algorithm
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
      // Mark current point as occupied cell
      int index = x + y * width;
      if (index >= 0 && index < static_cast<int>(grid.data.size())) {
        grid.data[index] = 100;
      }
      
      if (x == x1 && y == y1) {
        break;
      }
      
      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }
  }


  // Determine active bounds: polygon bbox with 2.5m margin if available, otherwise clipping params
  void getActiveBounds(float& out_minx, float& out_maxx, float& out_miny, float& out_maxy) {
    std::lock_guard<std::mutex> lock(area_polygon_mutex_);
    
    if (!hardcoded_polygon_points_.empty()) {
      // Add 2.5m margin to polygon's bounding box
      const double margin = 2.5;
      out_minx = static_cast<float>(hardcoded_minx_ - margin);
      out_maxx = static_cast<float>(hardcoded_maxx_ + margin);
      out_miny = static_cast<float>(hardcoded_miny_ - margin);
      out_maxy = static_cast<float>(hardcoded_maxy_ + margin);
    } else {
      out_minx = clipping_minx;
      out_maxx = clipping_maxx;
      out_miny = clipping_miny;
      out_maxy = clipping_maxy;
    }
  }

  // Remove occupied cells near waypoints
  nav_msgs::msg::OccupancyGrid removeWaypointAreas(const nav_msgs::msg::OccupancyGrid& grid) {
    
    nav_msgs::msg::OccupancyGrid result_grid = grid;
    float removal_radius = 0.5;  // 0.5 meter radius
    int removal_cells = static_cast<int>(removal_radius / grid_resolution);
    
    int removed_cells = 0;
    
    for (const auto& waypoint : waypoints) {
      float wp_x = waypoint.first;
      float wp_y = waypoint.second;
      
      // Convert waypoint world coordinates to grid coordinates
      int grid_x = static_cast<int>((wp_x - grid.info.origin.position.x) / grid_resolution);
      int grid_y = static_cast<int>((wp_y - grid.info.origin.position.y) / grid_resolution);
      
      // Remove occupied cells in circular area around waypoint
      for (int dy = -removal_cells; dy <= removal_cells; dy++) {
        for (int dx = -removal_cells; dx <= removal_cells; dx++) {
          if (dx * dx + dy * dy <= removal_cells * removal_cells) {  // Circular pattern
            int new_x = grid_x + dx;
            int new_y = grid_y + dy;
            
            if (new_x >= 0 && new_x < static_cast<int>(grid.info.width) && 
                new_y >= 0 && new_y < static_cast<int>(grid.info.height)) {
              int index = new_x + new_y * grid.info.width;
              if (result_grid.data[index] == 100) {  // If occupied
                result_grid.data[index] = 0;  // Make free
                removed_cells++;
              }
            }
          }
        }
      }
    }
    
    return result_grid;
  }

  // Apply inflation to remaining occupied cells after waypoint removal
  nav_msgs::msg::OccupancyGrid applyInflation(const nav_msgs::msg::OccupancyGrid& grid) {
    
    nav_msgs::msg::OccupancyGrid result_grid = grid;
    int inflation_cells = static_cast<int>(inflation_radius / grid_resolution);
    
    // Create a copy to avoid modifying the original while iterating
    nav_msgs::msg::OccupancyGrid original_grid = grid;
    
    // Find all occupied cells and apply inflation around them
    for (int y = 0; y < static_cast<int>(grid.info.height); y++) {
      for (int x = 0; x < static_cast<int>(grid.info.width); x++) {
        int index = x + y * grid.info.width;
        
        if (original_grid.data[index] == 100) {  // If occupied
          // Apply inflation around this occupied cell
          for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
            for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
              if (dx * dx + dy * dy <= inflation_cells * inflation_cells) {  // Circular pattern
                int new_x = x + dx;
                int new_y = y + dy;
                
                if (new_x >= 0 && new_x < static_cast<int>(grid.info.width) && 
                    new_y >= 0 && new_y < static_cast<int>(grid.info.height)) {
                  int inflate_index = new_x + new_y * grid.info.width;
                  result_grid.data[inflate_index] = 100;  // Mark as occupied
                }
              }
            }
          }
        }
      }
    }
    
    return result_grid;
  }

  // Cluster occupied cells in skeletonized grid
  std::vector<Cluster> clusterOccupiedCells(const nav_msgs::msg::OccupancyGrid& grid) {
    std::vector<Cluster> clusters;
    std::vector<std::vector<bool>> visited(grid.info.height, std::vector<bool>(grid.info.width, false));
    
    // Get polygon information (for filtering)
    std::vector<std::pair<double, double>> polygon_points;
    bool use_polygon_filter = false;
    {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      if (!hardcoded_polygon_points_.empty()) {
        polygon_points = hardcoded_polygon_points_;
        use_polygon_filter = true;
      }
    }
    
    // Directions for 8-connected components
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    for (int y = 0; y < static_cast<int>(grid.info.height); y++) {
      for (int x = 0; x < static_cast<int>(grid.info.width); x++) {
        int index = x + y * grid.info.width;
        
        // If occupied and not visited, check if it's inside polygon
        if (grid.data[index] == 100 && !visited[y][x]) {
          // Polygon filtering: ignore if polygon exists and cell is outside polygon
          if (use_polygon_filter) {
            // Convert grid coordinates to world coordinates
            float world_x = grid.info.origin.position.x + static_cast<float>(x) * grid_resolution;
            float world_y = grid.info.origin.position.y + static_cast<float>(y) * grid_resolution;
            
            // Ignore if outside polygon
            if (!isPointInPolygon(world_x, world_y, polygon_points)) {
              visited[y][x] = true;  // Mark as visited to avoid rechecking
              continue;
            }
          }
          Cluster cluster;
          std::queue<std::pair<int, int>> q;
          q.push({x, y});
          visited[y][x] = true;
          
          // BFS to find all connected occupied cells
          while (!q.empty()) {
            auto current = q.front();
            q.pop();
            
            int cx = current.first;
            int cy = current.second;
            cluster.cells.push_back({cx, cy});
            
            // Check 8 neighbors
            for (int i = 0; i < 8; i++) {
              int nx = cx + dx[i];
              int ny = cy + dy[i];
              
              if (nx >= 0 && nx < static_cast<int>(grid.info.width) &&
                  ny >= 0 && ny < static_cast<int>(grid.info.height)) {
                int neighbor_index = nx + ny * grid.info.width;
                
                if (!visited[ny][nx] && grid.data[neighbor_index] == 100) {
                  // Polygon filtering: ignore if polygon exists and neighbor cell is outside polygon
                  if (use_polygon_filter) {
                    // Convert grid coordinates to world coordinates
                    float neighbor_world_x = grid.info.origin.position.x + static_cast<float>(nx) * grid_resolution;
                    float neighbor_world_y = grid.info.origin.position.y + static_cast<float>(ny) * grid_resolution;
                    
                    // Ignore if outside polygon
                    if (!isPointInPolygon(neighbor_world_x, neighbor_world_y, polygon_points)) {
                      visited[ny][nx] = true;  // Mark as visited to avoid rechecking
                      continue;
                    }
                  }
                  
                  visited[ny][nx] = true;
                  q.push({nx, ny});
                }
              }
            }
          }
          
          // Calculate cluster center and maximum length
          if (!cluster.cells.empty()) {
            float sum_x = 0.0f, sum_y = 0.0f;
            for (const auto& cell : cluster.cells) {
              sum_x += cell.first;
              sum_y += cell.second;
            }
            cluster.center_x = sum_x / cluster.cells.size();
            cluster.center_y = sum_y / cluster.cells.size();
            cluster.size = cluster.cells.size();
            
            // Calculate maximum length (distance between farthest cells)
            float max_distance = 0.0f;
            for (size_t i = 0; i < cluster.cells.size(); i++) {
              for (size_t j = i + 1; j < cluster.cells.size(); j++) {
                int dx = cluster.cells[i].first - cluster.cells[j].first;
                int dy = cluster.cells[i].second - cluster.cells[j].second;
                float distance = std::sqrt(dx * dx + dy * dy) * grid_resolution;
                if (distance > max_distance) {
                  max_distance = distance;
                }
              }
            }
            cluster.length = max_distance;
            
            clusters.push_back(cluster);
          }
        }
      }
    }
    
    return clusters;
  }


  // Get color for a cluster based on its index
  void getClusterColor(size_t cluster_index, float& r, float& g, float& b) {
    // Use HSV color space for better color distribution
    float hue = fmod(cluster_index * 137.508f, 360.0f);  // Golden angle approximation for even distribution
    float saturation = 0.8f;
    float value = 0.9f;
    
    // Convert HSV to RGB
    float c = value * saturation;
    float x = c * (1.0f - fabs(fmod(hue / 60.0f, 2.0f) - 1.0f));
    float m = value - c;
    
    if (hue < 60.0f) {
      r = c + m; g = x + m; b = m;
    } else if (hue < 120.0f) {
      r = x + m; g = c + m; b = m;
    } else if (hue < 180.0f) {
      r = m; g = c + m; b = x + m;
    } else if (hue < 240.0f) {
      r = m; g = x + m; b = c + m;
    } else if (hue < 300.0f) {
      r = x + m; g = m; b = c + m;
    } else {
      r = c + m; g = m; b = x + m;
    }
  }

  // Visualize clusters as markers with min/max x points
  void visualizeClusters(const std::vector<Cluster>& clusters, const nav_msgs::msg::OccupancyGrid& grid) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete all previous markers first
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = "map";
    delete_all.header.stamp = this->get_clock()->now();
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);
    
    // Create markers for each cluster
    for (size_t i = 0; i < clusters.size(); i++) {
      const auto& cluster = clusters[i];
      
      if (cluster.size < 3) continue;  // Skip very small clusters
      
      // Get color for this cluster
      float r, g, b;
      getClusterColor(i, r, g, b);
      
      if (cluster.cells.empty()) continue;
      
      float world_x = grid.info.origin.position.x + cluster.center_x * grid_resolution;
      float world_y = grid.info.origin.position.y + cluster.center_y * grid_resolution;
      
      // Create CUBE_LIST marker to visualize all occupied cells in this cluster
      visualization_msgs::msg::Marker cells_marker;
      cells_marker.header.frame_id = "map";
      cells_marker.header.stamp = this->get_clock()->now();
      cells_marker.ns = "cluster_cells";
      cells_marker.id = i;
      cells_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      cells_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Scale for each cube (grid resolution)
      cells_marker.scale.x = grid_resolution * 0.8;
      cells_marker.scale.y = grid_resolution * 0.8;
      cells_marker.scale.z = 0.1;
      
      // Set color for the cluster
      cells_marker.color.r = r;
      cells_marker.color.g = g;
      cells_marker.color.b = b;
      cells_marker.color.a = 0.7;
      
      std::vector<std::pair<double, double>> world_points;
      world_points.reserve(cluster.cells.size());

      // Add position for each cell in the cluster
      for (const auto& cell : cluster.cells) {
        geometry_msgs::msg::Point point;
        // Convert grid coordinates to world coordinates
        point.x = grid.info.origin.position.x + cell.first * grid_resolution;
        point.y = grid.info.origin.position.y + cell.second * grid_resolution;
        point.z = 0.0;
        cells_marker.points.push_back(point);
        world_points.emplace_back(point.x, point.y);
      }
      
      cells_marker.lifetime = rclcpp::Duration::from_seconds(0);  // Never expire
      marker_array.markers.push_back(cells_marker);
      
      // Create sphere marker for cluster center
      visualization_msgs::msg::Marker center_marker;
      center_marker.header.frame_id = "map";
      center_marker.header.stamp = this->get_clock()->now();
      center_marker.ns = "cluster_centers";
      center_marker.id = i;
      center_marker.type = visualization_msgs::msg::Marker::SPHERE;
      center_marker.action = visualization_msgs::msg::Marker::ADD;
      
      center_marker.pose.position.x = world_x;
      center_marker.pose.position.y = world_y;
      center_marker.pose.position.z = 0.05;
      center_marker.pose.orientation.w = 1.0;
      
      // Size based on cluster size
      float marker_radius = std::max(0.1f, std::min(0.3f, 0.03f * static_cast<float>(std::sqrt(cluster.size))));
      center_marker.scale.x = marker_radius;
      center_marker.scale.y = marker_radius;
      center_marker.scale.z = marker_radius;
      
      // Use cluster color
      center_marker.color.r = r;
      center_marker.color.g = g;
      center_marker.color.b = b;
      center_marker.color.a = 1.0f;
      
      center_marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker_array.markers.push_back(center_marker);
      
      // Create text marker for cluster ID and size
      visualization_msgs::msg::Marker text_marker;
      text_marker.header = center_marker.header;
      text_marker.ns = "cluster_labels";
      text_marker.id = i;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position.x = world_x;
      text_marker.pose.position.y = world_y;
      text_marker.pose.position.z = marker_radius + 0.3;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.8;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      text_marker.text = "C" + std::to_string(i) + " (" + std::to_string(cluster.size) + ")\nL:" + std::to_string((int)cluster.length) + "m";
      text_marker.lifetime = rclcpp::Duration::from_seconds(0);
      
      marker_array.markers.push_back(text_marker);
    }
    
    publisher_cluster_markers->publish(marker_array);
  }

  // Point-in-polygon algorithm (Ray casting algorithm)
  bool isPointInPolygon(double px, double py, const std::vector<std::pair<double, double>>& polygon) {
    if (polygon.size() < 3) {
      return false;
    }
    
    bool inside = false;
    size_t j = polygon.size() - 1;
    
    for (size_t i = 0; i < polygon.size(); ++i) {
      const auto& pi = polygon[i];
      const auto& pj = polygon[j];
      
      // Ray casting algorithm
      double dy = pj.second - pi.second;
      if (std::abs(dy) > 1e-9) {  // Prevent division by zero
        if (((pi.second > py) != (pj.second > py)) &&
            (px < (pj.first - pi.first) * (py - pi.second) / dy + pi.first)) {
          inside = !inside;
        }
      }
      j = i;
    }
    
    return inside;
  }

  // Cluster and visualize skeletonized grid
  void clusterAndVisualizeSkeletonizedGrid(const nav_msgs::msg::OccupancyGrid& grid) {
    // Cluster occupied cells
    std::vector<Cluster> clusters = clusterOccupiedCells(grid);
    
    // Filter clusters by length (minimum length)
    std::vector<Cluster> length_filtered_clusters;
    float min_length = static_cast<float>(cluster_min_length_m_);
    
    for (const auto& cluster : clusters) {
      if (cluster.length >= min_length) {
        length_filtered_clusters.push_back(cluster);
      }
    }
    
    
    // Filter clusters inside area (for visualization) - check if cluster center point is inside polygon
    std::vector<Cluster> exploration_clusters;
    {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      
      if (!hardcoded_polygon_points_.empty()) {
        for (const auto& cluster : length_filtered_clusters) {
          if (cluster.cells.empty()) {
            continue;
          }
          
          // Check if cluster's center point is inside polygon
          float center_x = grid.info.origin.position.x + cluster.center_x * grid_resolution;
          float center_y = grid.info.origin.position.y + cluster.center_y * grid_resolution;
          
          if (isPointInPolygon(center_x, center_y, hardcoded_polygon_points_)) {
            exploration_clusters.push_back(cluster);
          }
        }
      } else {
        // Add all clusters as exploration clusters if no polygon
        exploration_clusters = length_filtered_clusters;
      }
    }
    
    // Visualize clusters (only clusters inside area)
    visualizeClusters(exploration_clusters, grid);
    
    // Publish cluster information sorted by y-value (only clusters inside area)
    publishClusterInfo(exploration_clusters, grid);
    
    // Convert clusters to tree rows and store/publish (use all clusters)
    convertClustersToTreeRows(length_filtered_clusters, grid);
  }
  
  // Convert clusters to tree rows
  void convertClustersToTreeRows(const std::vector<Cluster>& clusters, const nav_msgs::msg::OccupancyGrid& grid) {
    if (clusters.empty()) return;
    
    rclcpp::Time now = this->get_clock()->now();
    std::string frame_id = grid.header.frame_id;
    
    // Get polygon information (for filtering)
    std::vector<std::pair<double, double>> polygon_points;
    bool use_polygon_filter = false;
    {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      if (!hardcoded_polygon_points_.empty()) {
        polygon_points = hardcoded_polygon_points_;
        use_polygon_filter = true;
      }
    }
    
    // Convert clusters to tree rows (only clusters inside polygon)
    std::vector<TreeRowFromCluster> all_tree_rows;
    
    for (const auto& cluster : clusters) {
      if (cluster.cells.empty()) continue;
      
      // Calculate cluster center
      float center_x = grid.info.origin.position.x + cluster.center_x * grid_resolution;
      float center_y = grid.info.origin.position.y + cluster.center_y * grid_resolution;
      
      // Skip if polygon exists and cluster center is outside polygon
      if (use_polygon_filter) {
        if (!isPointInPolygon(center_x, center_y, polygon_points)) {
          continue;  // Don't convert clusters outside polygon to tree rows
        }
      }
      
      TreeRowFromCluster row;
      row.center = Eigen::Vector2d(center_x, center_y);
      
      // Convert all cells of cluster to world coordinates
      std::vector<Eigen::Vector2d> world_points;
      for (const auto& cell : cluster.cells) {
        float wx = grid.info.origin.position.x + cell.first * grid_resolution;
        float wy = grid.info.origin.position.y + cell.second * grid_resolution;
        world_points.push_back(Eigen::Vector2d(wx, wy));
      }
      
      // Find farthest point from center (first direction)
      double max_dist_sq = 0.0;
      size_t first_idx = 0;
      Eigen::Vector2d first_direction;
      
      for (size_t i = 0; i < world_points.size(); ++i) {
        Eigen::Vector2d diff = world_points[i] - row.center;
        double dist_sq = diff.squaredNorm();
        if (dist_sq > max_dist_sq) {
          max_dist_sq = dist_sq;
          first_idx = i;
          first_direction = diff.normalized();
        }
      }
      
      // Find farthest point in opposite direction from first point
      double max_opposite_dist_sq = 0.0;
      size_t second_idx = 0;
      
      for (size_t i = 0; i < world_points.size(); ++i) {
        if (i == first_idx) continue;
        
        Eigen::Vector2d diff = world_points[i] - row.center;
        double dist_sq = diff.squaredNorm();
        
        // Check if opposite direction (dot product is negative)
        double dot_product = diff.normalized().dot(first_direction);
        if (dot_product < 0.0 && dist_sq > max_opposite_dist_sq) {
          max_opposite_dist_sq = dist_sq;
          second_idx = i;
        }
      }
      
      // If opposite direction point not found, select point farthest from first point
      if (max_opposite_dist_sq == 0.0) {
        max_opposite_dist_sq = 0.0;
        for (size_t i = 0; i < world_points.size(); ++i) {
          if (i == first_idx) continue;
          Eigen::Vector2d diff = world_points[i] - world_points[first_idx];
          double dist_sq = diff.squaredNorm();
          if (dist_sq > max_opposite_dist_sq) {
            max_opposite_dist_sq = dist_sq;
            second_idx = i;
          }
        }
      }
      
      row.start_point = world_points[first_idx];
      row.end_point = world_points[second_idx];
      row.length = cluster.length;
      
      all_tree_rows.push_back(row);
    }
    
    // Filter tree rows inside area (already converted only clusters inside polygon, so use as-is if polygon exists)
    std::vector<TreeRowFromCluster> exploration_tree_rows;
    {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      if (!hardcoded_polygon_points_.empty()) {
        // Use already filtered tree rows if polygon exists (already filtered by center point)
        exploration_tree_rows = all_tree_rows;
      } else {
        // Add all tree rows as exploration tree rows if no polygon
        exploration_tree_rows = all_tree_rows;
      }
    }
    
    // Store tree row information (for adding lines to grid)
    {
      std::lock_guard<std::mutex> lock(last_tree_rows_mutex_);
      last_tree_rows_ = all_tree_rows;
    }
    
    // Publish all tree rows and tree rows inside area separately
    publishTreeRowsAllFromClusters(all_tree_rows, now, frame_id);
    publishTreeRowsExplorationFromClusters(exploration_tree_rows, now, frame_id);
    
    // Get filtered trees (use empty array since tree_tracking_node was removed)
    geometry_msgs::msg::PoseArray filtered_trees;
    
    // Generate virtual seeds (at 1m intervals between start and end points of tree rows)
    // Get skeletonized grid (for ray casting)
    nav_msgs::msg::OccupancyGrid skeletonized_grid_for_seeds;
    {
      std::lock_guard<std::mutex> lock(skeletonized_grid_mutex_);
      skeletonized_grid_for_seeds = last_skeletonized_grid_;
    }
    generateVirtualSeeds(all_tree_rows, skeletonized_grid_for_seeds);
    
    // Generate seeds by ray casting from tree row endpoints (recalculate each time)
    {
      std::lock_guard<std::mutex> lock(seeds_mutex_);
      // Remove previous ray points and generate new ones
      ray_seeds_from_endpoints_.clear();
      generateRayPointsFromEndpoints(exploration_tree_rows, skeletonized_grid_for_seeds, ray_seeds_from_endpoints_);
      
      // Add start and end points of tree rows inside polygon as seeds
      tree_row_endpoint_seeds_.clear();
      for (const auto& row : exploration_tree_rows) {
        // Add start point
        geometry_msgs::msg::Point start_point;
        start_point.x = row.start_point.x();
        start_point.y = row.start_point.y();
        start_point.z = 0.0;
        
        // Check for duplicates
        bool already_exists = false;
        for (const auto& existing : tree_row_endpoint_seeds_) {
          double dist = std::sqrt(
            std::pow(existing.x - start_point.x, 2) +
            std::pow(existing.y - start_point.y, 2)
          );
          if (dist < 0.5) {
            already_exists = true;
            break;
          }
        }
        if (!already_exists) {
          tree_row_endpoint_seeds_.push_back(start_point);
        }
        
        // Add end point
        geometry_msgs::msg::Point end_point;
        end_point.x = row.end_point.x();
        end_point.y = row.end_point.y();
        end_point.z = 0.0;
        
        // Check for duplicates
        already_exists = false;
        for (const auto& existing : tree_row_endpoint_seeds_) {
          double dist = std::sqrt(
            std::pow(existing.x - end_point.x, 2) +
            std::pow(existing.y - end_point.y, 2)
          );
          if (dist < 0.5) {
            already_exists = true;
            break;
          }
        }
        if (!already_exists) {
          tree_row_endpoint_seeds_.push_back(end_point);
        }
      }
    }
    
    // Update real seeds at positions robot passed through to confirmed tree positions
    if (robot_position_received_) {
      convertVirtualSeedsToReal();
    }
    
    // 1. Publish /voronoi_seeds: includes virtual seeds, real seeds, and ray points
    publishVoronoiSeedsFromVirtualAndReal(now, frame_id);
    
    // 2. Visualize virtual and real seeds
    publishVirtualAndRealSeedsVisualization(now, frame_id);
    
    // 3. Publish tree row information in area: sorted by center's y-value (ascending), then by x-value if equal
    publishExplorationTreeRowsInfoFromClusters(exploration_tree_rows, now, frame_id);
  }

  // Publish cluster information with min and max x points
  void publishClusterInfo(const std::vector<Cluster>& clusters, const nav_msgs::msg::OccupancyGrid& grid) {
    if (clusters.empty()) return;
    
    // Create list with cluster info
    struct ClusterInfo {
      size_t index;
      float center_x, center_y;
    };
    
    std::vector<ClusterInfo> cluster_infos;
    
    for (size_t i = 0; i < clusters.size(); i++) {
      const auto& cluster = clusters[i];
      
      if (cluster.cells.empty()) continue;
      
      // Get cluster center
      float center_x = grid.info.origin.position.x + cluster.center_x * grid_resolution;
      float center_y = grid.info.origin.position.y + cluster.center_y * grid_resolution;
      
      ClusterInfo info;
      info.index = i;
      info.center_x = center_x;
      info.center_y = center_y;
      
      cluster_infos.push_back(info);
    }
    
    // Sort by center y-value (ascending)
    std::sort(cluster_infos.begin(), cluster_infos.end(), 
              [](const ClusterInfo& a, const ClusterInfo& b) {
                return a.center_y < b.center_y;
              });
    
    // Create PoseArray with cluster centers
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->get_clock()->now();
    
    // Add cluster centers
    for (const auto& info : cluster_infos) {
      geometry_msgs::msg::Pose center_pose;
      center_pose.position.x = info.center_x;
      center_pose.position.y = info.center_y;
      center_pose.position.z = 0.0;
      center_pose.orientation.w = 1.0;
      pose_array.poses.push_back(center_pose);
    }
    
    publisher_cluster_info->publish(pose_array);
  }
  
  void publishTreeRowsAllFromClusters(const std::vector<TreeRowFromCluster>& tree_rows, const rclcpp::Time& stamp, const std::string& frame_id) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete existing markers
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id.empty() ? "map" : frame_id;
    delete_all.header.stamp = stamp;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);
    
    // Visualize all tree rows (gray)
    for (size_t i = 0; i < tree_rows.size(); ++i) {
      const auto& row = tree_rows[i];
      
      visualization_msgs::msg::Marker row_marker;
      row_marker.header.stamp = stamp;
      row_marker.header.frame_id = frame_id;
      row_marker.ns = "/tree_rows_all";
      row_marker.id = static_cast<int>(i);
      row_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      row_marker.action = visualization_msgs::msg::Marker::ADD;
      row_marker.scale.x = 0.08;
      row_marker.color.r = 0.5f;  // Gray
      row_marker.color.g = 0.5f;
      row_marker.color.b = 0.5f;
      row_marker.color.a = 0.6f;
      
      // Use only start and end points
      geometry_msgs::msg::Point p1;
      p1.x = row.start_point.x();
      p1.y = row.start_point.y();
      p1.z = 0.0;
      row_marker.points.push_back(p1);
      
      geometry_msgs::msg::Point p2;
      p2.x = row.end_point.x();
      p2.y = row.end_point.y();
      p2.z = 0.0;
      row_marker.points.push_back(p2);
      
      marker_array.markers.push_back(row_marker);
    }
    
    publisher_tree_rows_all->publish(marker_array);
  }
  
  void publishTreeRowsExplorationFromClusters(const std::vector<TreeRowFromCluster>& tree_rows, const rclcpp::Time& stamp, const std::string& frame_id) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete existing markers
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id.empty() ? "map" : frame_id;
    delete_all.header.stamp = stamp;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);
    
    // Sort tree rows in area by center's y-value (ascending), then by x-value if equal
    std::vector<TreeRowFromCluster> sorted_rows = tree_rows;
    std::sort(sorted_rows.begin(), sorted_rows.end(),
             [](const TreeRowFromCluster& a, const TreeRowFromCluster& b) {
               if (std::abs(a.center.y() - b.center.y()) < 1e-6) {
                 return a.center.x() < b.center.x();
               }
               return a.center.y() < b.center.y();
             });
    
    // Visualize tree rows inside area (green)
    for (size_t i = 0; i < sorted_rows.size(); ++i) {
      const auto& row = sorted_rows[i];
      
      visualization_msgs::msg::Marker row_marker;
      row_marker.header.stamp = stamp;
      row_marker.header.frame_id = frame_id;
      row_marker.ns = "/tree_rows_exploration";
      row_marker.id = static_cast<int>(i);
      row_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      row_marker.action = visualization_msgs::msg::Marker::ADD;
      row_marker.scale.x = 0.12;
      row_marker.color.r = 0.0f;  // Green
      row_marker.color.g = 1.0f;
      row_marker.color.b = 0.0f;
      row_marker.color.a = 0.9f;
      
      // Use only start and end points
      geometry_msgs::msg::Point p1;
      p1.x = row.start_point.x();
      p1.y = row.start_point.y();
      p1.z = 0.0;
      row_marker.points.push_back(p1);
      
      geometry_msgs::msg::Point p2;
      p2.x = row.end_point.x();
      p2.y = row.end_point.y();
      p2.z = 0.0;
      row_marker.points.push_back(p2);
      
      marker_array.markers.push_back(row_marker);
    }
    
    publisher_tree_rows_exploration->publish(marker_array);
  }
  
  // Publish virtual and real seeds as voronoi_seeds (replaces existing Voronoi seeds)
  void publishVoronoiSeedsFromVirtualAndReal(const rclcpp::Time& stamp, const std::string& frame_id) {
    std::lock_guard<std::mutex> lock(seeds_mutex_);
    
    geometry_msgs::msg::PoseArray voronoi_seeds;
    voronoi_seeds.header.stamp = stamp;
    voronoi_seeds.header.frame_id = frame_id;
    
    // Add virtual seeds (include all virtual seeds)
    for (const auto& seed : virtual_seeds_) {
      geometry_msgs::msg::Pose seed_pose;
      seed_pose.position = seed.position;
      seed_pose.orientation.w = 1.0;
      voronoi_seeds.poses.push_back(seed_pose);
    }
    
    // Add real seeds (taken from confirmed tree positions)
    for (const auto& real_seed : real_seeds_) {
      geometry_msgs::msg::Pose seed_pose;
      seed_pose.position = real_seed;
      seed_pose.orientation.w = 1.0;
      voronoi_seeds.poses.push_back(seed_pose);
    }
    
    // Add ray points generated from tree row endpoints
    for (const auto& ray_seed : ray_seeds_from_endpoints_) {
      geometry_msgs::msg::Pose seed_pose;
      seed_pose.position = ray_seed;
      seed_pose.orientation.w = 1.0;
      voronoi_seeds.poses.push_back(seed_pose);
    }
    
    // Add start and end points of tree rows inside polygon
    for (const auto& endpoint_seed : tree_row_endpoint_seeds_) {
      geometry_msgs::msg::Pose seed_pose;
      seed_pose.position = endpoint_seed;
      seed_pose.orientation.w = 1.0;
      voronoi_seeds.poses.push_back(seed_pose);
    }
    
    publisher_voronoi_seeds->publish(voronoi_seeds);
  }
  
  // Keep existing function for compatibility (not used)
  void publishVoronoiSeedsFromClusters(const std::vector<TreeRowFromCluster>& /* all_tree_rows */, 
                          const geometry_msgs::msg::PoseArray& /* filtered_trees */,
                          const rclcpp::Time& stamp, const std::string& frame_id) {
    // This function is no longer used - replaced by publishVoronoiSeedsFromVirtualAndReal
    publishVoronoiSeedsFromVirtualAndReal(stamp, frame_id);
  }
  
  // Keep existing function for compatibility (not used)
  void publishVoronoiSeedsVisualizationFromClusters(const std::vector<TreeRowFromCluster>& /* all_tree_rows */,
                                       const geometry_msgs::msg::PoseArray& /* filtered_trees */,
                                       const rclcpp::Time& stamp, const std::string& frame_id) {
    // This function is no longer used - replaced by publishVirtualAndRealSeedsVisualization
    // But publish virtual/real seed visualization for compatibility
    publishVirtualAndRealSeedsVisualization(stamp, frame_id);
  }
  
  // Find collision point in vertical direction using ray casting
  bool raycastToOccupiedCell(const nav_msgs::msg::OccupancyGrid& grid, 
                              double start_x, double start_y,
                              double dir_x, double dir_y,
                              double max_distance,
                              double& hit_x, double& hit_y) {
    const double step_size = grid_resolution * 0.5;  // Move by half of grid resolution
    const int max_steps = static_cast<int>(max_distance / step_size);
    const double min_distance = 1.0;  // Minimum distance 1m (ignore collisions less than 1m)
    
    double current_x = start_x;
    double current_y = start_y;
    
    for (int i = 0; i < max_steps; ++i) {
      current_x += dir_x * step_size;
      current_y += dir_y * step_size;
      
      // Calculate distance to current position
      double dx = current_x - start_x;
      double dy = current_y - start_y;
      double distance = std::sqrt(dx * dx + dy * dy);
      
      // Ignore and continue if less than minimum distance
      if (distance < min_distance) {
        continue;
      }
      
      // Convert to grid coordinates
      int gx, gy;
      if (worldToGrid(grid, static_cast<float>(current_x), static_cast<float>(current_y), gx, gy)) {
        int index = gx + gy * grid.info.width;
        if (index >= 0 && index < static_cast<int>(grid.data.size())) {
          if (grid.data[index] == 100) {  // Occupied cell found
            hit_x = current_x;
            hit_y = current_y;
            return true;
          }
        }
      }
    }
    
    return false;  // No collision
  }
  
  // Generate seeds by ray casting from tree row endpoints
  geometry_msgs::msg::Point castRayFromEndpoint(
      const geometry_msgs::msg::Point& start_point,
      const geometry_msgs::msg::Point& other_endpoint,
      double angle_offset_deg,
      const nav_msgs::msg::OccupancyGrid& skeletonized_grid,
      double min_distance = 1.0,
      double max_distance = 6.0) {
    
    Eigen::Vector2d start(start_point.x, start_point.y);
    Eigen::Vector2d other(other_endpoint.x, other_endpoint.y);
    
    Eigen::Vector2d endpoint_to_other = other - start;
    double dist_to_other = endpoint_to_other.norm();
    if (dist_to_other < 1e-6) {
      endpoint_to_other = Eigen::Vector2d(1.0, 0.0);
    } else {
      endpoint_to_other.normalize();
    }
    
    Eigen::Vector2d outward_dir = -endpoint_to_other;
    Eigen::Vector2d perp_dir(-endpoint_to_other.y(), endpoint_to_other.x());
    
    double angle_offset_rad = angle_offset_deg * M_PI / 180.0;
    
    Eigen::Vector2d ray_dir;
    if (angle_offset_deg > 0) {
      ray_dir = std::cos(angle_offset_rad) * outward_dir + std::sin(angle_offset_rad) * perp_dir;
    } else {
      ray_dir = std::cos(-angle_offset_rad) * outward_dir + std::sin(-angle_offset_rad) * (-perp_dir);
    }
    ray_dir.normalize();
    
    auto isInsideBounds = [&](const Eigen::Vector2d& point) -> bool {
      const double minx = skeletonized_grid.info.origin.position.x;
      const double maxx = minx + skeletonized_grid.info.width * skeletonized_grid.info.resolution;
      const double miny = skeletonized_grid.info.origin.position.y;
      const double maxy = miny + skeletonized_grid.info.height * skeletonized_grid.info.resolution;
      return point.x() >= minx && point.x() <= maxx && 
             point.y() >= miny && point.y() <= maxy;
    };
    
    // Function to check occupied cells in skeleton grid map
    auto isOccupiedInGrid = [&](const Eigen::Vector2d& point) -> bool {
      const double resolution = skeletonized_grid.info.resolution;
      const int width = static_cast<int>(skeletonized_grid.info.width);
      const int height = static_cast<int>(skeletonized_grid.info.height);

      int mx = static_cast<int>((point.x() - skeletonized_grid.info.origin.position.x) / resolution);
      int my = static_cast<int>((point.y() - skeletonized_grid.info.origin.position.y) / resolution);

      if (mx >= 0 && mx < width && my >= 0 && my < height) {
        int index = mx + my * width;
        if (index >= 0 && index < static_cast<int>(skeletonized_grid.data.size())) {
          return skeletonized_grid.data[index] == 100;
        }
      }
      return false;
    };
    
    const double step_size = 0.1;
    double current_dist = min_distance;
    
    // Continue until hitting grid boundary or occupied cell without max_distance limit
    double absolute_max_distance = max_distance;
    double grid_width = skeletonized_grid.info.width * skeletonized_grid.info.resolution;
    double grid_height = skeletonized_grid.info.height * skeletonized_grid.info.resolution;
    absolute_max_distance = std::sqrt(grid_width * grid_width + grid_height * grid_height) * 3.0;
    
    while (current_dist <= absolute_max_distance) {
      Eigen::Vector2d check_point = start + ray_dir * current_dist;
      
      if (!isInsideBounds(check_point)) {
        // Clip to grid boundary
        const double minx = skeletonized_grid.info.origin.position.x;
        const double maxx = minx + skeletonized_grid.info.width * skeletonized_grid.info.resolution;
        const double miny = skeletonized_grid.info.origin.position.y;
        const double maxy = miny + skeletonized_grid.info.height * skeletonized_grid.info.resolution;
        Eigen::Vector2d boundary_point = check_point;
        boundary_point.x() = std::max(minx, std::min(maxx, boundary_point.x()));
        boundary_point.y() = std::max(miny, std::min(maxy, boundary_point.y()));
        
        geometry_msgs::msg::Point result;
        result.x = boundary_point.x();
        result.y = boundary_point.y();
        result.z = 0.0;
        return result;
      }
      
      // Stop if passing through occupied cell in skeleton grid map
      if (isOccupiedInGrid(check_point)) {
        geometry_msgs::msg::Point result;
        result.x = check_point.x();
        result.y = check_point.y();
        result.z = 0.0;
        return result;
      }
      
      current_dist += step_size;
    }
    
    // Case when absolute maximum distance is reached
    Eigen::Vector2d final_point = start + ray_dir * absolute_max_distance;
    
    if (!isInsideBounds(final_point)) {
      const double minx = skeletonized_grid.info.origin.position.x;
      const double maxx = minx + skeletonized_grid.info.width * skeletonized_grid.info.resolution;
      const double miny = skeletonized_grid.info.origin.position.y;
      const double maxy = miny + skeletonized_grid.info.height * skeletonized_grid.info.resolution;
      final_point.x() = std::max(minx, std::min(maxx, final_point.x()));
      final_point.y() = std::max(miny, std::min(maxy, final_point.y()));
    }
    
    geometry_msgs::msg::Point result;
    result.x = final_point.x();
    result.y = final_point.y();
    result.z = 0.0;
    return result;
  }
  
  // Generate seeds by ray casting from tree row endpoints
  void generateRayPointsFromEndpoints(const std::vector<TreeRowFromCluster>& tree_rows,
                                      const nav_msgs::msg::OccupancyGrid& skeletonized_grid,
                                      std::vector<geometry_msgs::msg::Point>& ray_seeds) {
    ray_seeds.clear();
    
    // Get polygon information (for filtering)
    std::vector<std::pair<double, double>> polygon_points;
    bool use_polygon_filter = false;
    {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      if (!hardcoded_polygon_points_.empty()) {
        polygon_points = hardcoded_polygon_points_;
        use_polygon_filter = true;
      }
    }
    
    // Calculate max_distance dynamically according to grid size
    double effective_max_distance = 6.0;
    double grid_width = skeletonized_grid.info.width * skeletonized_grid.info.resolution;
    double grid_height = skeletonized_grid.info.height * skeletonized_grid.info.resolution;
    effective_max_distance = std::sqrt(grid_width * grid_width + grid_height * grid_height) * 2.0;
    
    for (const auto& row : tree_rows) {
      geometry_msgs::msg::Point ep1, ep2;
      ep1.x = row.start_point.x();
      ep1.y = row.start_point.y();
      ep1.z = 0.0;
      ep2.x = row.end_point.x();
      ep2.y = row.end_point.y();
      ep2.z = 0.0;
      
      // Cast rays in three directions from ep1
      geometry_msgs::msg::Point ray1_straight = castRayFromEndpoint(ep1, ep2, 0.0, skeletonized_grid, 1.0, effective_max_distance);
      geometry_msgs::msg::Point ray1_left = castRayFromEndpoint(ep1, ep2, -90.0, skeletonized_grid, 1.0, effective_max_distance);
      geometry_msgs::msg::Point ray1_right = castRayFromEndpoint(ep1, ep2, 90.0, skeletonized_grid, 1.0, effective_max_distance);
      
      // Cast rays in three directions from ep2
      geometry_msgs::msg::Point ray2_straight = castRayFromEndpoint(ep2, ep1, 0.0, skeletonized_grid, 1.0, effective_max_distance);
      geometry_msgs::msg::Point ray2_left = castRayFromEndpoint(ep2, ep1, -90.0, skeletonized_grid, 1.0, effective_max_distance);
      geometry_msgs::msg::Point ray2_right = castRayFromEndpoint(ep2, ep1, 90.0, skeletonized_grid, 1.0, effective_max_distance);
      
      // Add ray points to seeds (only if valid values, inside grid, and outside polygon)
      std::vector<geometry_msgs::msg::Point> ray_points = {
        ray1_straight, ray1_left, ray1_right,
        ray2_straight, ray2_left, ray2_right
      };
      
      for (const auto& ray_point : ray_points) {
        if (std::isfinite(ray_point.x) && std::isfinite(ray_point.y)) {
          // Check if inside grid
          const double minx = skeletonized_grid.info.origin.position.x;
          const double maxx = minx + skeletonized_grid.info.width * skeletonized_grid.info.resolution;
          const double miny = skeletonized_grid.info.origin.position.y;
          const double maxy = miny + skeletonized_grid.info.height * skeletonized_grid.info.resolution;
          
          bool is_inside_grid = ray_point.x >= minx && ray_point.x <= maxx && 
                               ray_point.y >= miny && ray_point.y <= maxy;
          
          if (!is_inside_grid) {
            continue;
          }
          
          // Don't add if inside polygon
          if (use_polygon_filter) {
            if (isPointInPolygon(ray_point.x, ray_point.y, polygon_points)) {
              continue;  // Skip if inside polygon
            }
          }
          
          // Check for duplicates
          bool already_exists = false;
          for (const auto& existing : ray_seeds) {
            double dist = std::sqrt(
              std::pow(existing.x - ray_point.x, 2) +
              std::pow(existing.y - ray_point.y, 2)
            );
            if (dist < 0.5) {
              already_exists = true;
              break;
            }
          }
          
          if (!already_exists) {
            ray_seeds.push_back(ray_point);
          }
        }
      }
    }
  }

  // Virtual seed generation function: generate virtual seeds at 1m intervals between start and end points of tree rows
  // Add seeds at collision points by ray casting in vertical direction for each seed
  // Regenerate each time tree rows are read (to handle false detections)
  void generateVirtualSeeds(const std::vector<TreeRowFromCluster>& tree_rows, 
                            const nav_msgs::msg::OccupancyGrid& skeletonized_grid) {
    std::lock_guard<std::mutex> lock(seeds_mutex_);
    
    // Among existing virtual seeds, those already converted are maintained as real seeds,
    // so virtual seeds are regenerated each time (since tree rows may change)
    std::vector<VirtualSeed> new_virtual_seeds;
    std::vector<RayCast> new_ray_casts;  // Store ray casting information
    
    // Get polygon information (for filtering)
    std::vector<std::pair<double, double>> polygon_points;
    bool use_polygon_filter = false;
    {
      std::lock_guard<std::mutex> lock(area_polygon_mutex_);
      if (!hardcoded_polygon_points_.empty()) {
        polygon_points = hardcoded_polygon_points_;
        use_polygon_filter = true;
      }
    }
    
    // Generate virtual seeds for each tree row (only tree rows inside polygon)
    for (size_t row_idx = 0; row_idx < tree_rows.size(); ++row_idx) {
      const auto& row = tree_rows[row_idx];
      
      // Skip if polygon exists and tree row center is outside polygon
      if (use_polygon_filter) {
        if (!isPointInPolygon(row.center.x(), row.center.y(), polygon_points)) {
          continue;  // Don't generate seeds for tree rows outside polygon
        }
      }
      
      // Calculate distance between start and end points
      double dx = row.end_point.x() - row.start_point.x();
      double dy = row.end_point.y() - row.start_point.y();
      double distance = std::sqrt(dx * dx + dy * dy);
      
      if (distance < virtual_seed_interval_) {
        // Don't generate virtual seeds if distance is too short
        continue;
      }
      
      // Normalize tree row direction vector
      double row_dir_norm = std::sqrt(dx * dx + dy * dy);
      if (row_dir_norm < 1e-6) {
        continue;  // Skip if direction vector is too small
      }
      double row_dir_x = dx / row_dir_norm;
      double row_dir_y = dy / row_dir_norm;
      
      // Calculate perpendicular direction vectors (90 degree rotation: (-dy, dx) and (dy, -dx))
      double perp_dir1_x = -row_dir_y;
      double perp_dir1_y = row_dir_x;
      double perp_dir2_x = row_dir_y;
      double perp_dir2_y = -row_dir_x;
      
      // Generate virtual seeds at 1m intervals between start and end points
      int num_seeds = static_cast<int>(std::floor(distance / virtual_seed_interval_));
      
      for (int i = 1; i <= num_seeds; ++i) {
        double t = static_cast<double>(i) / (num_seeds + 1);
        
        // Base seed position (middle of tree row)
        double base_x = row.start_point.x() + t * dx;
        double base_y = row.start_point.y() + t * dy;
        
        // Add base seed
        VirtualSeed base_seed;
        base_seed.position.x = base_x;
        base_seed.position.y = base_y;
        base_seed.position.z = 0.0;
        base_seed.tree_row_index = static_cast<int>(row_idx);
        base_seed.is_converted = false;
        
        // Check if position already has confirmed tree position (real seed)
        bool is_already_real_seed = false;
        for (const auto& real_seed : real_seeds_) {
          double dist = std::sqrt(
            std::pow(real_seed.x - base_seed.position.x, 2) +
            std::pow(real_seed.y - base_seed.position.y, 2)
          );
          if (dist < 0.5) {
            is_already_real_seed = true;
            break;
          }
        }
        
        // Add base seed (only duplicate check) - middle seeds of tree row are added without polygon check
        if (!is_already_real_seed) {
          bool already_exists = false;
          for (const auto& existing_seed : new_virtual_seeds) {
            double dist = std::sqrt(
              std::pow(existing_seed.position.x - base_seed.position.x, 2) +
              std::pow(existing_seed.position.y - base_seed.position.y, 2)
            );
            if (dist < 0.5) {
              already_exists = true;
              break;
            }
          }
          
          if (!already_exists) {
            new_virtual_seeds.push_back(base_seed);
          }
        }
        
        // Add seeds at collision points by ray casting in vertical direction
        const double max_raycast_distance = 4.0;  // Maximum 4m
        
        // First vertical direction (perp_dir1)
        double hit_x1, hit_y1;
        bool hit1 = raycastToOccupiedCell(skeletonized_grid, base_x, base_y, 
                                          perp_dir1_x, perp_dir1_y, 
                                          max_raycast_distance, hit_x1, hit_y1);
        
        // Store ray information
        RayCast ray1;
        ray1.start_point.x = base_x;
        ray1.start_point.y = base_y;
        ray1.start_point.z = 0.0;
        
        // Determine seed position: collision point if hit, otherwise 4m point
        double seed_x1, seed_y1;
        if (hit1) {
          seed_x1 = hit_x1;
          seed_y1 = hit_y1;
          ray1.end_point.x = hit_x1;
          ray1.end_point.y = hit_y1;
          ray1.end_point.z = 0.0;
          ray1.hit = true;
        } else {
          // Add seed at 4m point if no collision
          seed_x1 = base_x + perp_dir1_x * max_raycast_distance;
          seed_y1 = base_y + perp_dir1_y * max_raycast_distance;
          ray1.end_point.x = seed_x1;
          ray1.end_point.y = seed_y1;
          ray1.end_point.z = 0.0;
          ray1.hit = false;
        }
        new_ray_casts.push_back(ray1);
        
        // Add seed (regardless of collision) - only if outside polygon
        {
          // Don't add at all if inside polygon (check before seed generation)
          bool skip_seed = false;
          if (use_polygon_filter) {
            if (isPointInPolygon(seed_x1, seed_y1, polygon_points)) {
              skip_seed = true;  // Skip if inside polygon
            }
          }
          
          if (!skip_seed) {
            VirtualSeed hit_seed1;
            hit_seed1.position.x = seed_x1;
            hit_seed1.position.y = seed_y1;
            hit_seed1.position.z = 0.0;
            hit_seed1.tree_row_index = static_cast<int>(row_idx);
            hit_seed1.is_converted = false;
            
            // Add after duplicate check
            bool already_exists = false;
            for (const auto& existing_seed : new_virtual_seeds) {
              double dist = std::sqrt(
                std::pow(existing_seed.position.x - hit_seed1.position.x, 2) +
                std::pow(existing_seed.position.y - hit_seed1.position.y, 2)
              );
              if (dist < 0.5) {
                already_exists = true;
                break;
              }
            }
            
            if (!already_exists) {
              // Also check for duplicates with real seeds
              bool is_real_seed = false;
              for (const auto& real_seed : real_seeds_) {
                double dist = std::sqrt(
                  std::pow(real_seed.x - hit_seed1.position.x, 2) +
                  std::pow(real_seed.y - hit_seed1.position.y, 2)
                );
                if (dist < 0.5) {
                  is_real_seed = true;
                  break;
                }
              }
              
              if (!is_real_seed) {
                new_virtual_seeds.push_back(hit_seed1);
              }
            }
          }
        }
        
        // Second vertical direction (perp_dir2)
        double hit_x2, hit_y2;
        bool hit2 = raycastToOccupiedCell(skeletonized_grid, base_x, base_y, 
                                           perp_dir2_x, perp_dir2_y, 
                                           max_raycast_distance, hit_x2, hit_y2);
        
        // Store ray information
        RayCast ray2;
        ray2.start_point.x = base_x;
        ray2.start_point.y = base_y;
        ray2.start_point.z = 0.0;
        
        // Determine seed position: collision point if hit, otherwise 4m point
        double seed_x2, seed_y2;
        if (hit2) {
          seed_x2 = hit_x2;
          seed_y2 = hit_y2;
          ray2.end_point.x = hit_x2;
          ray2.end_point.y = hit_y2;
          ray2.end_point.z = 0.0;
          ray2.hit = true;
        } else {
          // Add seed at 4m point if no collision
          seed_x2 = base_x + perp_dir2_x * max_raycast_distance;
          seed_y2 = base_y + perp_dir2_y * max_raycast_distance;
          ray2.end_point.x = seed_x2;
          ray2.end_point.y = seed_y2;
          ray2.end_point.z = 0.0;
          ray2.hit = false;
        }
        new_ray_casts.push_back(ray2);
        
        // Add seed (regardless of collision) - only if outside polygon
        {
          // Don't add at all if inside polygon (check before seed generation)
          bool skip_seed = false;
          if (use_polygon_filter) {
            if (isPointInPolygon(seed_x2, seed_y2, polygon_points)) {
              skip_seed = true;  // Skip if inside polygon
            }
          }
          
          if (!skip_seed) {
            VirtualSeed hit_seed2;
            hit_seed2.position.x = seed_x2;
            hit_seed2.position.y = seed_y2;
            hit_seed2.position.z = 0.0;
            hit_seed2.tree_row_index = static_cast<int>(row_idx);
            hit_seed2.is_converted = false;
            
            // Add after duplicate check
            bool already_exists = false;
            for (const auto& existing_seed : new_virtual_seeds) {
              double dist = std::sqrt(
                std::pow(existing_seed.position.x - hit_seed2.position.x, 2) +
                std::pow(existing_seed.position.y - hit_seed2.position.y, 2)
              );
              if (dist < 0.5) {
                already_exists = true;
                break;
              }
            }
            
            if (!already_exists) {
              // Also check for duplicates with real seeds
              bool is_real_seed = false;
              for (const auto& real_seed : real_seeds_) {
                double dist = std::sqrt(
                  std::pow(real_seed.x - hit_seed2.position.x, 2) +
                  std::pow(real_seed.y - hit_seed2.position.y, 2)
                );
                if (dist < 0.5) {
                  is_real_seed = true;
                  break;
                }
              }
              
              if (!is_real_seed) {
                new_virtual_seeds.push_back(hit_seed2);
              }
            }
          }
        }
      }
    }
    
    // Completely replace virtual seeds (to handle tree row changes)
    virtual_seeds_ = new_virtual_seeds;
    ray_casts_ = new_ray_casts;  // Also replace ray information
  }
  
  // Function to convert virtual seeds to real seeds
  // When robot passes near virtual seeds, use confirmed tree positions near that location as real seeds
  void convertVirtualSeedsToReal() {
    if (!robot_position_received_) {
      return;
    }
    
    // Get confirmed tree positions (use empty array since tree_tracking_node was removed)
    geometry_msgs::msg::PoseArray confirmed_trees;
    
    // Don't remove early return even if no confirmed trees, since virtual seeds robot passed through should be removed
    
    // Keep virtual seeds without removing them
    // Add only confirmed trees at positions robot passed through as real seeds
    
    for (const auto& seed : virtual_seeds_) {
      // Calculate distance between robot and virtual seed
      double dx = seed.position.x - current_robot_position_.x;
      double dy = seed.position.y - current_robot_position_.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      
      if (distance <= conversion_radius_) {
        // If within 4m radius, consider robot has passed through and add confirmed tree positions as real seeds
        // Find confirmed tree position closest to virtual seed position
        double min_dist = std::numeric_limits<double>::max();
        geometry_msgs::msg::Point nearest_tree;
        bool found_nearby_tree = false;
        
        for (const auto& tree_pose : confirmed_trees.poses) {
          double tree_dx = tree_pose.position.x - seed.position.x;
          double tree_dy = tree_pose.position.y - seed.position.y;
          double tree_dist = std::sqrt(tree_dx * tree_dx + tree_dy * tree_dy);
          
          // Consider only confirmed trees within 2m of virtual seed position
          if (tree_dist < 2.0 && tree_dist < min_dist) {
            min_dist = tree_dist;
            nearest_tree.x = tree_pose.position.x;
            nearest_tree.y = tree_pose.position.y;
            nearest_tree.z = 0.0;
            found_nearby_tree = true;
          }
        }
        
        // Add as real seed if nearby confirmed tree exists
        if (found_nearby_tree) {
          // Add to real seed list (check for duplicates)
          bool already_exists = false;
          for (const auto& real_seed : real_seeds_) {
            double dist = std::sqrt(
              std::pow(real_seed.x - nearest_tree.x, 2) +
              std::pow(real_seed.y - nearest_tree.y, 2)
            );
            if (dist < 0.5) {  // Consider duplicate if within 0.5m
              already_exists = true;
              break;
            }
          }
          
          if (!already_exists) {
            real_seeds_.push_back(nearest_tree);
          }
        }
      }
    }
    
    // Keep virtual seeds without removing them (no change)
    
    // Update real seeds at positions robot passed through to confirmed tree positions
    // (update real seeds when confirmed tree positions are updated)
    if (!confirmed_trees.poses.empty()) {
      // Check all virtual seeds to find confirmed trees at positions robot passed through
      for (const auto& seed : virtual_seeds_) {
        // Calculate distance between robot and virtual seed
        double dx = seed.position.x - current_robot_position_.x;
        double dy = seed.position.y - current_robot_position_.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance <= conversion_radius_) {
          // Find closest confirmed tree at position robot passed through
          double min_dist = std::numeric_limits<double>::max();
          geometry_msgs::msg::Point nearest_tree;
          bool found_nearby_tree = false;
          
          for (const auto& tree_pose : confirmed_trees.poses) {
            double tree_dx = tree_pose.position.x - seed.position.x;
            double tree_dy = tree_pose.position.y - seed.position.y;
            double tree_dist = std::sqrt(tree_dx * tree_dx + tree_dy * tree_dy);
            
            // Consider only confirmed trees within 2m of virtual seed position
            if (tree_dist < 2.0 && tree_dist < min_dist) {
              min_dist = tree_dist;
              nearest_tree.x = tree_pose.position.x;
              nearest_tree.y = tree_pose.position.y;
              nearest_tree.z = 0.0;
              found_nearby_tree = true;
            }
          }
          
          // Add as real seed if nearby confirmed tree exists/update
          if (found_nearby_tree) {
            // Check if already exists in real seed list
            bool already_exists = false;
            for (const auto& real_seed : real_seeds_) {
              double dist = std::sqrt(
                std::pow(real_seed.x - nearest_tree.x, 2) +
                std::pow(real_seed.y - nearest_tree.y, 2)
              );
              if (dist < 0.5) {  // Already exists if within 0.5m
                already_exists = true;
                break;
              }
            }
            
            if (!already_exists) {
              real_seeds_.push_back(nearest_tree);
            }
          }
        }
      }
    }
  }
  
  // Function to visualize virtual and real seeds
  void publishVirtualAndRealSeedsVisualization(const rclcpp::Time& stamp, const std::string& frame_id) {
    std::lock_guard<std::mutex> lock(seeds_mutex_);
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete existing markers for each namespace
    std::vector<std::string> namespaces = {
      "virtual_seeds",
      "real_seeds", 
      "ray_seeds_from_endpoints",
      "tree_row_endpoint_seeds",
      "ray_casts"
    };
    
    for (const auto& ns : namespaces) {
      visualization_msgs::msg::Marker delete_all;
      delete_all.header.frame_id = frame_id.empty() ? "map" : frame_id;
      delete_all.header.stamp = stamp;
      delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_all.ns = ns;
      marker_array.markers.push_back(delete_all);
    }
    
    // Visualize virtual seeds (yellow spheres) - display all virtual seeds
    int virtual_seed_id = 0;
    for (const auto& seed : virtual_seeds_) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id;
      marker.ns = "virtual_seeds";
      marker.id = virtual_seed_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = seed.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color.r = 1.0f;  // Yellow
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent display
      marker_array.markers.push_back(marker);
    }
    
    // Visualize real seeds (green spheres) - seeds taken from confirmed tree positions
    int real_seed_id = 0;
    for (const auto& seed : real_seeds_) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id;
      marker.ns = "real_seeds";
      marker.id = real_seed_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = seed;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      marker.color.r = 0.0f;  // Green
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent display
      marker_array.markers.push_back(marker);
    }
    
    // Visualize ray points generated from tree row endpoints (green spheres, slightly larger)
    int ray_seed_id = 0;
    for (const auto& ray_seed : ray_seeds_from_endpoints_) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id;
      marker.ns = "ray_seeds_from_endpoints";
      marker.id = ray_seed_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = ray_seed;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.5;  // Slightly larger than real seeds
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      marker.color.r = 0.0f;  // Green
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent display
      marker_array.markers.push_back(marker);
    }
    
    // Visualize start and end points of tree rows inside polygon (red spheres)
    int endpoint_seed_id = 0;
    for (const auto& endpoint_seed : tree_row_endpoint_seeds_) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id;
      marker.ns = "tree_row_endpoint_seeds";
      marker.id = endpoint_seed_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = endpoint_seed;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      marker.color.r = 1.0f;  // Red
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent display
      marker_array.markers.push_back(marker);
    }
    
    // Visualize ray casting
    int ray_cast_id = 0;
    for (const auto& ray : ray_casts_) {
      visualization_msgs::msg::Marker ray_marker;
      ray_marker.header.stamp = stamp;
      ray_marker.header.frame_id = frame_id;
      ray_marker.ns = "ray_casts";
      ray_marker.id = ray_cast_id++;
      ray_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      ray_marker.action = visualization_msgs::msg::Marker::ADD;
      ray_marker.scale.x = 0.05;  // Line thickness
      
      // Add start and end points
      ray_marker.points.push_back(ray.start_point);
      ray_marker.points.push_back(ray.end_point);
      
      // Change color based on collision status
      if (ray.hit) {
        ray_marker.color.r = 1.0f;  // Red (collision)
        ray_marker.color.g = 0.0f;
        ray_marker.color.b = 0.0f;
        ray_marker.color.a = 0.8f;
      } else {
        ray_marker.color.r = 0.5f;  // Gray (no collision)
        ray_marker.color.g = 0.5f;
        ray_marker.color.b = 0.5f;
        ray_marker.color.a = 0.5f;
      }
      
      ray_marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent display
      marker_array.markers.push_back(ray_marker);
    }
    
    // Publish to both publishers (for compatibility)
    publisher_virtual_real_seeds_viz_->publish(marker_array);
    publisher_voronoi_seeds_viz->publish(marker_array);
  }

  void publishExplorationTreeRowsInfoFromClusters(const std::vector<TreeRowFromCluster>& exploration_tree_rows,
                                     const rclcpp::Time& stamp, const std::string& frame_id) {
    geometry_msgs::msg::PoseArray tree_rows_info;
    tree_rows_info.header.stamp = stamp;
    tree_rows_info.header.frame_id = frame_id;
    
    // Sort tree rows in area by center's y-value (ascending), then by x-value if equal
    std::vector<TreeRowFromCluster> sorted_rows = exploration_tree_rows;
    std::sort(sorted_rows.begin(), sorted_rows.end(),
             [](const TreeRowFromCluster& a, const TreeRowFromCluster& b) {
               if (std::abs(a.center.y() - b.center.y()) < 1e-6) {
                 return a.center.x() < b.center.x();
               }
               return a.center.y() < b.center.y();
             });
    
    // Add start and end points of each tree row
    for (const auto& row : sorted_rows) {
      // Start point
      geometry_msgs::msg::Pose start_pose;
      start_pose.position.x = row.start_point.x();
      start_pose.position.y = row.start_point.y();
      start_pose.position.z = 0.0;
      start_pose.orientation.w = 1.0;
      tree_rows_info.poses.push_back(start_pose);
      
      // End point
      geometry_msgs::msg::Pose end_pose;
      end_pose.position.x = row.end_point.x();
      end_pose.position.y = row.end_point.y();
      end_pose.position.z = 0.0;
      end_pose.orientation.w = 1.0;
      tree_rows_info.poses.push_back(end_pose);
    }
    
    publisher_exploration_tree_rows_info->publish(tree_rows_info);
  }

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_occupancy_grid;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_skeletonized_grid;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_cluster_markers;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_cluster_info;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_exploration_area_viz;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_filtered_trees;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_filtered_trees_viz;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_tree_rows_all;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_tree_rows_exploration;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_voronoi_seeds;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_voronoi_seeds_viz;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_exploration_tree_rows_info;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_virtual_real_seeds_viz_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_global_map;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscriber_exploration_area;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_robot_position_;

  // Parameters
  float clipping_minz;
  float clipping_maxz;
  float clipping_minx;
  float clipping_maxx;
  float clipping_miny;
  float clipping_maxy;
  float grid_resolution;
  float inflation_radius;
  float waypoint_offset_distance;
  float waypoint_offset_distance_x;
  float waypoint_offset_distance_y;
  bool publish_cluster_axes_ = true;
  double cluster_axis_scale_ = 1.0;
  double cluster_axis_width_ = 0.08;
  double cluster_min_length_m_ = 2.0;
  
  // Hardcoded waypoint coordinates (from /gvd/waypoints topic)
  std::vector<std::pair<float, float>> waypoints;

  // Exploration area state
  bool area_polygon_received = false;
  std::vector<geometry_msgs::msg::Point32> area_polygon_points;
  float area_minx = 0.0f;
  float area_maxx = 0.0f;
  float area_miny = 0.0f;
  float area_maxy = 0.0f;

  // Last received cloud for reprocessing when area updates arrive
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud;
  
  std::mutex area_polygon_mutex_;
  
  // Hardcoded polygon (taken from aos_gvd_node.cpp)
  std::vector<std::pair<double, double>> hardcoded_polygon_points_;
  std::vector<std::pair<double, double>> received_polygon_points_;  // Polygon received from topic (storage only)
  bool use_hardcoded_polygon_ = true;
  double hardcoded_minx_ = 0.0;
  double hardcoded_maxx_ = 0.0;
  double hardcoded_miny_ = 0.0;
  double hardcoded_maxy_ = 0.0;
  
  // Store last filtered trees (used when calculating tree rows)
  
  // Store last tree row information (used when adding lines to grid)
  std::vector<TreeRowFromCluster> last_tree_rows_;
  std::mutex last_tree_rows_mutex_;
  
  // Store last skeletonized grid (for ray casting)
  nav_msgs::msg::OccupancyGrid last_skeletonized_grid_;
  std::mutex skeletonized_grid_mutex_;
  
  // Virtual seed and real seed management
  std::vector<VirtualSeed> virtual_seeds_;  // Virtual seed list
  std::vector<geometry_msgs::msg::Point> real_seeds_;  // Real seed list (tree positions)
  std::vector<geometry_msgs::msg::Point> ray_seeds_from_endpoints_;  // Ray point seeds generated from tree row endpoints
  std::vector<geometry_msgs::msg::Point> tree_row_endpoint_seeds_;  // Start and end point seeds of tree rows inside polygon
  std::vector<geometry_msgs::msg::Point> converted_virtual_seed_positions_;  // Virtual seed positions robot passed through (to prevent regeneration)
  std::vector<RayCast> ray_casts_;  // Ray casting information list
  std::mutex seeds_mutex_;  // Protect access to virtual and real seeds
  geometry_msgs::msg::Point current_robot_position_;  // Current robot position
  bool robot_position_received_ = false;
  const double virtual_seed_interval_ = 1.0;  // Virtual seed interval (1m)
  const double conversion_radius_ = 4.0;  // Radius for converting virtual seeds to real seeds (4m)
  
  // Timer
  rclcpp::TimerBase::SharedPtr exploration_area_viz_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AosSeedGenNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
