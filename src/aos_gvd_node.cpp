#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <limits>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>

#include "aos/msg/gvd_graph.hpp"
#include "aos/voronoi_diagram.hpp"

class AosGvdNode : public rclcpp::Node {
public:
  AosGvdNode() : rclcpp::Node("aos_gvd_node") {
    // Parameters
    this->declare_parameter("max_graph_publish_rate", 10.0);
    this->declare_parameter("voronoi_seeds_topic", std::string("/voronoi_seeds"));
    this->declare_parameter("exploration_tree_rows_info_topic", std::string("/exploration_tree_rows_info"));
    this->declare_parameter("tree_rows_all_topic", std::string("/tree_rows_all"));
    this->declare_parameter("skeletonized_occupancy_grid_topic", std::string("/skeletonized_occupancy_grid"));
    this->declare_parameter("occupancy_grid_topic", std::string("/occupancy_grid"));
    this->declare_parameter("robot_position_topic", std::string("/Local/utm"));
    
    this->get_parameter("max_graph_publish_rate", max_graph_publish_rate_);
    std::string voronoi_seeds_topic, exploration_tree_rows_info_topic, tree_rows_all_topic;
    std::string skeletonized_occupancy_grid_topic, occupancy_grid_topic, robot_position_topic;
    this->get_parameter("voronoi_seeds_topic", voronoi_seeds_topic);
    this->get_parameter("exploration_tree_rows_info_topic", exploration_tree_rows_info_topic);
    this->get_parameter("tree_rows_all_topic", tree_rows_all_topic);
    this->get_parameter("skeletonized_occupancy_grid_topic", skeletonized_occupancy_grid_topic);
    this->get_parameter("occupancy_grid_topic", occupancy_grid_topic);
    this->get_parameter("robot_position_topic", robot_position_topic);

    rclcpp::QoS reliable_qos(10);
    reliable_qos.reliable();

    // Subscribers
    sub_voronoi_seeds_ = create_subscription<geometry_msgs::msg::PoseArray>(
        voronoi_seeds_topic, reliable_qos,
        std::bind(&AosGvdNode::voronoiSeedsCallback, this, std::placeholders::_1));

    sub_exploration_tree_rows_info_ = create_subscription<geometry_msgs::msg::PoseArray>(
        exploration_tree_rows_info_topic, reliable_qos,
        std::bind(&AosGvdNode::explorationTreeRowsInfoCallback, this, std::placeholders::_1));

    sub_all_tree_rows_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        tree_rows_all_topic, reliable_qos,
        std::bind(&AosGvdNode::allTreeRowsCallback, this, std::placeholders::_1));

    sub_skeletonized_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        skeletonized_occupancy_grid_topic, reliable_qos,
        std::bind(&AosGvdNode::skeletonizedGridCallback, this, std::placeholders::_1));

    sub_occupancy_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_grid_topic, reliable_qos,
        std::bind(&AosGvdNode::occupancyGridCallback, this, std::placeholders::_1));

    sub_robot_position_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        robot_position_topic, 10,
        std::bind(&AosGvdNode::robotPositionCallback, this, std::placeholders::_1));

    sub_docking_state_ = create_subscription<std_msgs::msg::Bool>(
        "/aos/docking_state", reliable_qos,
        std::bind(&AosGvdNode::dockingStateCallback, this, std::placeholders::_1));

    // Publishers
    pub_graph_ = create_publisher<aos::msg::GvdGraph>("/gvd/graph", reliable_qos);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/gvd/markers", reliable_qos);

    last_graph_publish_time_ = std::chrono::steady_clock::now();
  }

private:
  void voronoiSeedsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Collect raw seeds
    std::vector<Eigen::Vector2d> raw_seeds;
    for (const auto& pose : msg->poses) {
      raw_seeds.push_back(Eigen::Vector2d(pose.position.x, pose.position.y));
    }
    
    // Merge seeds within 50cm into one
    const double merge_distance = 0.5;  // 50cm
    voronoi_seeds_.clear();
    std::vector<bool> used(raw_seeds.size(), false);
    
    for (size_t i = 0; i < raw_seeds.size(); ++i) {
      if (used[i]) continue;
      
      // Collect cluster around current seed
      std::vector<size_t> cluster_indices;
      cluster_indices.push_back(i);
      used[i] = true;
      
      // Find other seeds within 50cm
      for (size_t j = i + 1; j < raw_seeds.size(); ++j) {
        if (used[j]) continue;
        
        double dist = (raw_seeds[i] - raw_seeds[j]).norm();
        if (dist <= merge_distance) {
          cluster_indices.push_back(j);
          used[j] = true;
        }
      }
      
      // Calculate cluster center point
      Eigen::Vector2d merged_point(0.0, 0.0);
      for (size_t idx : cluster_indices) {
        merged_point += raw_seeds[idx];
      }
      merged_point /= static_cast<double>(cluster_indices.size());
      
      voronoi_seeds_.push_back(merged_point);
    }
    
    processGraph();
  }

  void explorationTreeRowsInfoCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    exploration_tree_rows_.clear();
    // Read start and end point pairs from PoseArray
    // Smaller x-value is TOP (ep1), larger x-value is Bottom (ep2)
    for (size_t i = 0; i < msg->poses.size(); i += 2) {
      if (i + 1 < msg->poses.size()) {
        Eigen::Vector2d start(msg->poses[i].position.x, msg->poses[i].position.y);
        Eigen::Vector2d end(msg->poses[i + 1].position.x, msg->poses[i + 1].position.y);
        
        // Sort by x-value: smaller is TOP (ep1), larger is Bottom (ep2)
        if (start.x() > end.x()) {
          std::swap(start, end);
        }
        
        exploration_tree_rows_.push_back({start, end});  // {TOP, Bottom}
      }
    }
    // Ray point generation is performed in aos_seed_gen_node.cpp
    processGraph();
  }

  void allTreeRowsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    all_tree_rows_.clear();
    // Extract tree row information by finding LINE_STRIP type markers from MarkerArray
    for (const auto& marker : msg->markers) {
      // Ignore DELETEALL markers
      if (marker.action == visualization_msgs::msg::Marker::DELETEALL) {
        continue;
      }
      if (marker.type == visualization_msgs::msg::Marker::LINE_STRIP && 
          marker.points.size() >= 2 &&
          marker.action == visualization_msgs::msg::Marker::ADD) {
        // Extract start and end points
        Eigen::Vector2d start(marker.points[0].x, marker.points[0].y);
        Eigen::Vector2d end(marker.points.back().x, marker.points.back().y);
        all_tree_rows_.push_back({start, end});
      }
    }
    processGraph();
  }

  void skeletonizedGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    skeletonized_grid_ = *msg;
    processGraph();
  }

  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    occupancy_grid_ = *msg;
    processGraph();
  }

  void robotPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data.size() >= 2) {
      robot_position_.x() = msg->data[0];
      robot_position_.y() = msg->data[1];
      robot_position_received_ = true;
    }
  }

  void dockingStateCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    waiting_for_docking_completion_ = msg->data;
  }

  // Check distance between labeled nodes (TL/TR/BL/BR) and robot
  bool isRobotNearLabeledNode() {
    if (!robot_position_received_ || voronoi_boundary_points_.empty()) {
      return false;
    }

    const double threshold_distance = 1.0;  // 1m
    const double label_match_tolerance = 0.1;

    // Find labeled nodes
    for (size_t i = 0; i < voronoi_boundary_points_.size(); ++i) {
      const auto& point = voronoi_boundary_points_[i];
      int label_mask = 0;
      
      // Check all clusters to verify labels
      for (size_t cluster_id = 0; cluster_id < cluster_voronoi_boundary_points_.size(); ++cluster_id) {
        const auto& cluster_voro = cluster_voronoi_boundary_points_[cluster_id];
        
        if (cluster_voro.ep1_valid[0]) {
          double dist_tl = (point - cluster_voro.ep1_boundary_points[0]).norm();
          if (dist_tl < label_match_tolerance) {
            label_mask |= (1 << 0);  // TL
          }
        }
        if (cluster_voro.ep1_valid[1]) {
          double dist_tr = (point - cluster_voro.ep1_boundary_points[1]).norm();
          if (dist_tr < label_match_tolerance) {
            label_mask |= (1 << 1);  // TR
          }
        }
        if (cluster_voro.ep2_valid[0]) {
          double dist_bl = (point - cluster_voro.ep2_boundary_points[0]).norm();
          if (dist_bl < label_match_tolerance) {
            label_mask |= (1 << 2);  // BL
          }
        }
        if (cluster_voro.ep2_valid[1]) {
          double dist_br = (point - cluster_voro.ep2_boundary_points[1]).norm();
          if (dist_br < label_match_tolerance) {
            label_mask |= (1 << 3);  // BR
          }
        }
      }
      
      // If node has label, check distance to robot
      if (label_mask != 0) {
        double dist = (point - robot_position_).norm();
        if (dist <= threshold_distance) {
          return true;
        }
      }
    }
    
    return false;
  }

  void processGraph() {
    // Check if all required data is ready
    if (voronoi_seeds_.empty() || !skeletonized_grid_) {
      return;
    }

    try {
      // Use seeds (ray points are generated in aos_seed_gen_node.cpp and included)
      std::vector<Eigen::Vector2d> all_seeds;
      
      // Filter only valid seeds (remove NaN/Inf)
      for (const auto& seed : voronoi_seeds_) {
        if (std::isfinite(seed.x()) && std::isfinite(seed.y())) {
          all_seeds.push_back(seed);
        }
      }
      
      // Exit if no valid seeds
      if (all_seeds.empty()) {
        return;
      }
      
      // Compute Voronoi diagram
      double minx = skeletonized_grid_->info.origin.position.x;
      double maxx = minx + skeletonized_grid_->info.width * skeletonized_grid_->info.resolution;
      double miny = skeletonized_grid_->info.origin.position.y;
      double maxy = miny + skeletonized_grid_->info.height * skeletonized_grid_->info.resolution;

      // Validate boundary values
      if (!std::isfinite(minx) || !std::isfinite(maxx) || !std::isfinite(miny) || !std::isfinite(maxy)) {
        RCLCPP_WARN(this->get_logger(), "Invalid grid bounds, skipping Voronoi computation");
        return;
      }

      voronoi_diagram_.compute(all_seeds, minx, maxx, miny, maxy);
      voronoi_boundary_points_ = voronoi_diagram_.extractBoundaryPoints();

      // Build graph
      buildGraphFromBoundaryPoints();
      
      // Remove nodes and edges outside grid map (nodes near occupied cells are not removed - use Voronoi results as-is)
      filterNodesAndEdgesOutsideGrid();
      
      // Find Voronoi boundary points around cluster endpoints (for TL/TR/BL/BR labeling)
      findClusterEndpointVoronoiBoundaryPoints();

      // Publish graph
      std_msgs::msg::Header header;
      header.frame_id = "map";
      header.stamp = this->get_clock()->now();

      auto now = std::chrono::steady_clock::now();
      double min_interval = 1.0 / max_graph_publish_rate_;
      auto elapsed = std::chrono::duration<double>(now - last_graph_publish_time_).count();

      if (elapsed >= min_interval) {
        publishGraph(header);
        publishMarkers(header);
        last_graph_publish_time_ = now;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error computing Voronoi diagram: %s", e.what());
    }
  }

  bool edgePassesThroughOccupiedPixels(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
    // Use skeletonized_grid (for occupied cell checking)
    if (!skeletonized_grid_) {
      return false;
    }

    const double resolution = skeletonized_grid_->info.resolution;
    const int width = static_cast<int>(skeletonized_grid_->info.width);
    const int height = static_cast<int>(skeletonized_grid_->info.height);

    double edge_length = (end - start).norm();
    if (edge_length < 1e-6) {
      return false;
    }

    const double sample_step = resolution * 0.5;
    int num_samples = static_cast<int>(edge_length / sample_step) + 1;

    Eigen::Vector2d dir = (end - start).normalized();

    for (int i = 0; i <= num_samples; ++i) {
      double t = (i == num_samples) ? 1.0 : (static_cast<double>(i) / static_cast<double>(num_samples));
      Eigen::Vector2d sample_point = start + t * dir * edge_length;

      int mx = static_cast<int>((sample_point.x() - skeletonized_grid_->info.origin.position.x) / resolution);
      int my = static_cast<int>((sample_point.y() - skeletonized_grid_->info.origin.position.y) / resolution);

      if (mx >= 0 && mx < width && my >= 0 && my < height) {
        int index = mx + my * width;
        if (index >= 0 && index < static_cast<int>(skeletonized_grid_->data.size())) {
          // Return true if passing through occupied cell (value == 100)
          if (skeletonized_grid_->data[index] == 100) {
            return true;
          }
        }
      }
    }

    return false;
  }

  bool edgeCrossesTreeRow(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
    // Check if edge crosses any tree row
    for (const auto& row : all_tree_rows_) {
      const Eigen::Vector2d& row_start = row.first;
      const Eigen::Vector2d& row_end = row.second;
      
      // Line segment intersection check
      if (doSegmentsIntersect(start, end, row_start, row_end)) {
        return true;
      }
    }
    return false;
  }

  bool doSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                          const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) {
    // Line segment intersection algorithm (CCW direction check)
    auto ccw = [](const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) -> double {
      return (C.y() - A.y()) * (B.x() - A.x()) - (B.y() - A.y()) * (C.x() - A.x());
    };

    double o1 = ccw(p1, p2, q1);
    double o2 = ccw(p1, p2, q2);
    double o3 = ccw(q1, q2, p1);
    double o4 = ccw(q1, q2, p2);

    // General intersection case
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) &&
        ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) {
      return true;
    }

    // Collinear case (one segment's endpoint is on the other segment)
    if (std::abs(o1) < 1e-9 && isPointOnSegment(p1, p2, q1)) return true;
    if (std::abs(o2) < 1e-9 && isPointOnSegment(p1, p2, q2)) return true;
    if (std::abs(o3) < 1e-9 && isPointOnSegment(q1, q2, p1)) return true;
    if (std::abs(o4) < 1e-9 && isPointOnSegment(q1, q2, p2)) return true;

    return false;
  }

  bool isPointOnSegment(const Eigen::Vector2d& seg_start, const Eigen::Vector2d& seg_end, 
                        const Eigen::Vector2d& point) {
    Eigen::Vector2d seg_vec = seg_end - seg_start;
    Eigen::Vector2d point_vec = point - seg_start;
    
    // Collinearity check
    double cross = seg_vec.x() * point_vec.y() - seg_vec.y() * point_vec.x();
    if (std::abs(cross) > 1e-9) {
      return false;
    }
    
    // Check if point is inside segment
    double dot = seg_vec.dot(point_vec);
    double seg_len_sq = seg_vec.squaredNorm();
    
    return dot >= 0 && dot <= seg_len_sq;
  }

  void filterNodesAndEdgesOutsideGrid() {
    if (!skeletonized_grid_) {
      return;
    }

    const double minx = skeletonized_grid_->info.origin.position.x;
    const double maxx = minx + skeletonized_grid_->info.width * skeletonized_grid_->info.resolution;
    const double miny = skeletonized_grid_->info.origin.position.y;
    const double maxy = miny + skeletonized_grid_->info.height * skeletonized_grid_->info.resolution;

    // Keep only nodes within grid bounds
    std::vector<Eigen::Vector2d> filtered_boundary_points;
    std::vector<int> old_to_new_index_map(voronoi_boundary_points_.size(), -1);
    int new_index = 0;

    for (size_t i = 0; i < voronoi_boundary_points_.size(); ++i) {
      const auto& point = voronoi_boundary_points_[i];
      if (point.x() >= minx && point.x() <= maxx && 
          point.y() >= miny && point.y() <= maxy) {
        filtered_boundary_points.push_back(point);
        old_to_new_index_map[i] = new_index;
        new_index++;
      }
    }

    // Remap edges (connect only nodes within grid)
    std::vector<EdgeRecord> filtered_edge_records;
    for (const auto& edge : edge_records_) {
      int new_from = old_to_new_index_map[edge.from];
      int new_to = old_to_new_index_map[edge.to];
      
      if (new_from >= 0 && new_to >= 0 && new_from != new_to) {
        // Check if both endpoints of edge are within grid
        const auto& from_point = filtered_boundary_points[new_from];
        const auto& to_point = filtered_boundary_points[new_to];
        
        if (from_point.x() >= minx && from_point.x() <= maxx &&
            from_point.y() >= miny && from_point.y() <= maxy &&
            to_point.x() >= minx && to_point.x() <= maxx &&
            to_point.y() >= miny && to_point.y() <= maxy) {
          int a = new_from;
          int b = new_to;
          if (a > b) {
            std::swap(a, b);
          }
          double length_m = (to_point - from_point).norm();
          filtered_edge_records.push_back(EdgeRecord{a, b, length_m, edge.min_clearance_m});
        }
      }
    }

    // Reconstruct edges_
    edges_.clear();
    edges_.resize(filtered_boundary_points.size());
    for (const auto& edge : filtered_edge_records) {
      edges_[edge.from].push_back(edge.to);
      edges_[edge.to].push_back(edge.from);
    }

    // Update data
    voronoi_boundary_points_ = filtered_boundary_points;
    edge_records_ = filtered_edge_records;
    voro_nodes_ = filtered_boundary_points;
  }

  void findClusterEndpointVoronoiBoundaryPoints() {
    cluster_voronoi_boundary_points_.clear();
    cluster_voronoi_boundary_points_.reserve(exploration_tree_rows_.size());
    
    if (voronoi_boundary_points_.empty()) {
      return;
    }
    
    const std::vector<double> angle_tolerances = {90.0, 60.0, 45.0, 30.0};
    const double min_distance = 0.5;
    const double max_distance = 5.0;
    
    for (const auto& tree_row : exploration_tree_rows_) {
      ClusterVoronoiBoundaryPoints voro_points;
      // Smaller x-value is TOP (ep1), larger x-value is Bottom (ep2)
      voro_points.endpoint1 = tree_row.first;   // ep1 (TOP, smaller x-value)
      voro_points.endpoint2 = tree_row.second;  // ep2 (Bottom, larger x-value)
      
      voro_points.ep1_valid.fill(false);
      voro_points.ep2_valid.fill(false);
      
      // Find TL from TOP (ep1) in -90° direction
      findVoronoiBoundaryPointNearEndpoint(
        tree_row.first, 
        tree_row.second,
        -90.0,
        angle_tolerances,
        min_distance,
        max_distance,
        voro_points.ep1_boundary_points[0],
        voro_points.ep1_valid[0]
      );
      
      // Find TR from TOP (ep1) in +90° direction
      findVoronoiBoundaryPointNearEndpoint(
        tree_row.first,
        tree_row.second,
        90.0,
        angle_tolerances,
        min_distance,
        max_distance,
        voro_points.ep1_boundary_points[1],
        voro_points.ep1_valid[1]
      );
      
      // Find BL from Bottom (ep2) in -90° direction
      findVoronoiBoundaryPointNearEndpoint(
        tree_row.second,
        tree_row.first,
        -90.0,
        angle_tolerances,
        min_distance,
        max_distance,
        voro_points.ep2_boundary_points[0],
        voro_points.ep2_valid[0]
      );
      
      // Find BR from Bottom (ep2) in +90° direction
      findVoronoiBoundaryPointNearEndpoint(
        tree_row.second,
        tree_row.first,
        90.0,
        angle_tolerances,
        min_distance,
        max_distance,
        voro_points.ep2_boundary_points[1],
        voro_points.ep2_valid[1]
      );
      
      cluster_voronoi_boundary_points_.push_back(voro_points);
    }
  }
  
  Eigen::Vector2d castRay(const Eigen::Vector2d& start_point,
                          const Eigen::Vector2d& other_endpoint,
                          double angle_offset_deg,
                          double min_distance = 1.0,
                          double max_distance = 6.0) {
    Eigen::Vector2d endpoint_to_other = other_endpoint - start_point;
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
      if (!skeletonized_grid_) {
        return true;
      }
      const double minx = skeletonized_grid_->info.origin.position.x;
      const double maxx = minx + skeletonized_grid_->info.width * skeletonized_grid_->info.resolution;
      const double miny = skeletonized_grid_->info.origin.position.y;
      const double maxy = miny + skeletonized_grid_->info.height * skeletonized_grid_->info.resolution;
      return point.x() >= minx && point.x() <= maxx && 
             point.y() >= miny && point.y() <= maxy;
    };
    
    // Function to check occupied cells in skeleton grid map
    auto isOccupiedInGrid = [&](const Eigen::Vector2d& point) -> bool {
      // Always use skeletonized_grid
      if (!skeletonized_grid_) {
        return false;
      }

      const double resolution = skeletonized_grid_->info.resolution;
      const int width = static_cast<int>(skeletonized_grid_->info.width);
      const int height = static_cast<int>(skeletonized_grid_->info.height);

      int mx = static_cast<int>((point.x() - skeletonized_grid_->info.origin.position.x) / resolution);
      int my = static_cast<int>((point.y() - skeletonized_grid_->info.origin.position.y) / resolution);

      if (mx >= 0 && mx < width && my >= 0 && my < height) {
        int index = mx + my * width;
        if (index >= 0 && index < static_cast<int>(skeletonized_grid_->data.size())) {
          // Return true if passing through occupied cell (100)
          return skeletonized_grid_->data[index] == 100;
        }
      }
      return false;
    };
    
    // Set step_size proportional to grid resolution to avoid missing occupied cells
    // Setting to resolution * 0.5 or less allows checking all cells
    double step_size = 0.1;
    if (skeletonized_grid_) {
      step_size = skeletonized_grid_->info.resolution * 0.5;
      // Ensure minimum value (too small causes performance degradation)
      if (step_size < 0.01) {
        step_size = 0.01;
      }
    }
    double current_dist = min_distance;
    
    // Continue until hitting grid boundary or occupied cell without max_distance limit
    // Set sufficiently large maximum distance (3x grid diagonal if grid exists, 10000m otherwise)
    double absolute_max_distance = max_distance;
    if (skeletonized_grid_) {
      double grid_width = skeletonized_grid_->info.width * skeletonized_grid_->info.resolution;
      double grid_height = skeletonized_grid_->info.height * skeletonized_grid_->info.resolution;
      absolute_max_distance = std::sqrt(grid_width * grid_width + grid_height * grid_height) * 3.0;
    } else {
      absolute_max_distance = 10000.0;
    }
    
    while (current_dist <= absolute_max_distance) {
      Eigen::Vector2d check_point = start_point + ray_dir * current_dist;
      
      if (!isInsideBounds(check_point)) {
        // Clip to grid boundary
        if (skeletonized_grid_) {
          const double minx = skeletonized_grid_->info.origin.position.x;
          const double maxx = minx + skeletonized_grid_->info.width * skeletonized_grid_->info.resolution;
          const double miny = skeletonized_grid_->info.origin.position.y;
          const double maxy = miny + skeletonized_grid_->info.height * skeletonized_grid_->info.resolution;
          Eigen::Vector2d boundary_point = check_point;
          boundary_point.x() = std::max(minx, std::min(maxx, boundary_point.x()));
          boundary_point.y() = std::max(miny, std::min(maxy, boundary_point.y()));
          return boundary_point;
        }
        return check_point;
      }
      
      // Stop if passing through occupied cell in skeleton grid map
      if (isOccupiedInGrid(check_point)) {
        return check_point;
      }
      
      current_dist += step_size;
    }
    
    // Case when absolute maximum distance is reached (theoretically should not occur)
    Eigen::Vector2d final_point = start_point + ray_dir * absolute_max_distance;
    
    if (!isInsideBounds(final_point)) {
      if (skeletonized_grid_) {
        const double minx = skeletonized_grid_->info.origin.position.x;
        const double maxx = minx + skeletonized_grid_->info.width * skeletonized_grid_->info.resolution;
        const double miny = skeletonized_grid_->info.origin.position.y;
        const double maxy = miny + skeletonized_grid_->info.height * skeletonized_grid_->info.resolution;
        final_point.x() = std::max(minx, std::min(maxx, final_point.x()));
        final_point.y() = std::max(miny, std::min(maxy, final_point.y()));
      }
    }
    
    return final_point;
  }
  
  void findVoronoiBoundaryPointNearEndpoint(
    const Eigen::Vector2d& endpoint,
    const Eigen::Vector2d& other_endpoint,
    double target_angle_deg,
    const std::vector<double>& /* angle_tolerances */,
    double min_distance,
    double max_distance,
    Eigen::Vector2d& point,
    bool& valid
  ) {
    valid = false;
    
    Eigen::Vector2d main_dir = other_endpoint - endpoint;
    double main_length = main_dir.norm();
    if (main_length < 1e-6) {
      main_dir = Eigen::Vector2d(1.0, 0.0);
    } else {
      main_dir.normalize();
    }
    
    Eigen::Vector2d outward_dir = -main_dir;
    Eigen::Vector2d perp_dir(-main_dir.y(), main_dir.x());
    
    Eigen::Vector2d target_dir;
    if (std::abs(target_angle_deg - (-90.0)) < 1e-6) {
      target_dir = -perp_dir;
    } else if (std::abs(target_angle_deg - 90.0) < 1e-6) {
      target_dir = perp_dir;
    } else {
      double target_angle_rad = target_angle_deg * M_PI / 180.0;
      target_dir = std::cos(target_angle_rad) * outward_dir + std::sin(target_angle_rad) * perp_dir;
    }
    target_dir.normalize();
    
    // Search by progressively expanding range: 5m -> 7m -> 9m -> max
    std::vector<double> search_radii = {max_distance, 7.0, 9.0};
    if (skeletonized_grid_) {
      double grid_width = skeletonized_grid_->info.width * skeletonized_grid_->info.resolution;
      double grid_height = skeletonized_grid_->info.height * skeletonized_grid_->info.resolution;
      double max_radius = std::sqrt(grid_width * grid_width + grid_height * grid_height) * 2.0;
      search_radii.push_back(max_radius);
    } else {
      search_radii.push_back(1000.0);
    }
    
    // Find candidates in each range
    for (double search_radius : search_radii) {
      std::vector<Eigen::Vector2d> candidates_in_semicircle;
      
      for (const auto& voro_point : voronoi_boundary_points_) {
        Eigen::Vector2d dir = voro_point - endpoint;
        double dist = dir.norm();
        
        if (dist < min_distance || dist > search_radius) continue;
        
        dir.normalize();
        
        double dot_with_outward = outward_dir.dot(dir);
        if (dot_with_outward < 0.0) continue;
        
        double dot_with_perp = perp_dir.dot(dir);
        
        if (std::abs(target_angle_deg - (-90.0)) < 1e-6) {
          if (dot_with_perp > 0.0) continue;
        } else if (std::abs(target_angle_deg - 90.0) < 1e-6) {
          if (dot_with_perp < 0.0) continue;
        }
        
        candidates_in_semicircle.push_back(voro_point);
      }
      
      if (!candidates_in_semicircle.empty()) {
        // Select closest point among candidates
        double min_dist = std::numeric_limits<double>::max();
        size_t min_idx = 0;
        
        for (size_t i = 0; i < candidates_in_semicircle.size(); ++i) {
          double dist = (candidates_in_semicircle[i] - endpoint).norm();
          if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
          }
        }
        
        point = candidates_in_semicircle[min_idx];
        valid = true;
        return;  // Return immediately if found
      }
    }
    
    // If no candidate found in any range, cast ray
    // Set max_distance sufficiently large to reach grid boundary
    double effective_max_distance = max_distance;
    if (skeletonized_grid_) {
      // Calculate grid diagonal length and set to sufficiently large value
      double grid_width = skeletonized_grid_->info.width * skeletonized_grid_->info.resolution;
      double grid_height = skeletonized_grid_->info.height * skeletonized_grid_->info.resolution;
      effective_max_distance = std::sqrt(grid_width * grid_width + grid_height * grid_height) * 2.0;
    } else {
      // Use sufficiently large value if grid doesn't exist
      effective_max_distance = 1000.0;
    }
    point = castRay(endpoint, other_endpoint, target_angle_deg, min_distance, effective_max_distance);
    valid = true;
  }

  // generateRayPoints and addRayPointsToSeeds functions moved to aos_seed_gen_node.cpp

  void buildGraphFromBoundaryPoints() {
    const int M = static_cast<int>(voronoi_boundary_points_.size());
    edges_.clear();
    edge_records_.clear();
    voro_nodes_ = voronoi_boundary_points_;

    if (M == 0) {
      return;
    }

    edges_.resize(M);

    const auto& voronoi_edges = voronoi_diagram_.getEdges();

    if (voronoi_edges.empty()) {
      return;
    }

    auto findNearestBoundaryPoint = [&](const Eigen::Vector2d& edge_point) -> int {
      int nearest_idx = -1;
      double min_dist = std::numeric_limits<double>::max();

      for (int i = 0; i < M; ++i) {
        double dist = (voronoi_boundary_points_[i] - edge_point).norm();
        if (dist < min_dist) {
          min_dist = dist;
          nearest_idx = i;
        }
      }
      return nearest_idx;
    };

    std::unordered_set<int64_t> added_edges;

    for (const auto& voro_edge : voronoi_edges) {
      int start_idx = findNearestBoundaryPoint(voro_edge.start);
      int end_idx = findNearestBoundaryPoint(voro_edge.end);

      if (start_idx >= 0 && end_idx >= 0 && start_idx != end_idx) {
        const auto& start_point = voronoi_boundary_points_[start_idx];
        const auto& end_point = voronoi_boundary_points_[end_idx];

        int a = start_idx;
        int b = end_idx;
        if (a > b) {
          std::swap(a, b);
        }

        int64_t edge_key = (static_cast<int64_t>(a) << 32) ^ static_cast<uint32_t>(b);

        if (added_edges.find(edge_key) == added_edges.end()) {
          // Remove edges that pass through occupied cells
          if (edgePassesThroughOccupiedPixels(start_point, end_point)) {
            continue;
          }
          
          added_edges.insert(edge_key);

          edges_[start_idx].push_back(end_idx);
          edges_[end_idx].push_back(start_idx);

          double length_m = (end_point - start_point).norm();
          edge_records_.push_back(EdgeRecord{a, b, length_m, 0.0f});
        }
      }
    }

    // Also create direct edges between nearby boundary points
    const double nearby_edge_threshold = 0.5;
    for (int i = 0; i < M; ++i) {
      for (int j = i + 1; j < M; ++j) {
        double dist = (voronoi_boundary_points_[i] - voronoi_boundary_points_[j]).norm();
        if (dist <= nearby_edge_threshold && dist > 1e-6) {
          const auto& start_point = voronoi_boundary_points_[i];
          const auto& end_point = voronoi_boundary_points_[j];
          
          int a = i;
          int b = j;
          if (a > b) {
            std::swap(a, b);
          }

          int64_t edge_key = (static_cast<int64_t>(a) << 32) ^ static_cast<uint32_t>(b);

          if (added_edges.find(edge_key) == added_edges.end()) {
            // Remove edges that pass through occupied cells
            if (edgePassesThroughOccupiedPixels(start_point, end_point)) {
              continue;
            }
            
            added_edges.insert(edge_key);

            edges_[i].push_back(j);
            edges_[j].push_back(i);

            double edge_length = dist;
            edge_records_.push_back(EdgeRecord{a, b, edge_length, 0.0f});
          }
        }
      }
    }
  }

  void publishGraph(const std_msgs::msg::Header &h) {
    aos::msg::GvdGraph graph_msg;

    graph_msg.header = h;
    graph_msg.header.frame_id = "map";

    graph_msg.resolution = skeletonized_grid_ ? skeletonized_grid_->info.resolution : 0.05;
    if (skeletonized_grid_) {
      graph_msg.origin_x = skeletonized_grid_->info.origin.position.x;
      graph_msg.origin_y = skeletonized_grid_->info.origin.position.y;
    } else {
      graph_msg.origin_x = 0.0;
      graph_msg.origin_y = 0.0;
    }

    graph_msg.num_nodes = static_cast<int>(voronoi_boundary_points_.size());
    graph_msg.nodes.reserve(voronoi_boundary_points_.size());
    graph_msg.node_labels.reserve(voronoi_boundary_points_.size());
    graph_msg.node_cluster_indices.reserve(voronoi_boundary_points_.size());
    graph_msg.node_label_counts.reserve(voronoi_boundary_points_.size());

    const double label_match_tolerance = 0.1;
    
    for (size_t i = 0; i < voronoi_boundary_points_.size(); ++i) {
      const auto& point = voronoi_boundary_points_[i];
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = 0.0;
      graph_msg.nodes.push_back(p);
      
      // Combine multiple labels using bitmask: 1=TL, 2=TR, 4=BL, 8=BR
      int label_mask = 0;
      int cluster_idx = -1;  // Store first matching cluster index (deprecated, kept for compatibility)
      
      // Vector to store label information for each node
      std::vector<std::pair<int, int>> node_label_info;  // (cluster_id, label_type)
      
      // Check all clusters to collect all matching labels
      for (size_t cluster_id = 0; cluster_id < cluster_voronoi_boundary_points_.size(); ++cluster_id) {
        const auto& cluster_voro = cluster_voronoi_boundary_points_[cluster_id];
        
        if (cluster_voro.ep1_valid[0]) {
          double dist_tl = (point - cluster_voro.ep1_boundary_points[0]).norm();
          if (dist_tl < label_match_tolerance) {
            label_mask |= (1 << 0);  // TL: bit 0
            node_label_info.push_back({static_cast<int>(cluster_id), 0});  // 0=TL
            if (cluster_idx == -1) {
              cluster_idx = static_cast<int>(cluster_id);
            }
          }
        }
        
        if (cluster_voro.ep1_valid[1]) {
          double dist_tr = (point - cluster_voro.ep1_boundary_points[1]).norm();
          if (dist_tr < label_match_tolerance) {
            label_mask |= (1 << 1);  // TR: bit 1
            node_label_info.push_back({static_cast<int>(cluster_id), 1});  // 1=TR
            if (cluster_idx == -1) {
              cluster_idx = static_cast<int>(cluster_id);
            }
          }
        }
        
        if (cluster_voro.ep2_valid[0]) {
          double dist_bl = (point - cluster_voro.ep2_boundary_points[0]).norm();
          if (dist_bl < label_match_tolerance) {
            label_mask |= (1 << 2);  // BL: bit 2
            node_label_info.push_back({static_cast<int>(cluster_id), 2});  // 2=BL
            if (cluster_idx == -1) {
              cluster_idx = static_cast<int>(cluster_id);
            }
          }
        }
        
        if (cluster_voro.ep2_valid[1]) {
          double dist_br = (point - cluster_voro.ep2_boundary_points[1]).norm();
          if (dist_br < label_match_tolerance) {
            label_mask |= (1 << 3);  // BR: bit 3
            node_label_info.push_back({static_cast<int>(cluster_id), 3});  // 3=BR
            if (cluster_idx == -1) {
              cluster_idx = static_cast<int>(cluster_id);
            }
          }
        }
      }
      
      // Store bitmask as integer (0=normal, 1=TL, 2=TR, 3=TL+TR, 4=BL, 5=TL+BL, ...)
      // Use bitmask value as-is for backward compatibility
      graph_msg.node_labels.push_back(label_mask);
      graph_msg.node_cluster_indices.push_back(cluster_idx);
      
      // Store cluster-specific label information in new fields
      graph_msg.node_label_counts.push_back(static_cast<int>(node_label_info.size()));
      for (const auto& [cluster_id, label_type] : node_label_info) {
        graph_msg.node_label_clusters.push_back(cluster_id);
        graph_msg.node_label_types.push_back(label_type);
      }
    }

    graph_msg.num_edges = static_cast<int>(edge_records_.size());
    graph_msg.edges.reserve(edge_records_.size() * 2);
    graph_msg.edge_lengths.reserve(edge_records_.size());
    graph_msg.edge_clearances.reserve(edge_records_.size());

    for (const auto &edge : edge_records_) {
      graph_msg.edges.push_back(edge.from);
      graph_msg.edges.push_back(edge.to);
      graph_msg.edge_lengths.push_back(static_cast<float>(edge.length_m));
      graph_msg.edge_clearances.push_back(edge.min_clearance_m);
    }

    pub_graph_->publish(graph_msg);
  }

  void publishMarkers(const std_msgs::msg::Header &h) {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(delete_all);

    // Visualize Voronoi seeds (clearly distinguished in yellow)
    if (!voronoi_seeds_.empty()) {
      visualization_msgs::msg::Marker seeds_marker;
      seeds_marker.header = h;
      seeds_marker.ns = "/gvd_voronoi_seeds";
      seeds_marker.id = 0;
      seeds_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      seeds_marker.action = visualization_msgs::msg::Marker::ADD;
      seeds_marker.scale.x = 0.2;
      seeds_marker.scale.y = 0.2;
      seeds_marker.scale.z = 0.2;
      seeds_marker.color.r = 1.0f;
      seeds_marker.color.g = 1.0f;
      seeds_marker.color.b = 0.0f;  // Yellow (seeds)
      seeds_marker.color.a = 1.0f;

      for (const auto& seed : voronoi_seeds_) {
        geometry_msgs::msg::Point p;
        p.x = seed.x();
        p.y = seed.y();
        p.z = 0.0;
        seeds_marker.points.push_back(p);
      }
      ma.markers.push_back(seeds_marker);
    }

    // Visualize Voronoi nodes (changed to purple to distinguish from seeds)
    visualization_msgs::msg::Marker nodes_marker;
    nodes_marker.header = h;
    nodes_marker.ns = "/gvd_voronoi_nodes";
    nodes_marker.id = 0;
    nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodes_marker.action = visualization_msgs::msg::Marker::ADD;
    nodes_marker.scale.x = 0.15;  // Increased size
    nodes_marker.scale.y = 0.15;
    nodes_marker.scale.z = 0.15;
    nodes_marker.color.r = 0.8f;
    nodes_marker.color.g = 0.0f;
    nodes_marker.color.b = 0.8f;  // Purple (graph nodes)
    nodes_marker.color.a = 1.0f;

    for (const auto& point : voronoi_boundary_points_) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = 0.0;
      nodes_marker.points.push_back(p);
    }
    ma.markers.push_back(nodes_marker);

    // Visualize Voronoi edges (thicker and clearer)
    visualization_msgs::msg::Marker edges_marker;
    edges_marker.header = h;
    edges_marker.ns = "/gvd_voronoi_edges";
    edges_marker.id = 0;
    edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::msg::Marker::ADD;
    edges_marker.scale.x = 0.08;  // Increased thickness (0.03 -> 0.08)
    edges_marker.color.r = 0.0f;
    edges_marker.color.g = 0.8f;
    edges_marker.color.b = 1.0f;  // Sky blue
    edges_marker.color.a = 1.0f;  // Increased opacity

    for (const auto& edge : edge_records_) {
      const auto& from_point = voronoi_boundary_points_[edge.from];
      const auto& to_point = voronoi_boundary_points_[edge.to];

      geometry_msgs::msg::Point p1, p2;
      p1.x = from_point.x();
      p1.y = from_point.y();
      p1.z = 0.0;
      p2.x = to_point.x();
      p2.y = to_point.y();
      p2.z = 0.0;
      edges_marker.points.push_back(p1);
      edges_marker.points.push_back(p2);
    }
    ma.markers.push_back(edges_marker);

    // Visualize Voronoi cells (pure Voronoi partition)
    const auto& cell_boundaries = voronoi_diagram_.extractCellBoundaries();
    const auto& voronoi_seeds_from_diagram = voronoi_diagram_.getSeeds();
    
    for (size_t i = 0; i < cell_boundaries.size(); ++i) {
      if (cell_boundaries[i].size() < 3) {
        continue;
      }

      // Fill cell interior (triangle list)
      visualization_msgs::msg::Marker cell_marker;
      cell_marker.header = h;
      cell_marker.ns = "/gvd_voronoi_cells";
      cell_marker.id = static_cast<int>(i);
      cell_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      cell_marker.action = visualization_msgs::msg::Marker::ADD;
      cell_marker.scale.x = 1.0;
      cell_marker.scale.y = 1.0;
      cell_marker.scale.z = 1.0;

      // Different color for each cell (using HSV color space)
      float hue = static_cast<float>(i) / std::max(1.0f, static_cast<float>(cell_boundaries.size()));
      float saturation = 0.7f;
      float value = 0.9f;

      float c = value * saturation;
      float x = c * (1.0f - std::abs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
      float m = value - c;
      
      float r = 0.0f, g = 0.0f, b = 0.0f;
      if (hue < 1.0f / 6.0f) {
        r = c; g = x; b = 0.0f;
      } else if (hue < 2.0f / 6.0f) {
        r = x; g = c; b = 0.0f;
      } else if (hue < 3.0f / 6.0f) {
        r = 0.0f; g = c; b = x;
      } else if (hue < 4.0f / 6.0f) {
        r = 0.0f; g = x; b = c;
      } else if (hue < 5.0f / 6.0f) {
        r = x; g = 0.0f; b = c;
      } else {
        r = c; g = 0.0f; b = x;
      }
      
      cell_marker.color.r = r + m;
      cell_marker.color.g = g + m;
      cell_marker.color.b = b + m;
      cell_marker.color.a = 0.4f;  // Semi-transparent

      // Generate triangles centered at seed position
      if (i < voronoi_seeds_from_diagram.size()) {
        const auto& seed = voronoi_seeds_from_diagram[i];
        geometry_msgs::msg::Point center;
        center.x = seed.x();
        center.y = seed.y();
        center.z = 0.0;

        // Generate triangles from each boundary point to center
        for (size_t j = 0; j < cell_boundaries[i].size(); ++j) {
          size_t next_j = (j + 1) % cell_boundaries[i].size();

          cell_marker.points.push_back(center);
          
          geometry_msgs::msg::Point p1;
          p1.x = cell_boundaries[i][j].x();
          p1.y = cell_boundaries[i][j].y();
          p1.z = 0.0;
          cell_marker.points.push_back(p1);
          
          geometry_msgs::msg::Point p2;
          p2.x = cell_boundaries[i][next_j].x();
          p2.y = cell_boundaries[i][next_j].y();
          p2.z = 0.0;
          cell_marker.points.push_back(p2);
        }
      }

      ma.markers.push_back(cell_marker);

      // Visualize cell boundaries
      visualization_msgs::msg::Marker cell_boundary_marker;
      cell_boundary_marker.header = h;
      cell_boundary_marker.ns = "/gvd_voronoi_cell_boundaries";
      cell_boundary_marker.id = static_cast<int>(i);
      cell_boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      cell_boundary_marker.action = visualization_msgs::msg::Marker::ADD;
      cell_boundary_marker.scale.x = 0.05;
      cell_boundary_marker.color.r = 0.0f;
      cell_boundary_marker.color.g = 0.0f;
      cell_boundary_marker.color.b = 0.0f;
      cell_boundary_marker.color.a = 0.8f;
      
      for (const auto& point : cell_boundaries[i]) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.0;
        cell_boundary_marker.points.push_back(p);
      }

      // Add first point to close boundary
      if (!cell_boundaries[i].empty()) {
        geometry_msgs::msg::Point p;
        p.x = cell_boundaries[i][0].x();
        p.y = cell_boundaries[i][0].y();
        p.z = 0.0;
        cell_boundary_marker.points.push_back(p);
      }

      ma.markers.push_back(cell_boundary_marker);
    }

    // Visualize labeled nodes (TL/TR/BL/BR)
    const double label_match_tolerance = 0.1;
    std::vector<int> node_labels(voronoi_boundary_points_.size(), 0);
    
    for (size_t i = 0; i < voronoi_boundary_points_.size(); ++i) {
      const auto& point = voronoi_boundary_points_[i];
      int label_mask = 0;
      
      for (size_t cluster_id = 0; cluster_id < cluster_voronoi_boundary_points_.size(); ++cluster_id) {
        const auto& cluster_voro = cluster_voronoi_boundary_points_[cluster_id];
        
        if (cluster_voro.ep1_valid[0]) {
          double dist_tl = (point - cluster_voro.ep1_boundary_points[0]).norm();
          if (dist_tl < label_match_tolerance) {
            label_mask |= (1 << 0);  // TL
          }
        }
        if (cluster_voro.ep1_valid[1]) {
          double dist_tr = (point - cluster_voro.ep1_boundary_points[1]).norm();
          if (dist_tr < label_match_tolerance) {
            label_mask |= (1 << 1);  // TR
          }
        }
        if (cluster_voro.ep2_valid[0]) {
          double dist_bl = (point - cluster_voro.ep2_boundary_points[0]).norm();
          if (dist_bl < label_match_tolerance) {
            label_mask |= (1 << 2);  // BL
          }
        }
        if (cluster_voro.ep2_valid[1]) {
          double dist_br = (point - cluster_voro.ep2_boundary_points[1]).norm();
          if (dist_br < label_match_tolerance) {
            label_mask |= (1 << 3);  // BR
          }
        }
      }
      
      node_labels[i] = label_mask;
    }
    
    // Display labeled nodes distinguished by color (larger and above default nodes)
    visualization_msgs::msg::Marker labeled_nodes_marker;
    labeled_nodes_marker.header = h;
    labeled_nodes_marker.ns = "/gvd_labeled_nodes";
    labeled_nodes_marker.id = 0;
    labeled_nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    labeled_nodes_marker.action = visualization_msgs::msg::Marker::ADD;
    labeled_nodes_marker.scale.x = 0.3;  // Larger than default nodes
    labeled_nodes_marker.scale.y = 0.3;
    labeled_nodes_marker.scale.z = 0.3;
    
    std::vector<std_msgs::msg::ColorRGBA> node_colors;
    // Add only nodes with labels
    for (size_t i = 0; i < voronoi_boundary_points_.size(); ++i) {
      int label_mask = node_labels[i];
      
      // Add only nodes with labels
      if (label_mask != 0) {
        const auto& point = voronoi_boundary_points_[i];
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.1;  // Display slightly above default nodes
        labeled_nodes_marker.points.push_back(p);
        
        // Calculate color based on bitmask
        float r = 0.0f, g = 0.0f, b = 0.0f;
        int label_count = 0;
        
        if (label_mask & (1 << 0)) {  // TL: Cyan
          r += 0.0f; g += 1.0f; b += 1.0f;
          label_count++;
        }
        if (label_mask & (1 << 1)) {  // TR: Orange
          r += 1.0f; g += 0.5f; b += 0.0f;
          label_count++;
        }
        if (label_mask & (1 << 2)) {  // BL: Cyan
          r += 0.0f; g += 1.0f; b += 1.0f;
          label_count++;
        }
        if (label_mask & (1 << 3)) {  // BR: Orange
          r += 1.0f; g += 0.5f; b += 0.0f;
          label_count++;
        }
        
        std_msgs::msg::ColorRGBA color;
        if (label_count > 0) {
          color.r = r / label_count;
          color.g = g / label_count;
          color.b = b / label_count;
          color.a = 1.0f;
        } else {
          color.r = 0.0f;
          color.g = 0.0f;
          color.b = 1.0f;
          color.a = 1.0f;
        }
        node_colors.push_back(color);
      }
    }
    
    labeled_nodes_marker.colors = node_colors;
    if (!labeled_nodes_marker.points.empty()) {
      ma.markers.push_back(labeled_nodes_marker);
    }
    
    // Display label text
    int label_text_id = 0;
    for (size_t i = 0; i < voronoi_boundary_points_.size(); ++i) {
      int label_mask = node_labels[i];
      if (label_mask != 0) {
        const auto& point = voronoi_boundary_points_[i];
        double z_offset = 0.3;
        
        if (label_mask & (1 << 0)) {  // TL
          visualization_msgs::msg::Marker label_marker;
          label_marker.header = h;
          label_marker.ns = "/gvd_node_labels";
          label_marker.id = label_text_id++;
          label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          label_marker.action = visualization_msgs::msg::Marker::ADD;
          label_marker.pose.position.x = point.x();
          label_marker.pose.position.y = point.y();
          label_marker.pose.position.z = z_offset;
          label_marker.pose.orientation.w = 1.0;
          label_marker.scale.z = 0.5;
          label_marker.text = "TL";
          label_marker.color.r = 0.0f;
          label_marker.color.g = 1.0f;
          label_marker.color.b = 1.0f;
          label_marker.color.a = 1.0f;
          ma.markers.push_back(label_marker);
          z_offset += 0.1;
        }
        
        if (label_mask & (1 << 1)) {  // TR
          visualization_msgs::msg::Marker label_marker;
          label_marker.header = h;
          label_marker.ns = "/gvd_node_labels";
          label_marker.id = label_text_id++;
          label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          label_marker.action = visualization_msgs::msg::Marker::ADD;
          label_marker.pose.position.x = point.x();
          label_marker.pose.position.y = point.y();
          label_marker.pose.position.z = z_offset;
          label_marker.pose.orientation.w = 1.0;
          label_marker.scale.z = 0.5;
          label_marker.text = "TR";
          label_marker.color.r = 1.0f;
          label_marker.color.g = 0.5f;
          label_marker.color.b = 0.0f;
          label_marker.color.a = 1.0f;
          ma.markers.push_back(label_marker);
          z_offset += 0.1;
        }
        
        if (label_mask & (1 << 2)) {  // BL
          visualization_msgs::msg::Marker label_marker;
          label_marker.header = h;
          label_marker.ns = "/gvd_node_labels";
          label_marker.id = label_text_id++;
          label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          label_marker.action = visualization_msgs::msg::Marker::ADD;
          label_marker.pose.position.x = point.x();
          label_marker.pose.position.y = point.y();
          label_marker.pose.position.z = z_offset;
          label_marker.pose.orientation.w = 1.0;
          label_marker.scale.z = 0.5;
          label_marker.text = "BL";
          label_marker.color.r = 0.0f;
          label_marker.color.g = 1.0f;
          label_marker.color.b = 1.0f;
          label_marker.color.a = 1.0f;
          ma.markers.push_back(label_marker);
          z_offset += 0.1;
        }
        
        if (label_mask & (1 << 3)) {  // BR
          visualization_msgs::msg::Marker label_marker;
          label_marker.header = h;
          label_marker.ns = "/gvd_node_labels";
          label_marker.id = label_text_id++;
          label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          label_marker.action = visualization_msgs::msg::Marker::ADD;
          label_marker.pose.position.x = point.x();
          label_marker.pose.position.y = point.y();
          label_marker.pose.position.z = z_offset;
          label_marker.pose.orientation.w = 1.0;
          label_marker.scale.z = 0.5;
          label_marker.text = "BR";
          label_marker.color.r = 1.0f;
          label_marker.color.g = 0.5f;
          label_marker.color.b = 0.0f;
          label_marker.color.a = 1.0f;
          ma.markers.push_back(label_marker);
        }
      }
    }
    
    // Visualize cluster endpoints
    for (size_t i = 0; i < exploration_tree_rows_.size(); ++i) {
      const auto& tree_row = exploration_tree_rows_[i];
      
      // ep1 (red)
      visualization_msgs::msg::Marker ep1_marker;
      ep1_marker.header = h;
      ep1_marker.ns = "/gvd_cluster_endpoints";
      ep1_marker.id = static_cast<int>(i * 2);
      ep1_marker.type = visualization_msgs::msg::Marker::SPHERE;
      ep1_marker.action = visualization_msgs::msg::Marker::ADD;
      ep1_marker.pose.position.x = tree_row.first.x();
      ep1_marker.pose.position.y = tree_row.first.y();
      ep1_marker.pose.position.z = 0.0;
      ep1_marker.pose.orientation.w = 1.0;
      ep1_marker.scale.x = 0.5;
      ep1_marker.scale.y = 0.5;
      ep1_marker.scale.z = 0.5;
      ep1_marker.color.r = 1.0f;
      ep1_marker.color.g = 0.0f;
      ep1_marker.color.b = 0.0f;
      ep1_marker.color.a = 1.0f;
      ma.markers.push_back(ep1_marker);
      
      // ep2 (blue)
      visualization_msgs::msg::Marker ep2_marker;
      ep2_marker.header = h;
      ep2_marker.ns = "/gvd_cluster_endpoints";
      ep2_marker.id = static_cast<int>(i * 2 + 1);
      ep2_marker.type = visualization_msgs::msg::Marker::SPHERE;
      ep2_marker.action = visualization_msgs::msg::Marker::ADD;
      ep2_marker.pose.position.x = tree_row.second.x();
      ep2_marker.pose.position.y = tree_row.second.y();
      ep2_marker.pose.position.z = 0.0;
      ep2_marker.pose.orientation.w = 1.0;
      ep2_marker.scale.x = 0.5;
      ep2_marker.scale.y = 0.5;
      ep2_marker.scale.z = 0.5;
      ep2_marker.color.r = 0.0f;
      ep2_marker.color.g = 0.0f;
      ep2_marker.color.b = 1.0f;
      ep2_marker.color.a = 1.0f;
      ma.markers.push_back(ep2_marker);
    }
    
    // Ray point visualization is performed in aos_seed_gen_node.cpp
    
    // Visualize TL/TR/BL/BR Voronoi boundary points
    for (size_t i = 0; i < cluster_voronoi_boundary_points_.size(); ++i) {
      const auto& voro_data = cluster_voronoi_boundary_points_[i];
      
      // ep1's TL/TR boundary points
      for (size_t j = 0; j < 2; ++j) {
        if (voro_data.ep1_valid[j]) {
          visualization_msgs::msg::Marker voro_marker;
          voro_marker.header = h;
          voro_marker.ns = "/gvd_ep1_voronoi_boundary";
          voro_marker.id = static_cast<int>(i * 4 + j);
          voro_marker.type = visualization_msgs::msg::Marker::SPHERE;
          voro_marker.action = visualization_msgs::msg::Marker::ADD;
          voro_marker.pose.position.x = voro_data.ep1_boundary_points[j].x();
          voro_marker.pose.position.y = voro_data.ep1_boundary_points[j].y();
          voro_marker.pose.position.z = 0.0;
          voro_marker.pose.orientation.w = 1.0;
          voro_marker.scale.x = 0.3;
          voro_marker.scale.y = 0.3;
          voro_marker.scale.z = 0.3;
          if (j == 0) {
            // TL: Cyan
            voro_marker.color.r = 0.0f;
            voro_marker.color.g = 1.0f;
            voro_marker.color.b = 1.0f;
          } else {
            // TR: Orange
            voro_marker.color.r = 1.0f;
            voro_marker.color.g = 0.5f;
            voro_marker.color.b = 0.0f;
          }
          voro_marker.color.a = 1.0f;
          ma.markers.push_back(voro_marker);
        }
      }
      
      // ep2's BL/BR boundary points
      for (size_t j = 0; j < 2; ++j) {
        if (voro_data.ep2_valid[j]) {
          visualization_msgs::msg::Marker voro_marker;
          voro_marker.header = h;
          voro_marker.ns = "/gvd_ep2_voronoi_boundary";
          voro_marker.id = static_cast<int>(i * 4 + j + 2);
          voro_marker.type = visualization_msgs::msg::Marker::SPHERE;
          voro_marker.action = visualization_msgs::msg::Marker::ADD;
          voro_marker.pose.position.x = voro_data.ep2_boundary_points[j].x();
          voro_marker.pose.position.y = voro_data.ep2_boundary_points[j].y();
          voro_marker.pose.position.z = 0.0;
          voro_marker.pose.orientation.w = 1.0;
          voro_marker.scale.x = 0.3;
          voro_marker.scale.y = 0.3;
          voro_marker.scale.z = 0.3;
          if (j == 0) {
            // BL: Cyan
            voro_marker.color.r = 0.0f;
            voro_marker.color.g = 1.0f;
            voro_marker.color.b = 1.0f;
          } else {
            // BR: Orange
            voro_marker.color.r = 1.0f;
            voro_marker.color.g = 0.5f;
            voro_marker.color.b = 0.0f;
          }
          voro_marker.color.a = 1.0f;
          ma.markers.push_back(voro_marker);
        }
      }
      
      // Lines from endpoints to Voronoi boundary points
      visualization_msgs::msg::Marker ep1_lines;
      ep1_lines.header = h;
      ep1_lines.ns = "/gvd_ep1_voronoi_lines";
      ep1_lines.id = static_cast<int>(i * 2);
      ep1_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      ep1_lines.action = visualization_msgs::msg::Marker::ADD;
      ep1_lines.scale.x = 0.03;
      ep1_lines.color.r = 0.0f;
      ep1_lines.color.g = 0.8f;
      ep1_lines.color.b = 0.8f;
      ep1_lines.color.a = 0.7f;
      
      geometry_msgs::msg::Point p1;
      p1.x = voro_data.endpoint1.x();
      p1.y = voro_data.endpoint1.y();
      p1.z = 0.0;
      
      for (size_t j = 0; j < 2; ++j) {
        if (voro_data.ep1_valid[j]) {
          geometry_msgs::msg::Point p2;
          p2.x = voro_data.ep1_boundary_points[j].x();
          p2.y = voro_data.ep1_boundary_points[j].y();
          p2.z = 0.0;
          ep1_lines.points.push_back(p1);
          ep1_lines.points.push_back(p2);
        }
      }
      
      if (!ep1_lines.points.empty()) {
        ma.markers.push_back(ep1_lines);
      }
      
      visualization_msgs::msg::Marker ep2_lines;
      ep2_lines.header = h;
      ep2_lines.ns = "/gvd_ep2_voronoi_lines";
      ep2_lines.id = static_cast<int>(i * 2 + 1);
      ep2_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      ep2_lines.action = visualization_msgs::msg::Marker::ADD;
      ep2_lines.scale.x = 0.03;
      ep2_lines.color.r = 1.0f;
      ep2_lines.color.g = 0.5f;
      ep2_lines.color.b = 0.0f;
      ep2_lines.color.a = 0.7f;
      
      p1.x = voro_data.endpoint2.x();
      p1.y = voro_data.endpoint2.y();
      p1.z = 0.0;
      
      for (size_t j = 0; j < 2; ++j) {
        if (voro_data.ep2_valid[j]) {
          geometry_msgs::msg::Point p2;
          p2.x = voro_data.ep2_boundary_points[j].x();
          p2.y = voro_data.ep2_boundary_points[j].y();
          p2.z = 0.0;
          ep2_lines.points.push_back(p1);
          ep2_lines.points.push_back(p2);
        }
      }
      
      if (!ep2_lines.points.empty()) {
        ma.markers.push_back(ep2_lines);
      }
    }

    pub_markers_->publish(ma);
  }

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_voronoi_seeds_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_exploration_tree_rows_info_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_all_tree_rows_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_skeletonized_grid_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_grid_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_robot_position_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_docking_state_;

  // Publishers
  rclcpp::Publisher<aos::msg::GvdGraph>::SharedPtr pub_graph_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // Data
  std::vector<Eigen::Vector2d> voronoi_seeds_;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> exploration_tree_rows_;  // {start, end}
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> all_tree_rows_;  // All tree rows {start, end}
  std::optional<nav_msgs::msg::OccupancyGrid> skeletonized_grid_;
  std::optional<nav_msgs::msg::OccupancyGrid> occupancy_grid_;
  Eigen::Vector2d robot_position_;  // Robot position
  bool robot_position_received_ = false;
  bool waiting_for_docking_completion_ = false;  // Waiting for docking completion state

  aos::VoronoiDiagram voronoi_diagram_;
  std::vector<Eigen::Vector2d> voronoi_boundary_points_;
  std::vector<Eigen::Vector2d> voro_nodes_;
  std::vector<std::vector<int>> edges_;
  
  struct ClusterVoronoiBoundaryPoints {
    Eigen::Vector2d endpoint1;
    Eigen::Vector2d endpoint2;
    std::array<Eigen::Vector2d, 2> ep1_boundary_points;
    std::array<Eigen::Vector2d, 2> ep2_boundary_points;
    std::array<bool, 2> ep1_valid;
    std::array<bool, 2> ep2_valid;
  };
  std::vector<ClusterVoronoiBoundaryPoints> cluster_voronoi_boundary_points_;
  
  // ClusterRayPoints and ray_points_ moved to aos_seed_gen_node.cpp

  struct EdgeRecord {
    int from;
    int to;
    double length_m;
    float min_clearance_m;
  };
  std::vector<EdgeRecord> edge_records_;

  std::mutex data_mutex_;
  std::chrono::steady_clock::time_point last_graph_publish_time_;
  double max_graph_publish_rate_ = 10.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AosGvdNode>());
  rclcpp::shutdown();
  return 0;
}
