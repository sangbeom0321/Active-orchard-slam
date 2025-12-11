#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "aos/msg/gvd_graph.hpp"
#include "lio_sam_wo/srv/save_map.hpp"
#include <std_srvs/srv/empty.hpp>

// Log level macros for conditional logging
#define LOG_DEBUG(...) if (log_level_ >= 4 || debug_mode_) RCLCPP_DEBUG(this->get_logger(), __VA_ARGS__)
#define LOG_INFO(...) if (log_level_ >= 3) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
#define LOG_WARN(...) if (log_level_ >= 2) RCLCPP_WARN(this->get_logger(), __VA_ARGS__)
#define LOG_ERROR(...) if (log_level_ >= 1) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__)

class AosPathGenNode : public rclcpp::Node {
public:
  AosPathGenNode() : rclcpp::Node("aos_path_gen_node") {
    // Parameters
    this->declare_parameter("gvd_graph_topic", std::string("/gvd/graph"));
    this->declare_parameter("publish_namespace", std::string("gvd_cluster_planning"));
    this->declare_parameter("debug_mode", false);
    this->declare_parameter("log_level", std::string("WARN"));  // Log level: "NONE", "ERROR", "WARN", "INFO", "DEBUG"
    this->declare_parameter("current_pos_topic", std::string("/Local/utm"));
    this->declare_parameter("control_mod_topic", std::string("/Control/mod"));
    this->declare_parameter("skeletonized_grid_topic", std::string("/skeletonized_occupancy_grid"));
    
    this->get_parameter("gvd_graph_topic", gvd_graph_topic_);
    this->get_parameter("publish_namespace", ns_);
    this->get_parameter("debug_mode", debug_mode_);
    std::string log_level_str;
    this->get_parameter("log_level", log_level_str);
    // Parse log level: NONE < ERROR < WARN < INFO < DEBUG
    if (log_level_str == "NONE") {
      log_level_ = 0;
    } else if (log_level_str == "ERROR") {
      log_level_ = 1;
    } else if (log_level_str == "WARN") {
      log_level_ = 2;
    } else if (log_level_str == "INFO") {
      log_level_ = 3;
    } else if (log_level_str == "DEBUG") {
      log_level_ = 4;
    } else {
      log_level_ = 2;  // Default to WARN
    }
    std::string current_pos_topic, control_mod_topic, skeletonized_grid_topic;
    this->get_parameter("current_pos_topic", current_pos_topic);
    this->get_parameter("control_mod_topic", control_mod_topic);
    this->get_parameter("skeletonized_grid_topic", skeletonized_grid_topic);

    // QoS settings
    rclcpp::QoS reliable_qos(10);
    reliable_qos.reliable();
    
    rclcpp::QoS best_effort_qos(10);
    best_effort_qos.best_effort();
    best_effort_qos.keep_last(1);

    // Subscribers
    sub_graph_ = create_subscription<aos::msg::GvdGraph>(
        gvd_graph_topic_, reliable_qos,
        std::bind(&AosPathGenNode::graphCallback, this, std::placeholders::_1));

    sub_current_pos_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        current_pos_topic, 10,
        std::bind(&AosPathGenNode::currentPosCallback, this, std::placeholders::_1));

    sub_control_mod_ = create_subscription<std_msgs::msg::Int32>(
        control_mod_topic, reliable_qos,
        std::bind(&AosPathGenNode::controlModCallback, this, std::placeholders::_1));

    sub_skeletonized_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        skeletonized_grid_topic, reliable_qos,
        std::bind(&AosPathGenNode::skeletonizedGridCallback, this, std::placeholders::_1));

    // Publishers
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/aos/path", reliable_qos);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/aos/markers", reliable_qos);
    pub_cluster_index_ = create_publisher<std_msgs::msg::Int32>("/aos/current_cluster_index", reliable_qos);
    pub_waypoint_index_ = create_publisher<std_msgs::msg::Int32>("/aos/current_waypoint_index", reliable_qos);
    pub_path_status_ = create_publisher<std_msgs::msg::String>("/aos/path_planning_status", reliable_qos);
    pub_docking_state_ = create_publisher<std_msgs::msg::Bool>("/aos/docking_state", reliable_qos);
    
    // Create service client for saving map
    save_map_client_ = create_client<lio_sam_wo::srv::SaveMap>("/lio_sam/save_map");
    
    // Create service client for saving cluster information
    save_cluster_info_client_ = create_client<std_srvs::srv::Empty>("/gvd/save_cluster_info");
    
    // Create service server for forcing next waypoint
    srv_next_waypoint_ = create_service<std_srvs::srv::Empty>(
        "/aos/next_waypoint",
        std::bind(&AosPathGenNode::nextWaypointServiceCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    // Initialize initial waypoint (8, 0)
    initial_waypoint_.x = 8.0;
    initial_waypoint_.y = 0.0;
    initial_waypoint_.z = 0.0;
    
    // Record node start time (for initial marker publishing delay)
    node_start_time_ = std::chrono::steady_clock::now();
  }

private:
  // ROS
  rclcpp::Subscription<aos::msg::GvdGraph>::SharedPtr sub_graph_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_current_pos_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_control_mod_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_skeletonized_grid_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cluster_index_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_waypoint_index_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_path_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_docking_state_;
  rclcpp::Client<lio_sam_wo::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_cluster_info_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_next_waypoint_;

  // Parameters
  std::string gvd_graph_topic_;
  std::string ns_;
  bool debug_mode_ = false;
  int log_level_ = 2;  // Log level: 0=NONE, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG

  // Current position tracking
  geometry_msgs::msg::Point current_position_;
  bool current_position_received_ = false;
  int32_t current_control_mode_ = -1;

  // Progressive path planning state
  int current_target_waypoint_index_ = -1;  // Current target waypoint index
  int previous_waypoint_index_ = -1;  // Previous waypoint index (just arrived)
  bool waiting_for_docking_completion_ = false;  // Waiting for docking completion signal
  bool initial_waypoint_reached_ = false;  // Whether (8,0) initial waypoint is reached
  bool exploration_completed_ = false;  // Exploration completed flag
  bool map_saved_ = false;  // Flag to prevent multiple map saves
  geometry_msgs::msg::Point initial_waypoint_;  // Initial waypoint (8, 0)
  std::vector<geometry_msgs::msg::Point> fixed_waypoints_;  // Fixed waypoint positions
  std::vector<int> fixed_waypoint_node_indices_;  // Node indices for each waypoint
  nav_msgs::msg::Path last_published_path_;  // Last successfully published path
  
  // Marker publishing throttling
  std::chrono::steady_clock::time_point last_marker_publish_time_;
  std::chrono::steady_clock::time_point node_start_time_;
  static constexpr double MARKER_PUBLISH_INTERVAL = 0.2;  // Publish markers every 200ms
  static constexpr double INITIAL_DELAY = 1.0;  // Delay marker publishing for 1 second after node start

  // GVD Graph data
  bool graph_received_ = false;
  std::vector<geometry_msgs::msg::Point> graph_nodes_;
  std::vector<int> graph_edges_;
  std::vector<float> graph_edge_lengths_;
  std::vector<float> graph_edge_clearances_;
  std::vector<int> node_labels_;  // Bitmask labels (for backward compatibility)
  std::vector<int> node_cluster_indices_;  // deprecated
  std::vector<std::vector<int>> adjacency_list_;
  double resolution_ = 0.1;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  
  // New node label information: which cluster each node label comes from
  std::vector<int> node_label_clusters_;  // Cluster ID array for each node label
  std::vector<int> node_label_types_;  // Label type array for each node (0=TL, 1=TR, 2=BL, 3=BR)
  std::vector<int> node_label_counts_;  // Number of labels for each node

  // Cluster waypoint mapping: cluster_index -> {TL, TR, BL, BR} node indices
  std::unordered_map<int, std::vector<int>> cluster_waypoint_nodes_;

  // Skeletonized occupancy grid
  std::optional<nav_msgs::msg::OccupancyGrid> skeletonized_grid_;

  //--------------------------------------------------------------------------------
  // Callbacks
  //--------------------------------------------------------------------------------
  void currentPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
      current_position_.x = msg->data[0];
      current_position_.y = msg->data[1];
      current_position_.z = 0.0;
      current_position_received_ = true;

      // Check if we reached initial waypoint (8, 0)
      if (!initial_waypoint_reached_) {
        double dx = current_position_.x - initial_waypoint_.x;
        double dy = current_position_.y - initial_waypoint_.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance <= 1.0) {
          initial_waypoint_reached_ = true;
          previous_waypoint_index_ = -1;  // No previous waypoint yet
          if (!fixed_waypoints_.empty()) {
            current_target_waypoint_index_ = 0;  // Start from first waypoint
          }
          if (graph_received_) {
            planAndPublishPath();
          }
          return;
        }
      }

      // Check if we reached current target waypoint
      if (current_target_waypoint_index_ >= 0 && 
          current_target_waypoint_index_ < static_cast<int>(fixed_waypoints_.size())) {
        const auto& target = fixed_waypoints_[current_target_waypoint_index_];
        double dx = current_position_.x - target.x;
        double dy = current_position_.y - target.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        // Check if reached origin (0, 0) after exploration completion
        if (exploration_completed_ && std::abs(target.x) < 0.1 && std::abs(target.y) < 0.1 && dist <= 1.0) {
          // Publish Exploration Complete status
          std_msgs::msg::String status_msg;
          status_msg.data = "Exploration Complete";
          pub_path_status_->publish(status_msg);
          
          // Set cluster and waypoint indices to -1
          std_msgs::msg::Int32 cluster_msg;
          cluster_msg.data = -1;
          pub_cluster_index_->publish(cluster_msg);
          
          std_msgs::msg::Int32 waypoint_msg;
          waypoint_msg.data = -1;
          pub_waypoint_index_->publish(waypoint_msg);
          
          LOG_INFO("Exploration Complete - Returned to origin");
        }
        
        // Enter docking wait state when robot enters docking radius (path planning disabled until mod 3)
        static constexpr double DOCKING_RADIUS = 0.7;  // Distance threshold for entering docking state
        if (dist <= DOCKING_RADIUS && !waiting_for_docking_completion_) {
          waiting_for_docking_completion_ = true;
          publishDockingState();
          LOG_INFO("Entered %.2fm radius of WP[%d] - path planning disabled until arrival (mod 3)",
                   DOCKING_RADIUS,
                   current_target_waypoint_index_);
        }
      }

      // If graph is already received and initial waypoint reached and not waiting for docking, replan with new position
      if (graph_received_ && initial_waypoint_reached_ && !waiting_for_docking_completion_) {
        planAndPublishPath();
      } else if (graph_received_ && !initial_waypoint_reached_) {
        // Still going to initial waypoint, replan initial path
        planAndPublishPath();
      } else if (graph_received_ && waiting_for_docking_completion_ && !last_published_path_.poses.empty()) {
        // Publish last path while waiting for docking completion (path planning disabled within docking radius)
        // Synchronize path and marker timestamps
        rclcpp::Time synchronized_time = this->get_clock()->now();
        last_published_path_.header.stamp = synchronized_time;
        pub_path_->publish(last_published_path_);
        // Also publish markers with synchronized timestamp
        publishMarkers(fixed_waypoints_);
      } else if (graph_received_ && !waiting_for_docking_completion_) {
        // Graph received but no path yet, try to plan (only if not waiting for docking)
        planAndPublishPath();
      }
    }
  }

  void controlModCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    current_control_mode_ = msg->data;
    
    // Mode 3 indicates docking/arrival completion
    if (msg->data == 3 && waiting_for_docking_completion_) {
      waiting_for_docking_completion_ = false;
      publishDockingState();
      
      LOG_INFO("Docking completed at WP[%d] - resuming path planning",
               current_target_waypoint_index_);
      
      // Check if all waypoints are completed
      if (current_target_waypoint_index_ >= 0 && 
          current_target_waypoint_index_ >= static_cast<int>(fixed_waypoints_.size()) - 1) {
        // Set exploration completed flag
        exploration_completed_ = true;
        
        LOG_INFO("Exploration Complete");
        
        // Add origin return waypoint
        geometry_msgs::msg::Point origin;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = 0.0;
        
        // Check if origin is already the last waypoint
        if (fixed_waypoints_.empty() || 
            distance(origin, fixed_waypoints_.back()) > 0.2) {
          fixed_waypoints_.push_back(origin);
          fixed_waypoint_node_indices_.push_back(-1);  // Origin is not a graph node
        }
        
        // Move to origin
        previous_waypoint_index_ = current_target_waypoint_index_;
        current_target_waypoint_index_ = static_cast<int>(fixed_waypoints_.size()) - 1;
        
        // Publish Returning status
        std_msgs::msg::String status_msg;
        status_msg.data = "Returning...";
        pub_path_status_->publish(status_msg);
        
        // Immediately generate new path after docking completion
        if (graph_received_) {
          planAndPublishPath();
        } else {
          LOG_WARN("Graph not received yet, path will be generated when graph is available");
        }
      } else if (current_target_waypoint_index_ >= 0 && 
                 current_target_waypoint_index_ < static_cast<int>(fixed_waypoints_.size()) - 1) {
        // Move to next waypoint
        previous_waypoint_index_ = current_target_waypoint_index_;
        current_target_waypoint_index_++;
        
        // Immediately generate new path after docking completion
        if (graph_received_) {
          LOG_INFO("Moving to next waypoint: WP[%d] - generating new path",
                   current_target_waypoint_index_);
          planAndPublishPath();
        } else {
          LOG_WARN("Graph not received yet, path will be generated when graph is available");
        }
      }
    }
  }

  void skeletonizedGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    skeletonized_grid_ = *msg;
  }

  void nextWaypointServiceCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    (void)request;  // Unused
    (void)response;  // Unused
    
    LOG_INFO("Next waypoint service called - forcing next waypoint");
    
    // Reset waiting_for_docking_completion flag regardless of current state
    waiting_for_docking_completion_ = false;
    publishDockingState();
    
    // Check if initial waypoint is reached
    if (!initial_waypoint_reached_) {
      LOG_WARN("Initial waypoint not reached yet, cannot move to next waypoint");
      return;
    }
    
    // Check if all waypoints are completed
    if (current_target_waypoint_index_ >= 0 && 
        current_target_waypoint_index_ >= static_cast<int>(fixed_waypoints_.size()) - 1) {
      // Set exploration completed flag
      exploration_completed_ = true;
      
      LOG_INFO("Exploration Complete (forced by service)");
      
      // Add origin return waypoint
      geometry_msgs::msg::Point origin;
      origin.x = 0.0;
      origin.y = 0.0;
      origin.z = 0.0;
      
      // Check if origin is already the last waypoint
      if (fixed_waypoints_.empty() || 
          distance(origin, fixed_waypoints_.back()) > 0.2) {
        fixed_waypoints_.push_back(origin);
        fixed_waypoint_node_indices_.push_back(-1);  // Origin is not a graph node
      }
      
      // Move to origin
      // Set previous_waypoint_index_ to current_target to mark that we're moving from current waypoint
      previous_waypoint_index_ = current_target_waypoint_index_;
      current_target_waypoint_index_ = static_cast<int>(fixed_waypoints_.size()) - 1;
      
      // Publish Returning status
      std_msgs::msg::String status_msg;
      status_msg.data = "Returning...";
      pub_path_status_->publish(status_msg);
      
      planAndPublishPath(true);  // Pass true to indicate service call - use current position
    } else if (current_target_waypoint_index_ >= 0 && 
               current_target_waypoint_index_ < static_cast<int>(fixed_waypoints_.size()) - 1) {
      // Move to next waypoint
      // Set previous_waypoint_index_ to current_target to mark that we're moving from current waypoint
      previous_waypoint_index_ = current_target_waypoint_index_;
      current_target_waypoint_index_++;
      LOG_INFO("Moving to next waypoint: WP[%d]", current_target_waypoint_index_);
      planAndPublishPath(true);  // Pass true to indicate service call - use current position
    } else if (current_target_waypoint_index_ < 0 && !fixed_waypoints_.empty()) {
      // No target set yet, start from first waypoint
      current_target_waypoint_index_ = 0;
      previous_waypoint_index_ = -1;
      LOG_INFO("Starting from first waypoint: WP[0]");
      planAndPublishPath(true);  // Pass true to indicate service call - use current position
    } else {
      LOG_WARN("Cannot move to next waypoint: no waypoints available");
    }
  }

  void graphCallback(const aos::msg::GvdGraph::SharedPtr msg) {
    // Store graph data
    graph_nodes_ = msg->nodes;
    graph_edges_ = msg->edges;
    graph_edge_lengths_ = msg->edge_lengths;
    graph_edge_clearances_ = msg->edge_clearances;
    node_labels_ = msg->node_labels;
    node_cluster_indices_ = msg->node_cluster_indices;  // deprecated, kept for backward compatibility
    resolution_ = msg->resolution;
    origin_x_ = msg->origin_x;
    origin_y_ = msg->origin_y;
    
    // Store new node label information
    node_label_clusters_ = msg->node_label_clusters;
    node_label_types_ = msg->node_label_types;
    node_label_counts_ = msg->node_label_counts;
    
    if (debug_mode_) {
      LOG_DEBUG("Graph received: %zu nodes, %zu edges", 
                  graph_nodes_.size(), graph_edges_.size() / 2);
    }

    // Build adjacency list
    adjacency_list_.clear();
    adjacency_list_.resize(msg->num_nodes);

    for (size_t i = 0; i < msg->edges.size(); i += 2) {
      if (i + 1 < msg->edges.size()) {
        int from = msg->edges[i];
        int to = msg->edges[i + 1];
        if (from >= 0 && from < static_cast<int>(adjacency_list_.size()) &&
            to >= 0 && to < static_cast<int>(adjacency_list_.size())) {
          adjacency_list_[from].push_back(to);
          adjacency_list_[to].push_back(from);
        }
      }
    }

    // Build cluster waypoint mapping (TL, TR, BL, BR detection)
    // Note: This is skipped if waiting_for_docking_completion_ is true (early return above)
    buildClusterWaypointMapping();

    // Don't rebuild waypoint sequence if exploration is completed
    bool had_origin = false;
    geometry_msgs::msg::Point saved_origin;
    if (exploration_completed_ && !fixed_waypoints_.empty()) {
      // Check if origin is at the end
      const auto& last_wp = fixed_waypoints_.back();
      if (std::abs(last_wp.x) < 1e-6 && std::abs(last_wp.y) < 1e-6) {
        had_origin = true;
        saved_origin = last_wp;
      }
    }

    // Save current target waypoint index and position before rebuilding
    int saved_target_index = current_target_waypoint_index_;
    geometry_msgs::msg::Point saved_target_position;
    bool target_position_saved = false;
    if (saved_target_index >= 0 && saved_target_index < static_cast<int>(fixed_waypoints_.size())) {
      saved_target_position = fixed_waypoints_[saved_target_index];
      target_position_saved = true;
    }

    // Rebuild waypoint sequence every time graph is updated (BR/BL/TR/TL change in real-time)
    // Don't rebuild if returning to origin after exploration completion
    if (!exploration_completed_) {
      buildWaypointSequence();
    }

    // Add origin back if returning to origin after exploration completion
    if (exploration_completed_ && had_origin) {
      if (fixed_waypoints_.empty() || 
          distance(saved_origin, fixed_waypoints_.back()) > 0.2) {
        fixed_waypoints_.push_back(saved_origin);
        fixed_waypoint_node_indices_.push_back(-1);
      }
    }

    // Restore or adjust target waypoint index based on position (not index)
    // This is important because buildWaypointSequence() may change the waypoint sequence
    if (target_position_saved && !fixed_waypoints_.empty()) {
      // Find the closest waypoint to saved_target_position
      int best_match_idx = -1;
      double min_dist = std::numeric_limits<double>::max();
      
      for (size_t i = 0; i < fixed_waypoints_.size(); ++i) {
        double dist = distance(saved_target_position, fixed_waypoints_[i]);
        if (dist < min_dist) {
          min_dist = dist;
          best_match_idx = static_cast<int>(i);
        }
      }
      
      // If found a close match (within 0.5m), use it
      if (best_match_idx >= 0 && min_dist < 0.5) {
        current_target_waypoint_index_ = best_match_idx;
        
        // Check if waypoint position changed significantly
        bool waypoint_changed = (min_dist > 0.1);
        if (waypoint_changed && debug_mode_) {
          LOG_DEBUG( 
                      "Target waypoint matched to WP[%d] (distance=%.3fm from saved position)", 
                      best_match_idx, min_dist);
        }
      } else {
        // No close match found, try to use saved index if valid
        if (saved_target_index >= 0 && saved_target_index < static_cast<int>(fixed_waypoints_.size())) {
          current_target_waypoint_index_ = saved_target_index;
        } else if (!exploration_completed_) {
          // During exploration, don't reset to 0 - keep current progress
          // Only reset if we haven't started yet
          if (current_target_waypoint_index_ < 0) {
            current_target_waypoint_index_ = 0;
          }
          // Otherwise keep current_target_waypoint_index_ as is
        } else {
          // Exploration completed, keep last index
          current_target_waypoint_index_ = static_cast<int>(fixed_waypoints_.size()) - 1;
        }
      }
    } else {
      // No saved target position, use saved index or default
      if (exploration_completed_) {
        // Don't reset index if returning to origin after exploration completion
        if (saved_target_index >= 0 && saved_target_index < static_cast<int>(fixed_waypoints_.size())) {
          current_target_waypoint_index_ = saved_target_index;
        } else if (!fixed_waypoints_.empty()) {
          // Keep last index if moving to origin
          current_target_waypoint_index_ = static_cast<int>(fixed_waypoints_.size()) - 1;
        }
      } else {
        // Adjust index only during exploration
        if (saved_target_index >= 0 && saved_target_index < static_cast<int>(fixed_waypoints_.size())) {
          current_target_waypoint_index_ = saved_target_index;
        } else if (!fixed_waypoints_.empty()) {
          // Don't reset to 0 if we already have progress
          if (current_target_waypoint_index_ < 0) {
            current_target_waypoint_index_ = 0;
          }
          // Otherwise keep current_target_waypoint_index_ as is
        }
      }
    }
    
    // Check if target waypoint position changed (for path replanning decision)
    [[maybe_unused]] bool waypoint_changed = false;
    if (target_position_saved && current_target_waypoint_index_ >= 0 && 
        current_target_waypoint_index_ < static_cast<int>(fixed_waypoints_.size())) {
      const auto& new_target = fixed_waypoints_[current_target_waypoint_index_];
      double dx = new_target.x - saved_target_position.x;
      double dy = new_target.y - saved_target_position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist > 0.1) {  // Waypoint moved more than 0.1m
        waypoint_changed = true;
      }
    }

    graph_received_ = true;

    // Plan and publish path
    planAndPublishPath();
  }
  
  // Publish docking state to notify GVD node
  void publishDockingState() {
    std_msgs::msg::Bool docking_msg;
    docking_msg.data = waiting_for_docking_completion_;
    pub_docking_state_->publish(docking_msg);
  }

  void buildWaypointSequence() {
    fixed_waypoints_.clear();
    fixed_waypoint_node_indices_.clear();

    if (cluster_waypoint_nodes_.empty()) {
      return;
    }

    // Get sorted cluster indices
    std::vector<int> cluster_indices;
    for (const auto& [idx, _] : cluster_waypoint_nodes_) {
      cluster_indices.push_back(idx);
    }
    std::sort(cluster_indices.begin(), cluster_indices.end());

    // Build waypoint sequence (temporary)
    std::vector<geometry_msgs::msg::Point> temp_waypoints;
    std::vector<int> temp_node_indices;

    int max_cluster_idx = cluster_indices.empty() ? -1 : cluster_indices.back();
    bool is_last_column_odd = (max_cluster_idx >= 0 && max_cluster_idx % 2 == 1);

    for (size_t i = 0; i < cluster_indices.size(); ++i) {
      int cluster_idx = cluster_indices[i];
      bool is_last_cluster = (i == cluster_indices.size() - 1);
      const auto& waypoint_nodes = cluster_waypoint_nodes_[cluster_idx];

      if (cluster_idx % 2 == 0) {
        // Even cluster: BR -> BL
        int br_idx = waypoint_nodes[3];  // BR
        int bl_idx = waypoint_nodes[2];  // BL

        if (br_idx >= 0 && br_idx < static_cast<int>(graph_nodes_.size())) {
          temp_waypoints.push_back(graph_nodes_[br_idx]);
          temp_node_indices.push_back(br_idx);
        }

        if (bl_idx >= 0 && bl_idx < static_cast<int>(graph_nodes_.size())) {
          temp_waypoints.push_back(graph_nodes_[bl_idx]);
          temp_node_indices.push_back(bl_idx);
        }
        
        // Add TR after BL if last column is even
        if (is_last_cluster && !is_last_column_odd) {
          int tr_idx = waypoint_nodes[1];  // TR
          if (tr_idx >= 0 && tr_idx < static_cast<int>(graph_nodes_.size())) {
            temp_waypoints.push_back(graph_nodes_[tr_idx]);
            temp_node_indices.push_back(tr_idx);
          }
        }
      } else {
        // Odd cluster: TL -> TR
        int tl_idx = waypoint_nodes[0];  // TL
        int tr_idx = waypoint_nodes[1];  // TR

        if (tl_idx >= 0 && tl_idx < static_cast<int>(graph_nodes_.size())) {
          temp_waypoints.push_back(graph_nodes_[tl_idx]);
          temp_node_indices.push_back(tl_idx);
        }

        if (tr_idx >= 0 && tr_idx < static_cast<int>(graph_nodes_.size())) {
          temp_waypoints.push_back(graph_nodes_[tr_idx]);
          temp_node_indices.push_back(tr_idx);
        }

        // Add BL after TR if last column is odd
        if (is_last_cluster && is_last_column_odd) {
          int bl_idx = waypoint_nodes[2];  // BL
          if (bl_idx >= 0 && bl_idx < static_cast<int>(graph_nodes_.size())) {
            temp_waypoints.push_back(graph_nodes_[bl_idx]);
            temp_node_indices.push_back(bl_idx);
          }
        }
      }
    }

    // Filter out waypoints that are too close (distance <= 0.2m)
    const double min_waypoint_distance = 0.2;
    if (!temp_waypoints.empty()) {
      fixed_waypoints_.push_back(temp_waypoints[0]);
      fixed_waypoint_node_indices_.push_back(temp_node_indices[0]);
      if (debug_mode_) {
        LOG_DEBUG("WP[0]: node_idx=%d, pos=(%.3f, %.3f)", 
                    temp_node_indices[0], temp_waypoints[0].x, temp_waypoints[0].y);
      }
      
      for (size_t i = 1; i < temp_waypoints.size(); ++i) {
        double dist = distance(temp_waypoints[i], fixed_waypoints_.back());
        if (dist > min_waypoint_distance) {
          fixed_waypoints_.push_back(temp_waypoints[i]);
          fixed_waypoint_node_indices_.push_back(temp_node_indices[i]);
          if (debug_mode_) {
            LOG_DEBUG( "WP[%zu]: node_idx=%d, pos=(%.3f, %.3f), dist_from_prev=%.3f -> added", 
                        fixed_waypoints_.size() - 1, temp_node_indices[i], 
                        temp_waypoints[i].x, temp_waypoints[i].y, dist);
          }
        } else {
          if (debug_mode_) {
            LOG_DEBUG( 
                       "WP[%zu] FILTERED OUT: node_idx=%d, pos=(%.3f, %.3f), dist_from_prev=%.3f (too close, <=%.2fm). "
                       "This may cause direct straight-line path instead of graph path!",
                       i, temp_node_indices[i], temp_waypoints[i].x, temp_waypoints[i].y, 
                       dist, min_waypoint_distance);
          }
        }
      }
      
      if (debug_mode_) {
            LOG_DEBUG( "Waypoint sequence: %zu waypoints (filtered from %zu temp waypoints)", 
                    fixed_waypoints_.size(), temp_waypoints.size());
      }
    }

    // Origin return waypoint is added later when all waypoints are completed
  }

  void buildClusterWaypointMapping() {
    cluster_waypoint_nodes_.clear();

    // Map: cluster_index -> {TL, TR, BL, BR} node indices
    // Use new message structure: node_label_clusters, node_label_types, node_label_counts
    // label_type: 0=TL, 1=TR, 2=BL, 3=BR
    
    if (node_label_counts_.empty() || node_label_clusters_.empty() || node_label_types_.empty()) {
      // Fallback to old method for backward compatibility
      for (size_t i = 0; i < node_labels_.size(); ++i) {
        int label_mask = node_labels_[i];
        int cluster_idx = node_cluster_indices_[i];

        if (cluster_idx >= 0 && label_mask > 0) {
          if (cluster_waypoint_nodes_.find(cluster_idx) == cluster_waypoint_nodes_.end()) {
            cluster_waypoint_nodes_[cluster_idx] = std::vector<int>(4, -1);
          }
          
          // Extract labels from bitmask
          if (label_mask & (1 << 0)) {  // TL
            cluster_waypoint_nodes_[cluster_idx][0] = static_cast<int>(i);
          }
          if (label_mask & (1 << 1)) {  // TR
            cluster_waypoint_nodes_[cluster_idx][1] = static_cast<int>(i);
          }
          if (label_mask & (1 << 2)) {  // BL
            cluster_waypoint_nodes_[cluster_idx][2] = static_cast<int>(i);
          }
          if (label_mask & (1 << 3)) {  // BR
            cluster_waypoint_nodes_[cluster_idx][3] = static_cast<int>(i);
          }
        }
      }
    } else {
      // Use new message structure
      int label_data_idx = 0;
      for (size_t i = 0; i < node_label_counts_.size(); ++i) {
        int label_count = node_label_counts_[i];
        
        for (int j = 0; j < label_count; ++j) {
          if (label_data_idx + j < static_cast<int>(node_label_clusters_.size()) &&
              label_data_idx + j < static_cast<int>(node_label_types_.size())) {
            int cluster_idx = node_label_clusters_[label_data_idx + j];
            int label_type = node_label_types_[label_data_idx + j];
            
            if (cluster_idx >= 0 && label_type >= 0 && label_type <= 3) {
              if (cluster_waypoint_nodes_.find(cluster_idx) == cluster_waypoint_nodes_.end()) {
                cluster_waypoint_nodes_[cluster_idx] = std::vector<int>(4, -1);
              }
              
              // label_type: 0=TL, 1=TR, 2=BL, 3=BR
              // Same label from same cluster can exist on multiple nodes, use first found node
              if (cluster_waypoint_nodes_[cluster_idx][label_type] < 0) {
                cluster_waypoint_nodes_[cluster_idx][label_type] = static_cast<int>(i);
              }
            }
          }
        }
        label_data_idx += label_count;
      }
    }
  }

  //--------------------------------------------------------------------------------
  // Path Planning
  //--------------------------------------------------------------------------------
  
  struct NodeCost {
    int node_idx;
    double g_cost;
    double f_cost;
    
    bool operator>(const NodeCost& other) const {
      return f_cost > other.f_cost;
    }
  };

  double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  // A* heuristic function: straight-line distance to goal (with weight applied)
  static constexpr double DEFAULT_HEURISTIC_WEIGHT = 3.0;
  double heuristic(int node_idx, int goal_node_idx, double heuristic_weight = DEFAULT_HEURISTIC_WEIGHT) {
    if (node_idx < 0 || node_idx >= static_cast<int>(graph_nodes_.size()) ||
        goal_node_idx < 0 || goal_node_idx >= static_cast<int>(graph_nodes_.size())) {
      return std::numeric_limits<double>::max();
    }
    double h = distance(graph_nodes_[node_idx], graph_nodes_[goal_node_idx]);
    // Multiply heuristic by weight to prefer more direct paths toward goal
    return h * heuristic_weight;
  }

  // A* algorithm: applies high weight to direct paths toward goal
  std::vector<int> astar(int start_node, int goal_node) {
    if (start_node < 0 || start_node >= static_cast<int>(graph_nodes_.size()) ||
        goal_node < 0 || goal_node >= static_cast<int>(graph_nodes_.size())) {
      LOG_WARN("A*: Invalid node indices: start=%d (size=%zu), goal=%d (size=%zu)", 
                  start_node, graph_nodes_.size(), goal_node, graph_nodes_.size());
      return {};
    }

    if (start_node == goal_node) {
      LOG_WARN("A*: Start and goal are the same node %d", start_node);
      return {start_node};
    }
    
    if (adjacency_list_[start_node].empty()) {
      LOG_WARN("A*: Start node %d has no neighbors", start_node);
      return {};
    }
    
    if (adjacency_list_[goal_node].empty()) {
      LOG_WARN("A*: Goal node %d has no neighbors", goal_node);
      return {};
    }

    // A* algorithm parameter: heuristic weight (higher values prefer more direct paths toward goal)
    static constexpr double HEURISTIC_WEIGHT = 3.0;  // High weight for direct paths toward goal

    std::priority_queue<NodeCost, std::vector<NodeCost>, std::greater<NodeCost>> pq;
    std::vector<double> g_costs(graph_nodes_.size(), std::numeric_limits<double>::max());
    std::vector<int> parents(graph_nodes_.size(), -1);
    std::unordered_set<int> visited;

    g_costs[start_node] = 0.0;
    double h_start = heuristic(start_node, goal_node, HEURISTIC_WEIGHT);
    pq.push({start_node, 0.0, h_start});

    while (!pq.empty()) {
      NodeCost current = pq.top();
      pq.pop();

      if (visited.find(current.node_idx) != visited.end()) {
        continue;
      }

      visited.insert(current.node_idx);

      if (current.node_idx == goal_node) {
        // Reconstruct path
        std::vector<int> path;
        int node = goal_node;
        while (node != -1) {
          path.push_back(node);
          node = parents[node];
        }
        std::reverse(path.begin(), path.end());
        return path;
      }

      for (int neighbor : adjacency_list_[current.node_idx]) {
        if (visited.find(neighbor) != visited.end()) {
          continue;
        }

        // Find edge cost
        double edge_cost = std::numeric_limits<double>::max();
        for (size_t i = 0; i < graph_edges_.size(); i += 2) {
          if (i + 1 < graph_edges_.size()) {
            int from = graph_edges_[i];
            int to = graph_edges_[i + 1];
            if ((from == current.node_idx && to == neighbor) ||
                (from == neighbor && to == current.node_idx)) {
              size_t edge_idx = i / 2;
              if (edge_idx < graph_edge_lengths_.size()) {
                edge_cost = graph_edge_lengths_[edge_idx];
              } else {
                edge_cost = distance(graph_nodes_[current.node_idx], graph_nodes_[neighbor]);
              }
              break;
            }
          }
        }

        double new_g_cost = g_costs[current.node_idx] + edge_cost;
        if (new_g_cost < g_costs[neighbor]) {
          g_costs[neighbor] = new_g_cost;
          parents[neighbor] = current.node_idx;
          // A*: f(n) = g(n) + w * h(n), where w > 1.0 to prefer direct paths to goal
          double h_cost = heuristic(neighbor, goal_node, HEURISTIC_WEIGHT);
          double f_cost = new_g_cost + h_cost;
          pq.push({neighbor, new_g_cost, f_cost});
        }
      }
    }

    LOG_WARN("A*: No path found from node %d to node %d (visited %zu nodes)", 
                start_node, goal_node, visited.size());
    return {};  // No path found
  }

  int findNearestNode(const geometry_msgs::msg::Point& point) {
    int nearest_idx = -1;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < graph_nodes_.size(); ++i) {
      double dist = distance(point, graph_nodes_[i]);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_idx = static_cast<int>(i);
      }
    }

    return nearest_idx;
  }

  // Find k nearest nodes to a point
  std::vector<int> findKNearestNodes(const geometry_msgs::msg::Point& point, int k = 5) {
    std::vector<std::pair<double, int>> node_distances;
    
    for (size_t i = 0; i < graph_nodes_.size(); ++i) {
      double dist = distance(point, graph_nodes_[i]);
      node_distances.push_back({dist, static_cast<int>(i)});
    }
    
    // Sort by distance
    std::sort(node_distances.begin(), node_distances.end());
    
    // Return k nearest node indices
    std::vector<int> nearest_nodes;
    for (size_t i = 0; i < std::min(static_cast<size_t>(k), node_distances.size()); ++i) {
      nearest_nodes.push_back(node_distances[i].second);
    }
    
    return nearest_nodes;
  }

  // Calculate total path cost (sum of edge distances)
  double calculatePathCost(const std::vector<int>& node_path) {
    if (node_path.size() < 2) {
      return 0.0;
    }
    
    double total_cost = 0.0;
    for (size_t i = 0; i < node_path.size() - 1; ++i) {
      int from = node_path[i];
      int to = node_path[i + 1];
      
      // Find edge cost
      double edge_cost = std::numeric_limits<double>::max();
      for (size_t j = 0; j < graph_edges_.size(); j += 2) {
        if (j + 1 < graph_edges_.size()) {
          int edge_from = graph_edges_[j];
          int edge_to = graph_edges_[j + 1];
          if ((edge_from == from && edge_to == to) ||
              (edge_from == to && edge_to == from)) {
            size_t edge_idx = j / 2;
            if (edge_idx < graph_edge_lengths_.size()) {
              edge_cost = graph_edge_lengths_[edge_idx];
            } else {
              edge_cost = distance(graph_nodes_[from], graph_nodes_[to]);
            }
            break;
          }
        }
      }
      
      if (edge_cost == std::numeric_limits<double>::max()) {
        // Edge not found, use euclidean distance
        edge_cost = distance(graph_nodes_[from], graph_nodes_[to]);
      }
      
      total_cost += edge_cost;
    }
    
    return total_cost;
  }


  void planAndPublishPath(bool use_current_position = false) {
    // Build path by connecting start to target through graph
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    // Timestamp will be synchronized when publishing

    // If initial waypoint not reached, plan straight line from (0,0) to (8,0)
    if (!initial_waypoint_reached_) {
      geometry_msgs::msg::Point start_pos;
      start_pos.x = 0.0;
      start_pos.y = 0.0;
      start_pos.z = 0.0;

      // Generate straight line path from (0,0) to (8,0)
      double dx = initial_waypoint_.x - start_pos.x;
      double dy = initial_waypoint_.y - start_pos.y;
      double total_distance = std::sqrt(dx * dx + dy * dy);
      const double step_size = 0.2;
      int num_steps = static_cast<int>(std::ceil(total_distance / step_size));

      for (int i = 0; i <= num_steps; i++) {
        double t = static_cast<double>(i) / num_steps;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = start_pos.x + t * dx;
        pose.pose.position.y = start_pos.y + t * dy;
        pose.pose.position.z = 0.0;
        
        // Calculate orientation (always pointing towards (8,0))
        double yaw = std::atan2(dy, dx);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        
        path.poses.push_back(pose);
      }

      // Ensure goal is exactly at the end
      if (!path.poses.empty()) {
        path.poses.back().pose.position = initial_waypoint_;
      }

      // Trim path if it gets too close to occupied regions
      trimPathNearOccupiedRegions(path);

      last_published_path_ = path;
      // Synchronize path and marker timestamps
      rclcpp::Time synchronized_time = this->get_clock()->now();
      path.header.stamp = synchronized_time;
      pub_path_->publish(path);
      
      // Publish visualization markers with synchronized timestamp
      publishMarkers(fixed_waypoints_);
      // Publish status (initial waypoint path is always successful)
      publishPlanningStatus(true);
      return;
    }

    // After initial waypoint reached, plan to waypoints
    if (!graph_received_ || fixed_waypoints_.empty()) {
      // Publish last path if available
      if (!last_published_path_.poses.empty()) {
        last_published_path_.header.stamp = this->get_clock()->now();
        pub_path_->publish(last_published_path_);
      }
      publishMarkers(fixed_waypoints_);
      // Publish status (no waypoints available)
      publishPlanningStatus(false);
      return;
    }

    if (current_target_waypoint_index_ < 0 || 
        current_target_waypoint_index_ >= static_cast<int>(fixed_waypoints_.size())) {
      // Publish last path if available
      if (!last_published_path_.poses.empty()) {
        last_published_path_.header.stamp = this->get_clock()->now();
        pub_path_->publish(last_published_path_);
      }
      publishMarkers(fixed_waypoints_);
      // Publish status (invalid waypoint index)
      publishPlanningStatus(false);
      return;
    }

    // Get start position: use current position if service call, otherwise use previous waypoint or initial waypoint
    geometry_msgs::msg::Point start_point;
    int start_node_idx = -1;
    if (use_current_position && current_position_received_) {
      // Service call: use current robot position as start point
      start_point = current_position_;
      LOG_INFO("Service call: Starting from current position (%.3f, %.3f)", 
                  start_point.x, start_point.y);
    } else if (previous_waypoint_index_ >= 0 && 
        previous_waypoint_index_ < static_cast<int>(fixed_waypoints_.size())) {
      // Start from previous waypoint (just arrived)
      start_point = fixed_waypoints_[previous_waypoint_index_];
      if (previous_waypoint_index_ < static_cast<int>(fixed_waypoint_node_indices_.size())) {
        start_node_idx = fixed_waypoint_node_indices_[previous_waypoint_index_];
      }
      // RCLCPP_INFO(this->get_logger(), "Start from previous WP[%d]: pos=(%.3f, %.3f), node_idx=%d", 
      //             previous_waypoint_index_, start_point.x, start_point.y, start_node_idx);
    } else {
      // Start from initial waypoint (8, 0)
      start_point = initial_waypoint_;
      // RCLCPP_INFO(this->get_logger(), "Start from initial waypoint: pos=(%.3f, %.3f)", 
      //             start_point.x, start_point.y);
    }

    // Get target waypoint
    const auto& target = fixed_waypoints_[current_target_waypoint_index_];
    int target_node = fixed_waypoint_node_indices_[current_target_waypoint_index_];
    // RCLCPP_INFO(this->get_logger(), "Target WP[%d]: pos=(%.3f, %.3f), node_idx=%d", 
    //             current_target_waypoint_index_, target.x, target.y, target_node);

    // Build path by connecting start to target through graph
    // Note: path variable is already declared at function start
    path.header.frame_id = "map";
    // Timestamp will be synchronized when publishing
    path.poses.clear();  // Clear any previous poses

    // Origin return case (target_node == -1)
    if (target_node < 0) {
      // Find nearest graph node to origin and plan path through graph
      int nearest_to_origin = findNearestNode(target);
      
      if (nearest_to_origin < 0 || nearest_to_origin >= static_cast<int>(graph_nodes_.size())) {
        LOG_ERROR( 
                     "Graph planning failed: Cannot find nearest graph node to origin (%.2f, %.2f). Graph node count: %zu",
                     target.x, target.y, graph_nodes_.size());
        // Return without creating path
        if (!last_published_path_.poses.empty()) {
          last_published_path_.header.stamp = this->get_clock()->now();
          pub_path_->publish(last_published_path_);
        }
        publishMarkers(fixed_waypoints_);
        publishPlanningStatus(false);
        return;
      }

      // Plan path from start point to nearest graph node to origin
      std::vector<int> candidate_start_nodes = findKNearestNodes(start_point, 5);
      
      if (candidate_start_nodes.empty()) {
        LOG_ERROR( 
                     "Graph planning failed: Cannot find graph nodes near start point (%.2f, %.2f). Graph node count: %zu",
                     start_point.x, start_point.y, graph_nodes_.size());
        // Return without creating path
        if (!last_published_path_.poses.empty()) {
          last_published_path_.header.stamp = this->get_clock()->now();
          pub_path_->publish(last_published_path_);
        }
        publishMarkers(fixed_waypoints_);
        publishPlanningStatus(false);
        return;
      }

      // Plan path from each candidate start node to nearest node to origin
      std::vector<int> best_path;
      double min_total_cost = std::numeric_limits<double>::max();
      bool found_path = false;

      for (int candidate_start : candidate_start_nodes) {
        if (candidate_start == nearest_to_origin) {
          continue;
        }
        
        std::vector<int> node_path = astar(candidate_start, nearest_to_origin);
        
        if (!node_path.empty() && node_path.size() > 1) {
          found_path = true;
          double path_cost = calculatePathCost(node_path);
          double dist_to_candidate = distance(start_point, graph_nodes_[candidate_start]);
          double total_cost = dist_to_candidate + path_cost;
          
          if (total_cost < min_total_cost) {
            min_total_cost = total_cost;
            best_path = node_path;
          }
        }
      }

      if (!found_path || best_path.empty()) {
        LOG_ERROR( 
                     "Graph planning failed: Cannot find path through graph from start (%.2f, %.2f) to origin (%.2f, %.2f). "
                     "Tried %zu candidate start nodes, target node: %d",
                     start_point.x, start_point.y, target.x, target.y, 
                     candidate_start_nodes.size(), nearest_to_origin);
        // Return without creating path
        if (!last_published_path_.poses.empty()) {
          last_published_path_.header.stamp = this->get_clock()->now();
          pub_path_->publish(last_published_path_);
        }
        publishMarkers(fixed_waypoints_);
        publishPlanningStatus(false);
        return;
      }

      // Add start point if different from first node in best_path
      bool start_point_added = false;
      if (!best_path.empty() && best_path[0] >= 0 && 
          best_path[0] < static_cast<int>(graph_nodes_.size())) {
        const auto& first_node_pos = graph_nodes_[best_path[0]];
        double dist_to_first = distance(start_point, first_node_pos);
        if (dist_to_first > 0.1) {  // Add start point if more than 10cm away from first node
          geometry_msgs::msg::PoseStamped start_pose;
          start_pose.header = path.header;
          start_pose.pose.position = start_point;
          start_pose.pose.orientation.w = 1.0;
          path.poses.push_back(start_pose);
          start_point_added = true;
        }
      } else {
        // Add start point if best_path is empty or first node is invalid
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header = path.header;
        start_pose.pose.position = start_point;
        start_pose.pose.orientation.w = 1.0;
        path.poses.push_back(start_pose);
        start_point_added = true;
      }

      // Add path through graph (include all nodes to ensure graph traversal)
      for (size_t j = 0; j < best_path.size(); ++j) {
        int node_idx = best_path[j];
        if (node_idx < 0 || node_idx >= static_cast<int>(graph_nodes_.size())) {
          LOG_WARN("Invalid node index %d found in origin return path", node_idx);
          continue;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position = graph_nodes_[node_idx];
        pose.pose.orientation.w = 1.0;

        double dist_to_prev = 0.0;
        if (!path.poses.empty()) {
          dist_to_prev = distance(path.poses.back().pose.position, pose.pose.position);
        }

        // Always add first node if start point was not added
        if (path.poses.empty()) {
          path.poses.push_back(pose);
        }
        // Add all nodes more than 1mm apart
        else if (dist_to_prev > 0.001) {
          path.poses.push_back(pose);
        } else if (dist_to_prev > 0.0) {
          // Also add if distance is non-zero but less than 1mm
          path.poses.push_back(pose);
        }
      }

      // Connect from nearest node to origin with straight line
      if (!path.poses.empty()) {
        const auto& last_node_pos = path.poses.back().pose.position;
        double dx = target.x - last_node_pos.x;
        double dy = target.y - last_node_pos.y;
        double total_distance = std::sqrt(dx * dx + dy * dy);
        const double step_size = 0.2;
        int num_steps = static_cast<int>(std::ceil(total_distance / step_size));

        for (int i = 1; i <= num_steps; i++) {
          double t = static_cast<double>(i) / num_steps;
          geometry_msgs::msg::PoseStamped pose;
          pose.header = path.header;
          pose.pose.position.x = last_node_pos.x + t * dx;
          pose.pose.position.y = last_node_pos.y + t * dy;
          pose.pose.position.z = 0.0;
          
          double yaw = std::atan2(dy, dx);
          pose.pose.orientation.w = std::cos(yaw / 2.0);
          pose.pose.orientation.z = std::sin(yaw / 2.0);
          
          path.poses.push_back(pose);
        }
      }

      // Ensure goal is exactly at the end
      if (!path.poses.empty()) {
        path.poses.back().pose.position = target;
      }

      // Calculate orientations for all poses
      for (size_t i = 0; i < path.poses.size(); ++i) {
        if (i + 1 < path.poses.size()) {
          double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
          double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
          double yaw = std::atan2(dy, dx);
          
          path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
          path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
        }
      }

      // Trim path if it gets too close to occupied regions
      trimPathNearOccupiedRegions(path);

      last_published_path_ = path;
      // Synchronize path and marker timestamps
      rclcpp::Time synchronized_time = this->get_clock()->now();
      path.header.stamp = synchronized_time;
      pub_path_->publish(path);
      publishMarkers(fixed_waypoints_);
      publishPlanningStatus(true);
      return;
    }

    // Find k nearest nodes to current position
    std::vector<int> candidate_start_nodes = findKNearestNodes(start_point, 5);
    
    if (candidate_start_nodes.empty()) {
      LOG_ERROR( 
                   "Graph planning failed: Cannot find graph nodes near start point (%.2f, %.2f). Graph node count: %zu",
                   start_point.x, start_point.y, graph_nodes_.size());
      // Return without creating path
      if (!last_published_path_.poses.empty()) {
        last_published_path_.header.stamp = this->get_clock()->now();
        pub_path_->publish(last_published_path_);
      }
      publishMarkers(fixed_waypoints_);
      publishPlanningStatus(false);
      return;
    }

    // Validate target node
    if (target_node < 0 || target_node >= static_cast<int>(graph_nodes_.size())) {
      LOG_ERROR( 
                   "Graph planning failed: Invalid target node index. Target node: %d, graph node count: %zu, "
                   "target WP index: %d, target position: (%.2f, %.2f)",
                   target_node, graph_nodes_.size(), current_target_waypoint_index_, target.x, target.y);
      // Return without creating path
      if (!last_published_path_.poses.empty()) {
        last_published_path_.header.stamp = this->get_clock()->now();
        pub_path_->publish(last_published_path_);
      }
      publishMarkers(fixed_waypoints_);
      publishPlanningStatus(false);
      return;
    }

    // Try each candidate start node and find the path with minimum cost
    std::vector<int> best_path;
    double min_total_cost = std::numeric_limits<double>::max();
    bool found_path = false;

    // RCLCPP_INFO(this->get_logger(), "Trying %zu candidate start nodes to reach target_node=%d", 
    //             candidate_start_nodes.size(), target_node);
    
    for (int candidate_start : candidate_start_nodes) {
      // RCLCPP_INFO(this->get_logger(), "  Candidate start node %d at (%.3f, %.3f), adjacency_list size=%zu", 
      //             candidate_start, graph_nodes_[candidate_start].x, graph_nodes_[candidate_start].y,
      //             adjacency_list_[candidate_start].size());
      
      // Skip if candidate start node is the same as target node (this would create a direct line)
      if (candidate_start == target_node) {
        LOG_WARN("  Skipping candidate_start=%d (same as target_node=%d) - would create direct line path", 
                    candidate_start, target_node);
        continue;
      }
      
      std::vector<int> node_path = astar(candidate_start, target_node);
      
      // RCLCPP_INFO(this->get_logger(), "  A* result: path size=%zu", node_path.size());
      if (!node_path.empty() && node_path.size() > 1) {
        // RCLCPP_INFO(this->get_logger(), "  Path: %d -> ... -> %d (%zu nodes)", 
        //             node_path[0], node_path.back(), node_path.size());
      } else if (!node_path.empty()) {
        LOG_WARN("  Path has only 1 node (start=%d, target=%d, same node?)", 
                    candidate_start, target_node);
        // Skip paths with only 1 node (same start and goal)
        continue;
      }
      
      if (!node_path.empty() && node_path.size() > 1) {
        found_path = true;
        // Calculate total path cost
        double path_cost = calculatePathCost(node_path);
        
        // Also add distance from start point to candidate node
        double dist_to_candidate = distance(start_point, graph_nodes_[candidate_start]);
        double total_cost = dist_to_candidate + path_cost;
        
        // RCLCPP_INFO(this->get_logger(), "  Path cost=%.3f, dist_to_candidate=%.3f, total_cost=%.3f", 
        //             path_cost, dist_to_candidate, total_cost);
        
        if (total_cost < min_total_cost) {
          min_total_cost = total_cost;
          best_path = node_path;
          // RCLCPP_INFO(this->get_logger(), "  -> New best path selected");
        }
      } else {
        LOG_WARN("  No valid path found from candidate_start=%d to target_node=%d (path size=%zu)", 
                    candidate_start, target_node, node_path.size());
      }
    }

    if (!found_path || best_path.empty()) {
      LOG_ERROR( 
                   "Graph planning failed: Cannot find path through graph from start (%.2f, %.2f) to target WP[%d] (%.2f, %.2f). "
                   "Tried %zu candidate start nodes, target node: %d, graph node count: %zu, edge count: %zu",
                   start_point.x, start_point.y, current_target_waypoint_index_, target.x, target.y,
                   candidate_start_nodes.size(), target_node, graph_nodes_.size(), graph_edges_.size() / 2);
      // Return without creating path
      if (!last_published_path_.poses.empty()) {
        last_published_path_.header.stamp = this->get_clock()->now();
        pub_path_->publish(last_published_path_);
      }
      publishMarkers(fixed_waypoints_);
      // Publish failed status
      publishPlanningStatus(false);
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
    //               start_point.x, start_point.y, target.x, target.y);
    // RCLCPP_INFO(this->get_logger(), "Best path has %zu nodes, target_node=%d, graph_nodes=%zu, graph_edges=%zu",
    //               best_path.size(), target_node, graph_nodes_.size(), graph_edges_.size() / 2);

    // Add path through graph nodes
    // Always include ALL nodes in the path to ensure graph traversal
    size_t valid_nodes_added = 0;
    
    // Add start point if different from first node in best_path
    bool start_point_added = false;
    if (!best_path.empty() && best_path[0] >= 0 && 
        best_path[0] < static_cast<int>(graph_nodes_.size())) {
      const auto& first_node_pos = graph_nodes_[best_path[0]];
      double dist_to_first = distance(start_point, first_node_pos);
      if (dist_to_first > 0.1) {  // Add start point if more than 10cm away from first node
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header = path.header;
        start_pose.pose.position = start_point;
        start_pose.pose.orientation.w = 1.0;
        path.poses.push_back(start_pose);
        start_point_added = true;
      }
    } else {
      // Add start point if best_path is empty or first node is invalid
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.header = path.header;
      start_pose.pose.position = start_point;
      start_pose.pose.orientation.w = 1.0;
      path.poses.push_back(start_pose);
      start_point_added = true;
    }
    
    // Add all nodes from best_path to path (include all nodes to ensure graph traversal)
    for (size_t j = 0; j < best_path.size(); ++j) {
      int node_idx = best_path[j];
      if (node_idx < 0 || node_idx >= static_cast<int>(graph_nodes_.size())) {
        LOG_WARN("Invalid node index %d found in path (range: 0-%zu)", 
                    node_idx, graph_nodes_.size());
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = graph_nodes_[node_idx];
      pose.pose.orientation.w = 1.0;

      // Check distance to previous point
      double dist_to_prev = 0.0;
      if (!path.poses.empty()) {
        dist_to_prev = distance(path.poses.back().pose.position, pose.pose.position);
      }

      // Always add first node if start point was not added
      if (path.poses.empty()) {
        path.poses.push_back(pose);
        valid_nodes_added++;
      }
      // Add all nodes more than 1mm apart
      else if (dist_to_prev > 0.001) {
        path.poses.push_back(pose);
        valid_nodes_added++;
      } else if (dist_to_prev > 0.0) {
        // Also add if distance is non-zero but less than 1mm (accounting for floating point error)
        path.poses.push_back(pose);
        valid_nodes_added++;
      }
    }

    // Consider path generation failed if no graph nodes included
    if (valid_nodes_added == 0 && !start_point_added) {
      LOG_ERROR( 
                   "Graph planning failed: No valid graph nodes included in path. "
                   "best_path size: %zu, start: (%.2f, %.2f), target: (%.2f, %.2f)",
                   best_path.size(), start_point.x, start_point.y, target.x, target.y);
      // Return without creating path
      if (!last_published_path_.poses.empty()) {
        last_published_path_.header.stamp = this->get_clock()->now();
        pub_path_->publish(last_published_path_);
      }
      publishMarkers(fixed_waypoints_);
      publishPlanningStatus(false);
      return;
    }
    
    // Error if path is empty
    if (path.poses.empty()) {
      LOG_ERROR( 
                   "Graph planning failed: Path is empty. "
                   "best_path size: %zu, start: (%.2f, %.2f), target: (%.2f, %.2f)",
                   best_path.size(), start_point.x, start_point.y, target.x, target.y);
      if (!last_published_path_.poses.empty()) {
        last_published_path_.header.stamp = this->get_clock()->now();
        pub_path_->publish(last_published_path_);
      }
      publishMarkers(fixed_waypoints_);
      publishPlanningStatus(false);
      return;
    }

    // Always ensure target is at the end, regardless of distance to last node
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header = path.header;
    target_pose.pose.position = target;
    target_pose.pose.orientation.w = 1.0;

    if (!path.poses.empty()) {
      double dist_to_last = distance(path.poses.back().pose.position, target);
      if (dist_to_last > 0.01) {
        path.poses.push_back(target_pose);
            LOG_DEBUG( "Added target point at (%.2f, %.2f)", target.x, target.y);
      } else {
        // Update last pose to exact target position
        path.poses.back().pose.position = target;
            LOG_DEBUG( "Updated last pose to target position");
      }
    }

    if (path.poses.empty()) {
      // Publish last path if available
      if (!last_published_path_.poses.empty()) {
        last_published_path_.header.stamp = this->get_clock()->now();
        pub_path_->publish(last_published_path_);
      }
      // Publish visualization markers even if path is empty
      publishMarkers(fixed_waypoints_);
      return;
    }

    // Calculate orientations: last point faces next waypoint
    double last_yaw = 0.0;
    if (current_target_waypoint_index_ < static_cast<int>(fixed_waypoints_.size()) - 1) {
      const auto& next_target = fixed_waypoints_[current_target_waypoint_index_ + 1];
      const auto& last_point = path.poses.back().pose.position;
      double dx = next_target.x - last_point.x;
      double dy = next_target.y - last_point.y;
      last_yaw = std::atan2(dy, dx);
    } else {
      // Last waypoint: use direction from previous point
      if (path.poses.size() > 1) {
        const auto& prev_point = path.poses[path.poses.size() - 2].pose.position;
        const auto& last_point = path.poses.back().pose.position;
        double dx = last_point.x - prev_point.x;
        double dy = last_point.y - prev_point.y;
        last_yaw = std::atan2(dy, dx);
      }
    }

    for (size_t i = 0; i < path.poses.size(); ++i) {
      if (i + 1 < path.poses.size()) {
        double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
        double yaw = std::atan2(dy, dx);
        
        path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
        path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
      } else {
        // Last pose: face next waypoint (or previous direction if last waypoint)
        path.poses[i].pose.orientation.w = std::cos(last_yaw / 2.0);
        path.poses[i].pose.orientation.z = std::sin(last_yaw / 2.0);
      }
    }

    // Trim path if it gets too close to occupied regions
    trimPathNearOccupiedRegions(path);

    // Save path before publishing
    last_published_path_ = path;
    
    // Synchronize path and marker timestamps for proper visualization
    rclcpp::Time synchronized_time = this->get_clock()->now();
    path.header.stamp = synchronized_time;
    pub_path_->publish(path);

    // Publish visualization markers with synchronized timestamp
    publishMarkers(fixed_waypoints_);
    
    // Publish planning status
    publishPlanningStatus(true);
  }

  // Trim path if it gets too close to occupied regions (within 0.2m)
  void trimPathNearOccupiedRegions(nav_msgs::msg::Path& path) {
    if (!skeletonized_grid_ || path.poses.empty()) {
      return;
    }

    const double safety_distance = 0.2;  // 0.2m
    const auto& grid = *skeletonized_grid_;
    const double resolution = grid.info.resolution;
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;

    // Check each point in the path
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto& point = path.poses[i].pose.position;
      
      // Check if point is within safety_distance of any occupied cell
      bool too_close = false;
      
      // Check surrounding cells within safety_distance
      int check_radius_cells = static_cast<int>(std::ceil(safety_distance / resolution));
      
      for (int dx = -check_radius_cells; dx <= check_radius_cells && !too_close; ++dx) {
        for (int dy = -check_radius_cells; dy <= check_radius_cells && !too_close; ++dy) {
          double check_x = point.x + dx * resolution;
          double check_y = point.y + dy * resolution;
          
          // Calculate distance from point to check cell center
          double dist = std::sqrt(dx * dx + dy * dy) * resolution;
          if (dist > safety_distance) {
            continue;
          }
          
          // Convert to grid coordinates
          int mx = static_cast<int>((check_x - origin_x) / resolution);
          int my = static_cast<int>((check_y - origin_y) / resolution);
          
          if (mx >= 0 && mx < width && my >= 0 && my < height) {
            int index = mx + my * width;
            if (index >= 0 && index < static_cast<int>(grid.data.size())) {
              // Check if cell is occupied (value == 100)
              if (grid.data[index] == 100) {
                too_close = true;
                break;
              }
            }
          }
        }
      }
      
      // If point is too close to occupied region, trim path up to previous point
      if (too_close && i > 0) {
        path.poses.resize(i);
        RCLCPP_WARN(this->get_logger(), 
                   "Path trimmed at index %zu (point %.2f, %.2f) - too close to occupied region (within %.1fm)", 
                   i, point.x, point.y, safety_distance);
        break;
      }
    }
  }
  
  // Helper function to calculate cluster index from waypoint index
  int calculateClusterIndex(int waypoint_index, int total_clusters) {
    if (waypoint_index < 0 || total_clusters <= 0) {
      return -1;
    }
    
    // Each cluster has 2 waypoints, except the last one which has 3
    int cluster = 0;
    int wp_count = 0;
    
    for (int i = 0; i < total_clusters; i++) {
      int waypoints_in_cluster = (i == total_clusters - 1) ? 3 : 2;
      if (waypoint_index < wp_count + waypoints_in_cluster) {
        cluster = i;
        break;
      }
      wp_count += waypoints_in_cluster;
    }
    
    return cluster;
  }
  
  // Publish planning status (cluster index, waypoint index, path planning status)
  void publishPlanningStatus(bool path_success) {
    // Calculate cluster index from waypoint index
    int total_clusters = static_cast<int>(cluster_waypoint_nodes_.size());
    int cluster_idx = calculateClusterIndex(current_target_waypoint_index_, total_clusters);
    
    // Publish cluster index
    std_msgs::msg::Int32 cluster_msg;
    cluster_msg.data = cluster_idx;
    pub_cluster_index_->publish(cluster_msg);
    
    // Publish waypoint index
    std_msgs::msg::Int32 waypoint_msg;
    waypoint_msg.data = current_target_waypoint_index_;
    pub_waypoint_index_->publish(waypoint_msg);
    
    // Publish path planning status
    std_msgs::msg::String status_msg;
    status_msg.data = path_success ? "Success" : "Failed";
    pub_path_status_->publish(status_msg);
  }

  void publishMarkers(const std::vector<geometry_msgs::msg::Point>& waypoints) {
    // Use synchronized timestamp for all markers to ensure proper synchronization
    rclcpp::Time current_time = this->get_clock()->now();
    
    visualization_msgs::msg::MarkerArray ma;
    
    // Always add DELETEALL marker first to clear old markers and ensure synchronization
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = "map";
    delete_all.header.stamp = current_time;
    delete_all.ns = ns_ + "_waypoints";
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(delete_all);
    
    visualization_msgs::msg::Marker delete_all_labels;
    delete_all_labels.header.frame_id = "map";
    delete_all_labels.header.stamp = current_time;
    delete_all_labels.ns = ns_ + "_waypoint_labels";
    delete_all_labels.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(delete_all_labels);

    // Waypoint markers with synchronized timestamp
    for (size_t i = 0; i < waypoints.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = current_time;  // Use synchronized timestamp
      marker.ns = ns_ + "_waypoints";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = waypoints[i];
      marker.pose.orientation.w = 1.0;
      
      // Highlight current target waypoint
      // Completed waypoints: i < current_target_waypoint_index_
      // Current target: i == current_target_waypoint_index_
      // Future waypoints: i > current_target_waypoint_index_
      if (current_target_waypoint_index_ < 0) {
        // No target set yet, all waypoints are future waypoints
        static constexpr double DEFAULT_FUTURE_MARKER_SCALE = 0.3;
        marker.scale.x = DEFAULT_FUTURE_MARKER_SCALE;
        marker.scale.y = DEFAULT_FUTURE_MARKER_SCALE;
        marker.scale.z = DEFAULT_FUTURE_MARKER_SCALE;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;  // Red for future waypoints
      } else if (static_cast<int>(i) < current_target_waypoint_index_) {
        // Completed waypoint (passed)
        static constexpr double COMPLETED_MARKER_SCALE = 0.3;
        marker.scale.x = COMPLETED_MARKER_SCALE;
        marker.scale.y = COMPLETED_MARKER_SCALE;
        marker.scale.z = COMPLETED_MARKER_SCALE;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;  // Black for completed waypoints
      } else if (static_cast<int>(i) == current_target_waypoint_index_) {
        // Current target waypoint
        // Display larger: if docking, make it even larger, otherwise normal size
        if (waiting_for_docking_completion_) {
          // Docking WP: display very large (green)
          static constexpr double DOCKING_MARKER_SCALE = 1.0;
          marker.scale.x = DOCKING_MARKER_SCALE;
          marker.scale.y = DOCKING_MARKER_SCALE;
          marker.scale.z = DOCKING_MARKER_SCALE;
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;  // Green for docking
        } else {
          // Normal target WP
          static constexpr double TARGET_MARKER_SCALE = 0.5;
          marker.scale.x = TARGET_MARKER_SCALE;
          marker.scale.y = TARGET_MARKER_SCALE;
          marker.scale.z = TARGET_MARKER_SCALE;
          marker.color.r = 1.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;  // Yellow for current target
        }
      } else {
        // Future waypoint
        static constexpr double FUTURE_MARKER_SCALE = 0.3;
        marker.scale.x = FUTURE_MARKER_SCALE;
        marker.scale.y = FUTURE_MARKER_SCALE;
        marker.scale.z = FUTURE_MARKER_SCALE;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;  // Red for future waypoints
      }
      marker.color.a = 1.0f;
      ma.markers.push_back(marker);

      // Text label
      visualization_msgs::msg::Marker text;
      text.header = marker.header;
      text.ns = ns_ + "_waypoint_labels";
      text.id = static_cast<int>(i);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = waypoints[i];
      static constexpr double TEXT_OFFSET_Z = 0.5;
      text.pose.position.z += TEXT_OFFSET_Z;
      text.pose.orientation.w = 1.0;
      
      // Display text larger for docking WP
      if (waiting_for_docking_completion_ && static_cast<int>(i) == current_target_waypoint_index_) {
        static constexpr double DOCKING_TEXT_SCALE = 0.8;
        text.scale.z = DOCKING_TEXT_SCALE;  // Display larger
        text.color.r = 0.0f;
        text.color.g = 1.0f;
        text.color.b = 0.0f;  // Green
        text.text = "DOCKING WP" + std::to_string(i);
      } else {
        static constexpr double NORMAL_TEXT_SCALE = 0.3;
        text.scale.z = NORMAL_TEXT_SCALE;
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.text = "WP" + std::to_string(i);
      }
      text.color.a = 1.0f;
      ma.markers.push_back(text);
    }

    pub_markers_->publish(ma);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AosPathGenNode>());
  rclcpp::shutdown();
  return 0;
}


