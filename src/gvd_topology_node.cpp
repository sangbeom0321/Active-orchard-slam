#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <algorithm>
#include <map>

// Tree structures for orchard exploration
struct TreeCluster {
    geometry_msgs::msg::Point center;
    double radius;
    double height;
    int point_count;
};

struct TreeRow {
    std::vector<TreeCluster> trees;
    geometry_msgs::msg::Point start_point;
    geometry_msgs::msg::Point end_point;
    double orientation; // in radians
    double length;
    double width;
    int tree_count;
};

class GvdTopologyNode : public rclcpp::Node {
public:
  GvdTopologyNode()
      : rclcpp::Node("gvd_topology_node"),
        qos_odom(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)) {
    qos_odom.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_odom.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Parameters
    this->declare_parameter("connect_radius", 20);
    this->declare_parameter("publish_namespace", std::string("gvd"));
    this->declare_parameter("publish_gvd_grid", true);
    this->declare_parameter("subscribe_topic", std::string("/orbit_planner/pcd_occupancy"));
    this->declare_parameter("goal_topic", std::string("/gvd/goal"));
    this->get_parameter("connect_radius", connect_radius_);
    this->get_parameter("publish_namespace", ns_);
    this->get_parameter("subscribe_topic", occupancy_topic_);
    this->get_parameter("goal_topic", goal_topic_);
    this->get_parameter("publish_gvd_grid", publish_gvd_grid_);

    // Subscribers
    sub_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_topic_, 10,
        std::bind(&GvdTopologyNode::gridCallback, this, std::placeholders::_1));

    // Custom goal topic (configurable)
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10,
        std::bind(&GvdTopologyNode::goalCallback, this, std::placeholders::_1));

    // RViz default goal topics for convenience
    sub_goal_rviz1_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&GvdTopologyNode::goalCallback, this, std::placeholders::_1));
    sub_goal_rviz2_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10,
        std::bind(&GvdTopologyNode::goalCallback, this, std::placeholders::_1));
    
    // Waypoints subscriber
    sub_waypoints_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "/gvd/waypoints", 10,
        std::bind(&GvdTopologyNode::waypointsCallback, this, std::placeholders::_1));
    
    // Tree rows subscriber
    sub_tree_rows_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/tree_rows", 10,
        std::bind(&GvdTopologyNode::treeRowsCallback, this, std::placeholders::_1));
    
    // Exploration area subscriber
    sub_exploration_area_ = create_subscription<geometry_msgs::msg::Polygon>(
        "/orbit_planner/exploration_area", 10,
        std::bind(&GvdTopologyNode::explorationAreaCallback, this, std::placeholders::_1));

    // Publishers
    pub_path_ = create_publisher<nav_msgs::msg::Path>(ns_ + "/path", 10);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(ns_ + "/markers", 10);
    pub_gvd_grid_ = create_publisher<nav_msgs::msg::OccupancyGrid>(ns_ + "/gvd_grid", 10);
    pub_waypoint_path_ = create_publisher<nav_msgs::msg::Path>(ns_ + "/waypoint_path", 10);

    // State
    goal_defined_ = false;
  }

private:
  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_rviz1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_rviz2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_waypoints_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_tree_rows_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_exploration_area_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_gvd_grid_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_waypoint_path_;
  rclcpp::QoS qos_odom;

  // Params
  int connect_radius_ = 20; // in cells
  std::string ns_;
  std::string occupancy_topic_;
  std::string goal_topic_;
  bool publish_gvd_grid_ = true;

  // Map data
  int width_ = 0, height_ = 0;
  double resolution_ = 0.1, origin_x_ = 0.0, origin_y_ = 0.0;
  std::vector<bool> obstacle_;
  std::vector<int> dist_map_;
  const int INF_ = std::numeric_limits<int>::max();

  // Voronoi graph
  std::vector<int> voro_inds_;
  std::unordered_map<int, int> idx2node_;
  std::vector<std::vector<int>> edges_;

  // Goal state
  bool goal_defined_;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  
  // Waypoints state
  std::vector<geometry_msgs::msg::Point> waypoints_;
  bool waypoints_defined_ = false;
  
  // Tree rows state
  std::vector<TreeRow> tree_rows_;
  bool tree_rows_defined_ = false;
  double tree_row_expansion_distance_ = 2.0; // meters to expand tree rows
  
  // Exploration area state
  std::vector<geometry_msgs::msg::Point> exploration_area_;
  bool exploration_area_defined_ = false;
  
  // GVD intersection points for visualization
  std::vector<geometry_msgs::msg::Point> gvd_intersections_;
  std::vector<int> gvd_intersection_visit_order_; // Track visit order for each intersection
  std::vector<std::string> gvd_intersection_types_; // Track type of each intersection (start_left, start_right, end_left, end_right)
  
  // Perpendicular lines for visualization
  std::vector<geometry_msgs::msg::Point> perpendicular_lines_start_left_;
  std::vector<geometry_msgs::msg::Point> perpendicular_lines_start_right_;
  std::vector<geometry_msgs::msg::Point> perpendicular_lines_end_left_;
  std::vector<geometry_msgs::msg::Point> perpendicular_lines_end_right_;
  
  // Path visualization
  std::vector<int> used_gvd_nodes_;

  // Callbacks
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_defined_ = true;
    RCLCPP_INFO(this->get_logger(), "GVD goal set: (%.3f, %.3f)", goal_x_, goal_y_);
  }
  
  void waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    waypoints_.clear();
    for (const auto& pose : msg->poses) {
      geometry_msgs::msg::Point point;
      point.x = pose.position.x;
      point.y = pose.position.y;
      point.z = 0.0;
      waypoints_.push_back(point);
    }
    waypoints_defined_ = true;
    RCLCPP_INFO(this->get_logger(), "GVD waypoints set: %zu waypoints", waypoints_.size());
    
    // Plan waypoint tour if we have a map
    if (width_ > 0 && height_ > 0) {
      planWaypointTour(msg->header);
    }
  }
  
  void explorationAreaCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
    exploration_area_.clear();
    for (const auto& point32 : msg->points) {
      geometry_msgs::msg::Point point;
      point.x = point32.x;
      point.y = point32.y;
      point.z = point32.z;
      exploration_area_.push_back(point);
    }
    exploration_area_defined_ = true;
    RCLCPP_INFO(this->get_logger(), "Exploration area received: %zu points", exploration_area_.size());
  }
  
  void treeRowsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    tree_rows_.clear();
    
    RCLCPP_INFO(this->get_logger(), "Received MarkerArray with %zu markers", msg->markers.size());
    
    // Parse tree row markers from MarkerArray
    // Based on orbit_planner_node.cpp publishTreeRowsVisualization function
    std::map<int, TreeRow> row_map; // Map row index to TreeRow
    
    for (const auto& marker : msg->markers) {
      RCLCPP_DEBUG(this->get_logger(), "Processing marker: ns=%s, type=%d, id=%d, points=%zu", 
                   marker.ns.c_str(), marker.type, marker.id, marker.points.size());
      
      // Parse tree row line markers (LINE_STRIP type)
      if (marker.ns == "tree_rows" && marker.type == visualization_msgs::msg::Marker::LINE_STRIP) {
        int row_id = marker.id / 2; // Even IDs for lines
        
        if (marker.points.size() >= 2) {
          TreeRow row;
          row.start_point = marker.points[0];
          row.end_point = marker.points[1];
          
          // Calculate orientation and length
          double dx = row.end_point.x - row.start_point.x;
          double dy = row.end_point.y - row.start_point.y;
          row.orientation = std::atan2(dy, dx);
          row.length = std::hypot(dx, dy);
          
          // Initialize other properties
          row.tree_count = 0;
          row.width = 0.0;
          
          row_map[row_id] = row;
          RCLCPP_INFO(this->get_logger(), "Found tree row %d: start(%.2f,%.2f) end(%.2f,%.2f), length=%.2f m", 
                      row_id, row.start_point.x, row.start_point.y, row.end_point.x, row.end_point.y, row.length);
        }
      }
      
      // Parse individual tree markers (CYLINDER type for trunks)
      else if (marker.ns == "row_trees" && marker.type == visualization_msgs::msg::Marker::CYLINDER) {
        int row_id = marker.id / 1000; // Extract row ID from marker ID
        int tree_id = (marker.id % 1000) / 2; // Extract tree ID within row
        
        if (row_map.find(row_id) != row_map.end()) {
          TreeCluster tree;
          tree.center.x = marker.pose.position.x;
          tree.center.y = marker.pose.position.y;
          tree.center.z = marker.pose.position.z - 0.25; // Adjust for cylinder center
          tree.radius = marker.scale.x / 2.0; // Half of scale.x
          tree.height = marker.scale.z;
          tree.point_count = 1;
          
          // Ensure we have enough space in the trees vector
          if (row_map[row_id].trees.size() <= static_cast<size_t>(tree_id)) {
            row_map[row_id].trees.resize(tree_id + 1);
          }
          row_map[row_id].trees[tree_id] = tree;
        }
      }
    }
    
    // Convert map to vector and add all rows (we only need start/end points and orientation)
    for (auto& [row_id, row] : row_map) {
      // Set a default tree count (we don't need exact count for GVD planning)
      row.tree_count = 10; // Default value for visualization
      
      // Add all rows that have valid start and end points
      if (row.length > 0.1) { // Only add rows with reasonable length
        tree_rows_.push_back(row);
      }
    }
    
    tree_rows_defined_ = true;
    RCLCPP_INFO(this->get_logger(), "Tree rows processed: %zu rows", tree_rows_.size());
    
    // Log details for each row
    for (size_t i = 0; i < tree_rows_.size(); ++i) {
      const auto& row = tree_rows_[i];
      RCLCPP_INFO(this->get_logger(), "Row %zu: %d trees, start(%.2f,%.2f) end(%.2f,%.2f), orientation=%.3f rad, length=%.2f m", 
                  i+1, row.tree_count, 
                  row.start_point.x, row.start_point.y,
                  row.end_point.x, row.end_point.y,
                  row.orientation, row.length);
    }
    
    // Plan orchard exploration if we have a map and tree rows
    if (width_ > 0 && height_ > 0 && !tree_rows_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Starting orchard exploration planning...");
      // Create a header for the exploration
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = "map";
      planOrchardExploration(header);
    } else {
      if (width_ == 0 || height_ == 0) {
        RCLCPP_WARN(this->get_logger(), "Map not available for orchard exploration (width=%d, height=%d)", width_, height_);
      }
      if (tree_rows_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No tree rows available for orchard exploration");
      }
    }
  }

  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // 1) Map info
    width_ = static_cast<int>(msg->info.width);
    height_ = static_cast<int>(msg->info.height);
    resolution_ = msg->info.resolution;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;

    // 2) Binarize occupancy
    int N = width_ * height_;
    obstacle_.assign(N, false);
    for (int i = 0; i < N; ++i) {
      obstacle_[i] = (msg->data[i] > 0);
    }

    // 3) Distance transform
    computeBrushfire();

    // 4) Extract Voronoi points (GVD)
    extractVoronoi();

    // 5) Build topology graph
    buildGraph();

    // 6) If goal set, plan path on graph from map frame origin to nearest to goal
    // if (goal_defined_) {
    //   RCLCPP_INFO(this->get_logger(), "Goal is defined at (%.3f, %.3f), planning path...", goal_x_, goal_y_);
      
    //   int start_idx = worldToIndex(0.0, 0.0); // using map frame origin (0, 0) as start
    //   int goal_idx = worldToIndex(goal_x_, goal_y_);
      
    //   RCLCPP_INFO(this->get_logger(), "Start index: %d, Goal index: %d", start_idx, goal_idx);
      
    //   int s_node = findNearestNode(start_idx);
    //   int g_node = findNearestNode(goal_idx);
      
    //   RCLCPP_INFO(this->get_logger(), "Nearest GVD nodes - Start: %d, Goal: %d", s_node, g_node);
      
    //   if (s_node == g_node) {
    //     RCLCPP_WARN(this->get_logger(), "Start and goal nodes are the same, creating direct path");
    //     // Create direct path from origin to goal
    //     std::vector<std::pair<double, double>> direct_path;
    //     direct_path.emplace_back(0.0, 0.0);
    //     direct_path.emplace_back(goal_x_, goal_y_);
    //     publishPath(msg->header, direct_path);
    //   } else {
    //     auto raw_path = aStar(s_node, g_node);
    //     if (raw_path.empty()) {
    //       RCLCPP_ERROR(this->get_logger(), "A* path planning failed!");
    //       return;
    //     }
        
    //     auto smooth_pts = projectAndSmooth(raw_path);
        
    //     // Ensure the path starts from map origin (0, 0)
    //     std::vector<std::pair<double, double>> full_path;
    //     full_path.emplace_back(0.0, 0.0); // Start from map origin
    //     full_path.insert(full_path.end(), smooth_pts.begin(), smooth_pts.end());
        
    //     auto final_path = addGoalConnection(full_path);
    //     publishPath(msg->header, final_path);
    //   }
    // } else {
    //   RCLCPP_INFO(this->get_logger(), "No goal defined, skipping path planning");
    // }

    // 7) Publish markers for nodes and edges
    publishMarkers(msg->header);

    // 8) Optionally publish GVD occupancy grid for visualization
    // if (publish_gvd_grid_) {
    //   publishGvdGrid(msg->header);
    // }
  }

  void computeBrushfire() {
    int N = width_ * height_;
    dist_map_.assign(N, INF_);
    std::queue<int> q;
    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    for (int i = 0; i < N; ++i) {
      if (obstacle_[i]) {
        dist_map_[i] = 0;
        q.push(i);
      }
    }
    while (!q.empty()) {
      int idx = q.front();
      q.pop();
      int x = idx % width_;
      int y = idx / width_;
      for (int k = 0; k < 8; ++k) {
        int nx = x + dx[k];
        int ny = y + dy[k];
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
          continue;
        int nidx = ny * width_ + nx;
        if (dist_map_[nidx] > dist_map_[idx] + 1) {
          dist_map_[nidx] = dist_map_[idx] + 1;
          q.push(nidx);
        }
      }
    }
  }

  void extractVoronoi() {
    voro_inds_.clear();
    idx2node_.clear();
    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    
    int total_cells = width_ * height_;
    int obstacle_cells = 0;
    int valid_cells = 0;
    int voronoi_cells = 0;
    
    for (int idx = 0; idx < total_cells; ++idx) {
      if (obstacle_[idx]) {
        obstacle_cells++;
        continue;
      }
      if (dist_map_[idx] == 0 || dist_map_[idx] == INF_) {
        continue;
      }
      valid_cells++;
      
      bool is_ridge = true;
      for (int k = 0; k < 8; ++k) {
        int x = (idx % width_) + dx[k];
        int y = (idx / width_) + dy[k];
        if (x < 0 || x >= width_ || y < 0 || y >= height_)
          continue;
        int nidx = y * width_ + x;
        if (dist_map_[nidx] > dist_map_[idx]) {
          is_ridge = false;
          break;
        }
      }
      if (is_ridge) {
        idx2node_[idx] = static_cast<int>(voro_inds_.size());
        voro_inds_.push_back(idx);
        voronoi_cells++;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Voronoi extraction complete: %d total cells, %d obstacles, %d valid, %d voronoi nodes", 
                total_cells, obstacle_cells, valid_cells, voronoi_cells);
  }

  void verifyFullConnectivity() {
    int M = static_cast<int>(voro_inds_.size());
    if (M <= 1) return;
    
    RCLCPP_INFO(this->get_logger(), "Verifying full connectivity from multiple starting points...");
    
    // Test connectivity from several random starting points
    std::vector<int> test_starts = {0, M/4, M/2, 3*M/4, M-1};
    for (int start : test_starts) {
      if (start >= M) continue;
      
      std::vector<bool> visited(M, false);
      std::queue<int> bfs_queue;
      bfs_queue.push(start);
      visited[start] = true;
      int reachable = 1;
      
      while (!bfs_queue.empty()) {
        int u = bfs_queue.front();
        bfs_queue.pop();
        for (int v : edges_[u]) {
          if (!visited[v]) {
            visited[v] = true;
            bfs_queue.push(v);
            reachable++;
          }
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "From start node %d: %d/%d nodes reachable", start, reachable, M);
      
      if (reachable < M) {
        RCLCPP_ERROR(this->get_logger(), "Connectivity verification failed from node %d!", start);
        return;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Full connectivity verification passed! Graph is fully connected.");
  }

  void ensureFullyConnectedGraph() {
    int M = static_cast<int>(voro_inds_.size());
    if (M <= 1) return;
    
    // Check current connectivity
    std::vector<bool> visited(M, false);
    std::queue<int> bfs_queue;
    bfs_queue.push(0);
    visited[0] = true;
    int reachable = 1;
    
    while (!bfs_queue.empty()) {
      int u = bfs_queue.front();
      bfs_queue.pop();
      for (int v : edges_[u]) {
        if (!visited[v]) {
          visited[v] = true;
          bfs_queue.push(v);
          reachable++;
        }
      }
    }
    
    if (reachable == M) {
      RCLCPP_INFO(this->get_logger(), "Graph is already fully connected (%d/%d nodes reachable)", reachable, M);
      return;
    }
    
    RCLCPP_WARN(this->get_logger(), "Graph is not fully connected (%d/%d nodes reachable), adding MST connections...", reachable, M);
    
    // Find all unconnected nodes
    std::vector<int> unconnected_nodes;
    for (int i = 0; i < M; ++i) {
      if (!visited[i]) {
        unconnected_nodes.push_back(i);
      }
    }
    
    // Connect unconnected nodes to the main component using closest connections
    int connections_added = 0;
    for (int unconnected_node : unconnected_nodes) {
      double min_distance = std::numeric_limits<double>::infinity();
      int closest_connected = -1;
      
      // Find closest node in the main connected component
      for (int i = 0; i < M; ++i) {
        if (visited[i] && i != unconnected_node) {
          int idx_unconnected = voro_inds_[unconnected_node];
          int idx_connected = voro_inds_[i];
          double x_unconnected = static_cast<double>(idx_unconnected % width_);
          double y_unconnected = static_cast<double>(idx_unconnected / width_);
          double x_connected = static_cast<double>(idx_connected % width_);
          double y_connected = static_cast<double>(idx_connected / width_);
          double distance = std::hypot(x_unconnected - x_connected, y_unconnected - y_connected);
          
          if (distance < min_distance) {
            min_distance = distance;
            closest_connected = i;
          }
        }
      }
      
      // Add connection
      if (closest_connected != -1) {
        edges_[unconnected_node].push_back(closest_connected);
        edges_[closest_connected].push_back(unconnected_node);
        connections_added++;
        
        // RCLCPP_INFO(this->get_logger(), "MST connection: node %d -> node %d (distance: %.2f)", 
        //            unconnected_node, closest_connected, min_distance);
        
        // Mark as visited and add to connected component
        visited[unconnected_node] = true;
        bfs_queue.push(unconnected_node);
        reachable++;
        
        // Update connectivity for next iteration
        while (!bfs_queue.empty()) {
          int u = bfs_queue.front();
          bfs_queue.pop();
          for (int v : edges_[u]) {
            if (!visited[v]) {
              visited[v] = true;
              bfs_queue.push(v);
              reachable++;
            }
          }
        }
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "MST connections added: %d, final connectivity: %d/%d nodes", 
                connections_added, reachable, M);
    
    if (reachable < M) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create fully connected graph! Only %d/%d nodes reachable", reachable, M);
    } else {
      RCLCPP_INFO(this->get_logger(), "Successfully created fully connected graph!");
      
      // Final verification: ensure all nodes are reachable from any starting point
      verifyFullConnectivity();
    }
  }

  void buildGraph() {
    int M = static_cast<int>(voro_inds_.size());
    edges_.assign(M, {});
    
    RCLCPP_INFO(this->get_logger(), "Building graph with %d nodes, connection radius: %d cells (%.2f m)", 
                M, connect_radius_, connect_radius_ * resolution_);
    
    int total_edges = 0;
    int debug_connections = 0;
    for (int i = 0; i < M; ++i) {
      int idx = voro_inds_[i];
      int x = idx % width_;
      int y = idx / width_;
      int node_edges = 0;
      
      // Debug: log first few nodes
      if (i < 1) { // Only log first node
        double wx = origin_x_ + x * resolution_;
        double wy = origin_y_ + y * resolution_;
        RCLCPP_INFO(this->get_logger(), "Node %d: grid(%d,%d) world(%.2f,%.2f)", 
                    i, x, y, wx, wy);
      }
      
      for (int dy = -connect_radius_; dy <= connect_radius_; ++dy) {
        for (int dx = -connect_radius_; dx <= connect_radius_; ++dx) {
          if (dx == 0 && dy == 0)
            continue;
          
          // Use Euclidean distance instead of Chebyshev distance
          double distance = std::hypot(dx, dy);
          if (distance > connect_radius_)
            continue;
            
          int nx = x + dx;
          int ny = y + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
            continue;
          int nidx = ny * width_ + nx;
          auto it = idx2node_.find(nidx);
          if (it != idx2node_.end()) {
            int neighbor_node = it->second;
            
            // Debug: log connection details for first few nodes
            if (i < 1 && debug_connections < 5) {
              double real_distance = distance * resolution_;
              double nwx = origin_x_ + nx * resolution_;
              double nwy = origin_y_ + ny * resolution_;
              RCLCPP_INFO(this->get_logger(), "  Checking connection: node %d -> node %d, distance: %.2f cells (%.2f m), world(%.2f,%.2f)", 
                         i, neighbor_node, distance, real_distance, nwx, nwy);
              debug_connections++;
            }
            
            // Check if this edge crosses any tree row
            double start_x = origin_x_ + x * resolution_;
            double start_y = origin_y_ + y * resolution_;
            double end_x = origin_x_ + nx * resolution_;
            double end_y = origin_y_ + ny * resolution_;
            
            bool crosses_tree_row = false;
            if (tree_rows_defined_ && !tree_rows_.empty()) {
              for (const auto& row : tree_rows_) {
                if (lineSegmentsIntersect(start_x, start_y, end_x, end_y,
                                         row.start_point.x, row.start_point.y,
                                         row.end_point.x, row.end_point.y)) {
                  crosses_tree_row = true;
                  break;
                }
              }
            }
            
            // Skip this connection if it crosses a tree row
            if (crosses_tree_row) {
              if (i < 1) {
                RCLCPP_INFO(this->get_logger(), "  ✗ Blocked: node %d -> node %d (crosses tree row)", 
                           i, neighbor_node);
              }
              continue;
            }
            
            // Avoid duplicate connections
            bool already_connected = false;
            for (int existing_neighbor : edges_[i]) {
              if (existing_neighbor == neighbor_node) {
                already_connected = true;
                break;
              }
            }
            
            if (!already_connected) {
              edges_[i].push_back(neighbor_node);
              node_edges++;
              total_edges++;
              
              // Debug: log successful connection
              if (i < 1) {
                double real_distance = distance * resolution_;
                RCLCPP_INFO(this->get_logger(), "  ✓ Connected: node %d -> node %d (%.2f m)", 
                           i, neighbor_node, real_distance);
              }
            }
          }
        }
      }
    }
    
    // Ensure symmetry: if A connects to B, then B should connect to A
    int symmetry_fixes = 0;
    for (int i = 0; i < M; ++i) {
      for (int neighbor : edges_[i]) {
        // Check if neighbor also has connection back to i
        bool has_reverse_connection = false;
        for (int reverse_neighbor : edges_[neighbor]) {
          if (reverse_neighbor == i) {
            has_reverse_connection = true;
            break;
          }
        }
        
        if (!has_reverse_connection) {
          edges_[neighbor].push_back(i);
          symmetry_fixes++;
        }
      }
    }
    
    if (symmetry_fixes > 0) {
      RCLCPP_INFO(this->get_logger(), "Added %d reverse connections for symmetry", symmetry_fixes);
    }
    
    RCLCPP_INFO(this->get_logger(), "Graph building complete: %d total edges, avg %.2f edges per node", 
                total_edges + symmetry_fixes, M > 0 ? static_cast<double>(total_edges + symmetry_fixes) / M : 0.0);
    
    // Additional connectivity statistics
    int max_edges = 0, min_edges = M > 0 ? M : 0;
    for (int i = 0; i < M; ++i) {
      int edge_count = static_cast<int>(edges_[i].size());
      max_edges = std::max(max_edges, edge_count);
      min_edges = std::min(min_edges, edge_count);
    }
    RCLCPP_INFO(this->get_logger(), "Edge statistics: min=%d, max=%d per node", min_edges, max_edges);
    
    // Debug: Find nodes that should be connected but aren't
    RCLCPP_INFO(this->get_logger(), "Checking for missed connections within %.2f m radius...", connect_radius_ * resolution_);
    int missed_connections = 0;
    for (int i = 0; i < std::min(M, 5); ++i) { // Check first 5 nodes
      for (int j = i + 1; j < M; ++j) {
        int idx_i = voro_inds_[i];
        int idx_j = voro_inds_[j];
        double x_i = static_cast<double>(idx_i % width_);
        double y_i = static_cast<double>(idx_i / width_);
        double x_j = static_cast<double>(idx_j % width_);
        double y_j = static_cast<double>(idx_j / width_);
        double distance = std::hypot(x_i - x_j, y_i - y_j);
        double real_distance = distance * resolution_;
        
        if (real_distance <= connect_radius_ * resolution_) {
          // Check if they are actually connected
          bool connected = false;
          for (int neighbor : edges_[i]) {
            if (neighbor == j) {
              connected = true;
              break;
            }
          }
          
          if (!connected) {
            missed_connections++;
            if (missed_connections <= 3) { // Log first 3 missed connections
              double wx_i = origin_x_ + x_i * resolution_;
              double wy_i = origin_y_ + y_i * resolution_;
              double wx_j = origin_x_ + x_j * resolution_;
              double wy_j = origin_y_ + y_j * resolution_;
              RCLCPP_WARN(this->get_logger(), "MISSED CONNECTION: node %d(%.2f,%.2f) <-> node %d(%.2f,%.2f), distance: %.2f m", 
                         i, wx_i, wy_i, j, wx_j, wy_j, real_distance);
            }
          }
        }
      }
    }
    
    if (missed_connections > 0) {
      RCLCPP_WARN(this->get_logger(), "Found %d missed connections within %.2f m radius!", 
                 missed_connections, connect_radius_ * resolution_);
    }
    
    // Check for isolated nodes
    int isolated_nodes = 0;
    for (int i = 0; i < M; ++i) {
      if (edges_[i].empty()) {
        isolated_nodes++;
      }
    }
    
    if (isolated_nodes > 0) {
      RCLCPP_WARN(this->get_logger(), "Found %d isolated nodes (nodes with no connections)", isolated_nodes);
    }
    
    // Check graph connectivity
    if (M > 0) {
      std::vector<bool> visited(M, false);
      std::queue<int> bfs_queue;
      bfs_queue.push(0);
      visited[0] = true;
      int connected_components = 1;
      int reachable_from_first = 1;
      
      while (!bfs_queue.empty()) {
        int u = bfs_queue.front();
        bfs_queue.pop();
        for (int v : edges_[u]) {
          if (!visited[v]) {
            visited[v] = true;
            bfs_queue.push(v);
            reachable_from_first++;
          }
        }
      }
      
      // Check for other connected components
      for (int i = 1; i < M; ++i) {
        if (!visited[i]) {
          connected_components++;
          // BFS from this unvisited node
          bfs_queue.push(i);
          visited[i] = true;
          while (!bfs_queue.empty()) {
            int u = bfs_queue.front();
            bfs_queue.pop();
            for (int v : edges_[u]) {
              if (!visited[v]) {
                visited[v] = true;
                bfs_queue.push(v);
              }
            }
          }
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "Graph connectivity: %d connected components, %d nodes reachable from first node", 
                  connected_components, reachable_from_first);
      
      if (connected_components > 1) {
        RCLCPP_WARN(this->get_logger(), "Graph is disconnected! Attempting to connect isolated components...");
        
        // Try to connect isolated components by finding closest pairs
        int connections_added = 0;
        for (int i = 0; i < M; ++i) {
          if (edges_[i].empty()) continue; // Skip isolated nodes
          
          // Find the closest node in other components
          double min_distance = std::numeric_limits<double>::infinity();
          int closest_node = -1;
          
          for (int j = 0; j < M; ++j) {
            if (i == j || !edges_[j].empty()) continue; // Skip self and non-isolated nodes
            
            int idx_i = voro_inds_[i];
            int idx_j = voro_inds_[j];
            double x_i = static_cast<double>(idx_i % width_);
            double y_i = static_cast<double>(idx_i / width_);
            double x_j = static_cast<double>(idx_j % width_);
            double y_j = static_cast<double>(idx_j / width_);
            double distance = std::hypot(x_i - x_j, y_i - y_j);
            
            if (distance < min_distance) {
              min_distance = distance;
              closest_node = j;
            }
          }
          
          // Add connection if found a close isolated node
          if (closest_node != -1 && min_distance < connect_radius_ * 2) {
            edges_[i].push_back(closest_node);
            edges_[closest_node].push_back(i);
            connections_added++;
            
            RCLCPP_INFO(this->get_logger(), "Connected isolated node %d to node %d (distance: %.2f)", 
                       closest_node, i, min_distance);
          }
        }
        
        if (connections_added > 0) {
          RCLCPP_INFO(this->get_logger(), "Added %d connections to reduce graph disconnection", connections_added);
          
          // Re-check connectivity after adding connections
          std::vector<bool> visited_after(M, false);
          std::queue<int> bfs_queue_after;
          bfs_queue_after.push(0);
          visited_after[0] = true;
          int reachable_after = 1;
          
          while (!bfs_queue_after.empty()) {
            int u = bfs_queue_after.front();
            bfs_queue_after.pop();
            for (int v : edges_[u]) {
              if (!visited_after[v]) {
                visited_after[v] = true;
                bfs_queue_after.push(v);
                reachable_after++;
              }
            }
          }
          
          RCLCPP_INFO(this->get_logger(), "After connection: %d nodes reachable from first node", reachable_after);
        } else {
          RCLCPP_WARN(this->get_logger(), "Could not find close enough nodes to connect isolated components");
        }
      }
      
      // Ensure fully connected graph using MST approach
      RCLCPP_INFO(this->get_logger(), "Ensuring fully connected graph using MST approach...");
      ensureFullyConnectedGraph();
    }
  }

  int worldToIndex(double wx, double wy) const {
    int mx = static_cast<int>((wx - origin_x_) / resolution_);
    int my = static_cast<int>((wy - origin_y_) / resolution_);
    return my * width_ + mx;
  }

  // Check if a point is inside a polygon using ray casting algorithm
  bool isPointInPolygon(const geometry_msgs::msg::Point& point, const std::vector<geometry_msgs::msg::Point>& polygon) const {
    if (polygon.size() < 3) return false;
    
    bool inside = false;
    size_t j = polygon.size() - 1;
    
    for (size_t i = 0; i < polygon.size(); i++) {
      if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
          (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)) {
        inside = !inside;
      }
      j = i;
    }
    
    return inside;
  }
  
  // Check if a tree row is within the exploration area
  bool isTreeRowInExplorationArea(const TreeRow& row) const {
    if (!exploration_area_defined_ || exploration_area_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No exploration area defined, including all tree rows");
      return true; // If no exploration area is defined, include all rows
    }
    
    // Check if both start and end points are within the exploration area
    bool start_inside = isPointInPolygon(row.start_point, exploration_area_);
    bool end_inside = isPointInPolygon(row.end_point, exploration_area_);
    
    // Include row if at least one endpoint is inside the exploration area
    bool row_inside = start_inside || end_inside;
    
    RCLCPP_INFO(this->get_logger(), "Tree row check: start(%.2f,%.2f) %s, end(%.2f,%.2f) %s -> %s", 
                row.start_point.x, row.start_point.y, start_inside ? "inside" : "outside",
                row.end_point.x, row.end_point.y, end_inside ? "inside" : "outside",
                row_inside ? "INCLUDED" : "EXCLUDED");
    
    return row_inside;
  }

  // Check if two line segments intersect
  bool lineSegmentsIntersect(double x1, double y1, double x2, double y2,
                            double x3, double y3, double x4, double y4) const {
    // Calculate the direction vectors
    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x4 - x3;
    double dy2 = y4 - y3;
    
    // Calculate the cross product
    double cross = dx1 * dy2 - dy1 * dx2;
    
    // If cross product is zero, lines are parallel
    if (std::abs(cross) < 1e-10) {
      return false;
    }
    
    // Calculate the intersection point parameters
    double t1 = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / cross;
    double t2 = ((x3 - x1) * dy1 - (y3 - y1) * dx1) / cross;
    
    // Check if intersection point is within both line segments
    return (t1 >= 0.0 && t1 <= 1.0 && t2 >= 0.0 && t2 <= 1.0);
  }

  int findNearestNode(int idx) const {
    if (voro_inds_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No Voronoi nodes available for nearest node search");
      return 0;
    }
    
    double best_d = std::numeric_limits<double>::infinity();
    int best_n = 0;
    double ix = static_cast<double>(idx % width_);
    double iy = static_cast<double>(idx / width_);
    
    for (size_t i = 0; i < voro_inds_.size(); ++i) {
      int vidx = voro_inds_[i];
      double vx = static_cast<double>(vidx % width_);
      double vy = static_cast<double>(vidx / width_);
      double d = std::hypot(vx - ix, vy - iy);
      if (d < best_d) {
        best_d = d;
        best_n = static_cast<int>(i);
      }
    }
    
    // Convert to world coordinates for logging
    double wx = origin_x_ + ix * resolution_;
    double wy = origin_y_ + iy * resolution_;
    double best_wx = origin_x_ + (voro_inds_[best_n] % width_) * resolution_;
    double best_wy = origin_y_ + (voro_inds_[best_n] / width_) * resolution_;
    
    RCLCPP_INFO(this->get_logger(), "Nearest node search: (%.3f, %.3f) -> node %d at (%.3f, %.3f), distance: %.3f", 
                wx, wy, best_n, best_wx, best_wy, best_d * resolution_);
    
    return best_n;
  }

  struct Node {
    int id;
    double f;
  };
  struct Cmp {
    bool operator()(const Node &a, const Node &b) const { return a.f > b.f; }
  };

  double heuristic(int a, int b) const {
    int ia = voro_inds_[a];
    int ib = voro_inds_[b];
    double xa = static_cast<double>(ia % width_);
    double ya = static_cast<double>(ia / width_);
    double xb = static_cast<double>(ib % width_);
    double yb = static_cast<double>(ib / width_);
    return std::hypot(xa - xb, ya - yb);
  }

  double edgeCost(int a, int b) const { return heuristic(a, b); }

  std::vector<int> aStar(int s, int g) const {
    int M = static_cast<int>(voro_inds_.size());
    if (M == 0) {
      RCLCPP_WARN(this->get_logger(), "No Voronoi nodes available for path planning");
      return {};
    }
    
    if (s == g) {
      RCLCPP_INFO(this->get_logger(), "Start and goal are the same node, returning single node path");
      return {s};
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting A* path planning from node %d to node %d (total nodes: %d)", s, g, M);
    
    std::vector<double> gScore(M, std::numeric_limits<double>::infinity());
    std::vector<double> fScore(M, std::numeric_limits<double>::infinity());
    std::vector<int> cameFrom(M, -1);
    std::vector<bool> closedSet(M, false);
    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    
    gScore[s] = 0.0;
    fScore[s] = heuristic(s, g);
    open.push({s, fScore[s]});
    
    int nodes_explored = 0;
    bool goal_found = false;
    
    while (!open.empty()) {
      auto top = open.top();
      open.pop();
      int u = top.id;
      
      if (closedSet[u]) continue; // Skip already processed nodes
      closedSet[u] = true;
      nodes_explored++;
      
      if (u == g) {
        RCLCPP_INFO(this->get_logger(), "Goal reached! Explored %d nodes", nodes_explored);
        goal_found = true;
        break;
      }
      
      for (int v : edges_[u]) {
        if (closedSet[v]) continue; // Skip already processed neighbors
        
        double tentative = gScore[u] + edgeCost(u, v);
        if (tentative < gScore[v]) {
          cameFrom[v] = u;
          gScore[v] = tentative;
          fScore[v] = tentative + heuristic(v, g);
          open.push({v, fScore[v]});
        }
      }
    }
    
    if (!goal_found) {
      RCLCPP_ERROR(this->get_logger(), "A* failed to find path from node %d to node %d", s, g);
      RCLCPP_ERROR(this->get_logger(), "Explored %d nodes out of %d total nodes", nodes_explored, M);
      
      // Debug: check if start and goal nodes are connected
      RCLCPP_ERROR(this->get_logger(), "Start node %d has %zu edges, Goal node %d has %zu edges", 
                   s, edges_[s].size(), g, edges_[g].size());
      
      // Check connectivity using BFS
      std::vector<bool> visited(M, false);
      std::queue<int> bfs_queue;
      bfs_queue.push(s);
      visited[s] = true;
      int reachable_nodes = 1;
      
      while (!bfs_queue.empty()) {
        int u = bfs_queue.front();
        bfs_queue.pop();
        for (int v : edges_[u]) {
          if (!visited[v]) {
            visited[v] = true;
            bfs_queue.push(v);
            reachable_nodes++;
          }
        }
      }
      
      RCLCPP_ERROR(this->get_logger(), "BFS from start node %d can reach %d nodes out of %d", 
                   s, reachable_nodes, M);
      RCLCPP_ERROR(this->get_logger(), "Goal node %d is %s", g, visited[g] ? "reachable" : "NOT reachable");
      
      return {};
    }
    
    std::vector<int> path;
    for (int cur = g; cur != -1; cur = cameFrom[cur])
      path.push_back(cur);
    std::reverse(path.begin(), path.end());
    
    RCLCPP_INFO(this->get_logger(), "A* completed. Path length: %zu nodes", path.size());
    
    // Debug: print path nodes
    for (size_t i = 0; i < path.size(); ++i) {
      int idx = voro_inds_[path[i]];
      double wx = origin_x_ + (idx % width_) * resolution_;
      double wy = origin_y_ + (idx / width_) * resolution_;
      RCLCPP_INFO(this->get_logger(), "Path node %zu: %d at (%.3f, %.3f)", i, path[i], wx, wy);
    }
    
    return path;
  }

  std::vector<std::pair<double, double>> projectAndSmooth(const std::vector<int> &nodes) {
    std::vector<std::pair<double, double>> P;
    P.reserve(nodes.size());
    for (int id : nodes) {
      int idx = voro_inds_[id];
      double x = origin_x_ + (idx % width_) * resolution_;
      double y = origin_y_ + (idx / width_) * resolution_;
      P.emplace_back(x, y);
    }
    
    RCLCPP_INFO(this->get_logger(), "Projected %zu nodes to world coordinates", P.size());
    
    // Conservative smoothing to avoid cutting corners
    auto S = P;
    const double w_data = 0.8, w_smooth = 0.2; // Increased data weight, reduced smooth weight
    for (int iter = 0; iter < 5; ++iter) { // Reduced iterations from 10 to 5
      for (size_t i = 1; i + 1 < P.size(); ++i) {
        // Check if smoothing would create collision
        double smoothed_x = (w_data * P[i].first + w_smooth * (S[i - 1].first + S[i + 1].first)) / (w_data + 2.0 * w_smooth);
        double smoothed_y = (w_data * P[i].second + w_smooth * (S[i - 1].second + S[i + 1].second)) / (w_data + 2.0 * w_smooth);
        
        // Check collision for smoothed point
        int smoothed_idx = worldToIndex(smoothed_x, smoothed_y);
        bool collision = false;
        
        if (smoothed_idx >= 0 && smoothed_idx < static_cast<int>(obstacle_.size())) {
          // Check 3x3 neighborhood around smoothed point for safety
          int mx = smoothed_idx % width_;
          int my = smoothed_idx / width_;
          
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              int check_idx = (my + dy) * width_ + (mx + dx);
              if (check_idx >= 0 && check_idx < static_cast<int>(obstacle_.size()) && obstacle_[check_idx]) {
                collision = true;
                break;
              }
            }
            if (collision) break;
          }
        }
        
        // Only apply smoothing if no collision detected
        if (!collision) {
          S[i].first = smoothed_x;
          S[i].second = smoothed_y;
        }
        // If collision detected, keep original point
      }
    }
    return S;
  }
  

  void publishPath(const std_msgs::msg::Header &h, const std::vector<std::pair<double, double>> &pts) {
    if (pts.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot publish empty path");
      return;
    }
    
    nav_msgs::msg::Path path;
    path.header = h;
    path.header.frame_id = h.frame_id;
    
    for (auto &[x, y] : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = h;
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing path with %zu points from (%.3f, %.3f) to (%.3f, %.3f)", 
                pts.size(), pts[0].first, pts[0].second, pts.back().first, pts.back().second);
    
    pub_path_->publish(path);
  }

  void publishMarkers(const std_msgs::msg::Header &h) {
    visualization_msgs::msg::MarkerArray ma;

    // All GVD Nodes (green)
    visualization_msgs::msg::Marker all_nodes;
    all_nodes.header = h;
    all_nodes.ns = ns_ + "_all_nodes";
    all_nodes.id = 0;
    all_nodes.type = visualization_msgs::msg::Marker::POINTS;
    all_nodes.action = visualization_msgs::msg::Marker::ADD;
    all_nodes.scale.x = 0.03;
    all_nodes.scale.y = 0.03;
    all_nodes.color.g = 1.0f;
    all_nodes.color.a = 0.5f; // Semi-transparent
    for (int idx : voro_inds_) {
      geometry_msgs::msg::Point p;
      p.x = origin_x_ + (idx % width_) * resolution_;
      p.y = origin_y_ + (idx / width_) * resolution_;
      p.z = 0.0;
      all_nodes.points.push_back(p);
    }
    ma.markers.push_back(all_nodes);

    // Used GVD Nodes (red) - only if there are used nodes
    if (!used_gvd_nodes_.empty()) {
      visualization_msgs::msg::Marker used_nodes;
      used_nodes.header = h;
      used_nodes.ns = ns_ + "_used_nodes";
      used_nodes.id = 1;
      used_nodes.type = visualization_msgs::msg::Marker::POINTS;
      used_nodes.action = visualization_msgs::msg::Marker::ADD;
      used_nodes.scale.x = 0.08;
      used_nodes.scale.y = 0.08;
      used_nodes.color.r = 1.0f;
      used_nodes.color.a = 1.0f; // Fully opaque
      
      for (int node_id : used_gvd_nodes_) {
        if (node_id >= 0 && node_id < static_cast<int>(voro_inds_.size())) {
          int idx = voro_inds_[node_id];
          geometry_msgs::msg::Point p;
          p.x = origin_x_ + (idx % width_) * resolution_;
          p.y = origin_y_ + (idx / width_) * resolution_;
          p.z = 0.0;
          used_nodes.points.push_back(p);
        }
      }
      ma.markers.push_back(used_nodes);
    }

    // GVD Intersection Points - create separate markers for each type
    if (!gvd_intersections_.empty()) {
      // Create markers for each intersection type with different colors
      std::map<std::string, std::vector<size_t>> type_indices;
      for (size_t i = 0; i < gvd_intersections_.size(); ++i) {
        type_indices[gvd_intersection_types_[i]].push_back(i);
      }
      
      // Use fixed marker IDs to prevent accumulation
      std::map<std::string, int> type_to_id = {
        {"start_left", 2},
        {"start_right", 3},
        {"end_left", 4},
        {"end_right", 5}
      };
      
      for (const auto& [type, indices] : type_indices) {
        visualization_msgs::msg::Marker intersection_marker;
        intersection_marker.header = h;
        intersection_marker.ns = ns_ + "_gvd_intersections_" + type;
        intersection_marker.id = type_to_id[type];
        intersection_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        intersection_marker.action = visualization_msgs::msg::Marker::ADD;
        intersection_marker.scale.x = 0.3; // Large spheres
        intersection_marker.scale.y = 0.3;
        intersection_marker.scale.z = 0.3;
        
        // Set color based on type
        if (type == "start_left") {
          intersection_marker.color.r = 1.0f; // Red
          intersection_marker.color.g = 0.0f;
          intersection_marker.color.b = 0.0f;
        } else if (type == "start_right") {
          intersection_marker.color.r = 0.0f;
          intersection_marker.color.g = 1.0f; // Green
          intersection_marker.color.b = 0.0f;
        } else if (type == "end_left") {
          intersection_marker.color.r = 0.0f;
          intersection_marker.color.g = 0.0f;
          intersection_marker.color.b = 1.0f; // Blue
        } else if (type == "end_right") {
          intersection_marker.color.r = 1.0f; // Magenta
          intersection_marker.color.g = 0.0f;
          intersection_marker.color.b = 1.0f;
        } else {
          intersection_marker.color.r = 0.5f; // Gray for unknown
          intersection_marker.color.g = 0.5f;
          intersection_marker.color.b = 0.5f;
        }
        intersection_marker.color.a = 0.9f; // Semi-transparent
        
        for (size_t idx : indices) {
          geometry_msgs::msg::Point p;
          p.x = gvd_intersections_[idx].x;
          p.y = gvd_intersections_[idx].y;
          p.z = gvd_intersections_[idx].z;
          intersection_marker.points.push_back(p);
        }
        ma.markers.push_back(intersection_marker);
      }
      
      // Add numbered text labels for each intersection showing visit order
      // Use fixed IDs to prevent accumulation (max 20 intersections)
      for (size_t i = 0; i < gvd_intersections_.size() && i < 20; ++i) {
        visualization_msgs::msg::Marker number_marker;
        number_marker.header = h;
        number_marker.ns = ns_ + "_intersection_numbers";
        number_marker.id = 10 + static_cast<int>(i); // Fixed IDs from 10-29
        number_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        number_marker.action = visualization_msgs::msg::Marker::ADD;
        number_marker.pose.position.x = gvd_intersections_[i].x;
        number_marker.pose.position.y = gvd_intersections_[i].y;
        number_marker.pose.position.z = gvd_intersections_[i].z + 0.5; // Above the sphere
        number_marker.pose.orientation.w = 1.0;
        number_marker.scale.z = 0.4; // Text size
        number_marker.color.r = 1.0f;
        number_marker.color.g = 1.0f;
        number_marker.color.b = 0.0f; // Yellow text
        number_marker.color.a = 1.0f;
        number_marker.text = std::to_string(gvd_intersection_visit_order_[i]);
        ma.markers.push_back(number_marker);
      }
      
      // Add legend for intersection types
      visualization_msgs::msg::Marker legend_marker;
      legend_marker.header = h;
      legend_marker.ns = ns_ + "_intersection_legend";
      legend_marker.id = 100;
      legend_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      legend_marker.action = visualization_msgs::msg::Marker::ADD;
      legend_marker.pose.position.x = 0.0;
      legend_marker.pose.position.y = 0.0;
      legend_marker.pose.position.z = 2.5; // Above the map
      legend_marker.pose.orientation.w = 1.0;
      legend_marker.scale.z = 0.4; // Text size
      legend_marker.color.r = 1.0f;
      legend_marker.color.g = 1.0f;
      legend_marker.color.b = 1.0f;
      legend_marker.color.a = 1.0f;
      legend_marker.text = "Legend: Red=Start_Left, Green=Start_Right, Blue=End_Left, Magenta=End_Right";
      ma.markers.push_back(legend_marker);
      
      // Add summary text for intersection count and visit order
      visualization_msgs::msg::Marker text_marker;
      text_marker.header = h;
      text_marker.ns = ns_ + "_intersection_text";
      text_marker.id = 3;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position.x = 0.0;
      text_marker.pose.position.y = 0.0;
      text_marker.pose.position.z = 2.0; // Above the map
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.5; // Text size
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 1.0f;
      text_marker.color.a = 1.0f;
      text_marker.text = "GVD Intersections: " + std::to_string(gvd_intersections_.size()) + " (Numbers show visit order)";
      ma.markers.push_back(text_marker);
    }
    
    // Add perpendicular lines visualization
    if (!perpendicular_lines_start_left_.empty()) {
      // Start left perpendicular lines
      visualization_msgs::msg::Marker start_left_lines;
      start_left_lines.header = h;
      start_left_lines.ns = ns_ + "_perpendicular_start_left";
      start_left_lines.id = 200;
      start_left_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      start_left_lines.action = visualization_msgs::msg::Marker::ADD;
      start_left_lines.scale.x = 0.05; // Line width
      start_left_lines.color.r = 1.0f; // Red
      start_left_lines.color.g = 0.0f;
      start_left_lines.color.b = 0.0f;
      start_left_lines.color.a = 0.7f; // Semi-transparent
      start_left_lines.points = perpendicular_lines_start_left_;
      ma.markers.push_back(start_left_lines);
      
      // Start right perpendicular lines
      visualization_msgs::msg::Marker start_right_lines;
      start_right_lines.header = h;
      start_right_lines.ns = ns_ + "_perpendicular_start_right";
      start_right_lines.id = 201;
      start_right_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      start_right_lines.action = visualization_msgs::msg::Marker::ADD;
      start_right_lines.scale.x = 0.05; // Line width
      start_right_lines.color.r = 0.0f;
      start_right_lines.color.g = 1.0f; // Green
      start_right_lines.color.b = 0.0f;
      start_right_lines.color.a = 0.7f; // Semi-transparent
      start_right_lines.points = perpendicular_lines_start_right_;
      ma.markers.push_back(start_right_lines);
      
      // End left perpendicular lines
      visualization_msgs::msg::Marker end_left_lines;
      end_left_lines.header = h;
      end_left_lines.ns = ns_ + "_perpendicular_end_left";
      end_left_lines.id = 202;
      end_left_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      end_left_lines.action = visualization_msgs::msg::Marker::ADD;
      end_left_lines.scale.x = 0.05; // Line width
      end_left_lines.color.r = 0.0f;
      end_left_lines.color.g = 0.0f;
      end_left_lines.color.b = 1.0f; // Blue
      end_left_lines.color.a = 0.7f; // Semi-transparent
      end_left_lines.points = perpendicular_lines_end_left_;
      ma.markers.push_back(end_left_lines);
      
      // End right perpendicular lines
      visualization_msgs::msg::Marker end_right_lines;
      end_right_lines.header = h;
      end_right_lines.ns = ns_ + "_perpendicular_end_right";
      end_right_lines.id = 203;
      end_right_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      end_right_lines.action = visualization_msgs::msg::Marker::ADD;
      end_right_lines.scale.x = 0.05; // Line width
      end_right_lines.color.r = 1.0f; // Magenta
      end_right_lines.color.g = 0.0f;
      end_right_lines.color.b = 1.0f;
      end_right_lines.color.a = 0.7f; // Semi-transparent
      end_right_lines.points = perpendicular_lines_end_right_;
      ma.markers.push_back(end_right_lines);
      
      // Add perpendicular lines legend
      visualization_msgs::msg::Marker perp_legend_marker;
      perp_legend_marker.header = h;
      perp_legend_marker.ns = ns_ + "_perpendicular_legend";
      perp_legend_marker.id = 204;
      perp_legend_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      perp_legend_marker.action = visualization_msgs::msg::Marker::ADD;
      perp_legend_marker.pose.position.x = 0.0;
      perp_legend_marker.pose.position.y = 0.0;
      perp_legend_marker.pose.position.z = 3.0; // Above the map
      perp_legend_marker.pose.orientation.w = 1.0;
      perp_legend_marker.scale.z = 0.4; // Text size
      perp_legend_marker.color.r = 1.0f;
      perp_legend_marker.color.g = 1.0f;
      perp_legend_marker.color.b = 1.0f;
      perp_legend_marker.color.a = 1.0f;
      perp_legend_marker.text = "Perpendicular Lines: Red=Start_Left, Green=Start_Right, Blue=End_Left, Magenta=End_Right";
      ma.markers.push_back(perp_legend_marker);
    }

    // Edges
    visualization_msgs::msg::Marker edges;
    edges.header = h;
    edges.ns = ns_ + "_edges";
    edges.id = 1;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.action = visualization_msgs::msg::Marker::ADD;
    edges.scale.x = 0.02;
    edges.color.r = 1.0f;
    edges.color.a = 1.0f;
    for (size_t i = 0; i < edges_.size(); ++i) {
      geometry_msgs::msg::Point p1, p2;
      int idx_from = voro_inds_[i];
      p1.x = origin_x_ + (idx_from % width_) * resolution_;
      p1.y = origin_y_ + (idx_from / width_) * resolution_;
      p1.z = 0.0;
      for (int j : edges_[i]) {
        int idx_to = voro_inds_[j];
        p2.x = origin_x_ + (idx_to % width_) * resolution_;
        p2.y = origin_y_ + (idx_to / width_) * resolution_;
        p2.z = 0.0;
        edges.points.push_back(p1);
        edges.points.push_back(p2);
      }
    }
    ma.markers.push_back(edges);

    pub_markers_->publish(ma);
  }

  void publishGvdGrid(const std_msgs::msg::Header &h) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header = h;
    grid.info.resolution = resolution_;
    grid.info.width = width_;
    grid.info.height = height_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(width_ * height_, 0);
    for (int idx : voro_inds_) {
      if (idx >= 0 && idx < static_cast<int>(grid.data.size())) {
        grid.data[idx] = 100; // mark GVD ridge as occupied for visualization
      }
    }
    pub_gvd_grid_->publish(grid);
  }
  
  // Plan waypoint tour: 0,0 -> waypoint1 -> waypoint2 -> ... -> waypointN
  void planWaypointTour(const std_msgs::msg::Header& header) {
    if (waypoints_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No waypoints to plan tour");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Planning waypoint tour with %zu waypoints", waypoints_.size());
    
    std::vector<std::pair<double, double>> full_tour_path;
    used_gvd_nodes_.clear(); // Clear previous path nodes
    
    // Start from map origin (0, 0)
    full_tour_path.emplace_back(0.0, 0.0);
    
    // Track the current GVD node (starts at origin's nearest GVD node)
    int current_gvd_node = -1;
    
    // Initialize current GVD node from origin
    int origin_idx = worldToIndex(0.0, 0.0);
    current_gvd_node = findNearestNode(origin_idx);
    RCLCPP_INFO(this->get_logger(), "Starting from origin GVD node: %d", current_gvd_node);
    
    // Plan path to each waypoint in sequence, connecting them continuously
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& waypoint = waypoints_[i];
      RCLCPP_INFO(this->get_logger(), "Planning to waypoint %zu: (%.3f, %.3f)", i+1, waypoint.x, waypoint.y);
      
      // Start from current GVD node (can be reused)
      int s_node = current_gvd_node;
      
      // Find goal GVD node
      int goal_idx = worldToIndex(waypoint.x, waypoint.y);
      int g_node = findNearestNode(goal_idx);
      
      RCLCPP_INFO(this->get_logger(), "Waypoint %zu: Start GVD node %d, Goal GVD node %d", 
                  i+1, s_node, g_node);
      
      if (s_node == g_node) {
        RCLCPP_WARN(this->get_logger(), "Start and goal nodes are the same for waypoint %zu", i+1);
        // No need to add anything, already at the right GVD node
        current_gvd_node = g_node;
      } else {
        auto raw_path = aStar(s_node, g_node);
        if (raw_path.empty()) {
          RCLCPP_ERROR(this->get_logger(), "A* path planning failed for waypoint %zu - maintaining current GVD node", i+1);
          // Keep current_gvd_node unchanged when path planning fails
          continue;
        }
        
        // Add used GVD nodes to visualization list
        for (int node_id : raw_path) {
          used_gvd_nodes_.push_back(node_id);
        }
        
        auto smooth_pts = projectAndSmooth(raw_path);
        
        // Add GVD path to full tour (skip first point to avoid duplication)
        for (size_t j = 1; j < smooth_pts.size(); ++j) {
          full_tour_path.push_back(smooth_pts[j]);
        }
        
        // Update current GVD node to the goal node for next iteration
        current_gvd_node = g_node;
      }
    }
    
    // Add return path to origin (0, 0) after visiting all waypoints
    RCLCPP_INFO(this->get_logger(), "Adding return path to origin (0, 0)");
    
    int origin_gvd_node = findNearestNode(origin_idx);
    
    if (current_gvd_node != origin_gvd_node) {
      auto return_path = aStar(current_gvd_node, origin_gvd_node);
      if (return_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "A* path planning failed for return to origin!");
      } else {
        // Add used GVD nodes to visualization list
        for (int node_id : return_path) {
          used_gvd_nodes_.push_back(node_id);
        }
        
        auto smooth_return_pts = projectAndSmooth(return_path);
        
        // Add return path to full tour (skip first point to avoid duplication)
        for (size_t k = 1; k < smooth_return_pts.size(); ++k) {
          full_tour_path.push_back(smooth_return_pts[k]);
        }
        
        RCLCPP_INFO(this->get_logger(), "Return path added with %zu points", smooth_return_pts.size());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Already at origin, no return path needed");
    }
    
    // Publish the complete waypoint tour
    publishWaypointPath(header, full_tour_path);
    
    RCLCPP_INFO(this->get_logger(), "Waypoint tour completed with %zu total points", full_tour_path.size());
  }
  
  void publishWaypointPath(const std_msgs::msg::Header& h, const std::vector<std::pair<double, double>>& pts) {
    if (pts.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot publish empty waypoint path");
      return;
    }
    
    nav_msgs::msg::Path path;
    path.header = h;
    path.header.frame_id = h.frame_id;
    
    for (auto &[x, y] : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = h;
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing waypoint tour with %zu points from (%.3f, %.3f) to (%.3f, %.3f)", 
                pts.size(), pts[0].first, pts[0].second, pts.back().first, pts.back().second);
    
    pub_waypoint_path_->publish(path);
  }
  
  // Generate artificial intersections at 1.5m intervals when no real intersections are found
  std::vector<std::pair<geometry_msgs::msg::Point, std::string>> generateArtificialIntersections(const TreeRow& row) {
    std::vector<std::pair<geometry_msgs::msg::Point, std::string>> artificial_intersections;
    
    // Calculate row direction (PCA direction)
    double row_dx = std::cos(row.orientation);
    double row_dy = std::sin(row.orientation);
    
    // Calculate perpendicular direction (90 degrees rotated)
    double perp_dx = -row_dy; // Perpendicular to row direction
    double perp_dy = row_dx;
    
    // Generate 4 artificial intersections at 1.5m intervals
    double interval = 1.5; // 1.5m intervals
    
    // Helper function to find nearest GVD node for artificial intersection
    auto findNearestGVDForPoint = [this](const geometry_msgs::msg::Point& point) -> geometry_msgs::msg::Point {
      int point_idx = worldToIndex(point.x, point.y);
      int nearest_gvd_node = findNearestNode(point_idx);
      
      if (nearest_gvd_node >= 0 && nearest_gvd_node < static_cast<int>(voro_inds_.size())) {
        int gvd_idx = voro_inds_[nearest_gvd_node];
        geometry_msgs::msg::Point gvd_point;
        gvd_point.x = origin_x_ + (gvd_idx % width_) * resolution_;
        gvd_point.y = origin_y_ + (gvd_idx / width_) * resolution_;
        gvd_point.z = point.z;
        return gvd_point;
      }
      return point; // Fallback to original point if no GVD node found
    };
    
    // 1. Start right (1.5m from start point)
    geometry_msgs::msg::Point start_right_raw;
    start_right_raw.x = row.start_point.x + perp_dx * interval;
    start_right_raw.y = row.start_point.y + perp_dy * interval;
    start_right_raw.z = row.start_point.z;
    geometry_msgs::msg::Point start_right = findNearestGVDForPoint(start_right_raw);
    artificial_intersections.push_back({start_right, "start_right"});
    
    // 2. End right (1.5m from end point)
    geometry_msgs::msg::Point end_right_raw;
    end_right_raw.x = row.end_point.x + perp_dx * interval;
    end_right_raw.y = row.end_point.y + perp_dy * interval;
    end_right_raw.z = row.end_point.z;
    geometry_msgs::msg::Point end_right = findNearestGVDForPoint(end_right_raw);
    artificial_intersections.push_back({end_right, "end_right"});
    
    // 3. End left (1.5m from end point, opposite direction)
    geometry_msgs::msg::Point end_left_raw;
    end_left_raw.x = row.end_point.x - perp_dx * interval;
    end_left_raw.y = row.end_point.y - perp_dy * interval;
    end_left_raw.z = row.end_point.z;
    geometry_msgs::msg::Point end_left = findNearestGVDForPoint(end_left_raw);
    artificial_intersections.push_back({end_left, "end_left"});
    
    // 4. Start left (1.5m from start point, opposite direction)
    geometry_msgs::msg::Point start_left_raw;
    start_left_raw.x = row.start_point.x - perp_dx * interval;
    start_left_raw.y = row.start_point.y - perp_dy * interval;
    start_left_raw.z = row.start_point.z;
    geometry_msgs::msg::Point start_left = findNearestGVDForPoint(start_left_raw);
    artificial_intersections.push_back({start_left, "start_left"});
    
    RCLCPP_INFO(this->get_logger(), "Generated artificial intersections for row:");
    RCLCPP_INFO(this->get_logger(), "  Row orientation: %.3f rad (%.1f deg)", row.orientation, row.orientation * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  Perpendicular direction: (%.3f, %.3f)", perp_dx, perp_dy);
    RCLCPP_INFO(this->get_logger(), "  Interval distance: %.3f m", interval);
    RCLCPP_INFO(this->get_logger(), "  Start right: raw(%.3f, %.3f) -> GVD(%.3f, %.3f)", start_right_raw.x, start_right_raw.y, start_right.x, start_right.y);
    RCLCPP_INFO(this->get_logger(), "  End right: raw(%.3f, %.3f) -> GVD(%.3f, %.3f)", end_right_raw.x, end_right_raw.y, end_right.x, end_right.y);
    RCLCPP_INFO(this->get_logger(), "  End left: raw(%.3f, %.3f) -> GVD(%.3f, %.3f)", end_left_raw.x, end_left_raw.y, end_left.x, end_left.y);
    RCLCPP_INFO(this->get_logger(), "  Start left: raw(%.3f, %.3f) -> GVD(%.3f, %.3f)", start_left_raw.x, start_left_raw.y, start_left.x, start_left.y);
    RCLCPP_INFO(this->get_logger(), "  Total artificial intersections: %zu", artificial_intersections.size());
    
    return artificial_intersections;
  }

  // Find intersection points between perpendicular lines from tree row endpoints and GVD edges
  std::vector<std::pair<geometry_msgs::msg::Point, std::string>> findTreeRowGVDIntersections(const TreeRow& row) {
    std::vector<std::pair<geometry_msgs::msg::Point, std::string>> intersections;
    
    // Calculate row direction (PCA direction)
    double row_dx = std::cos(row.orientation);
    double row_dy = std::sin(row.orientation);
    
    // Calculate perpendicular direction (90 degrees rotated)
    double perp_dx = -row_dy; // Perpendicular to row direction
    double perp_dy = row_dx;
    
    // Extension distance for perpendicular lines
    double extension_distance = 5.0; // Extend 10 meters in each perpendicular direction
    double min_distance_from_endpoints = 1.0; // Minimum distance from start/end points
    
    // 1. Perpendicular line from start point (left and right)
    geometry_msgs::msg::Point start_left;
    start_left.x = row.start_point.x + perp_dx * extension_distance;
    start_left.y = row.start_point.y + perp_dy * extension_distance;
    start_left.z = row.start_point.z;
    
    geometry_msgs::msg::Point start_right;
    start_right.x = row.start_point.x - perp_dx * extension_distance;
    start_right.y = row.start_point.y - perp_dy * extension_distance;
    start_right.z = row.start_point.z;
    
    // 2. Perpendicular line from end point (left and right)
    geometry_msgs::msg::Point end_left;
    end_left.x = row.end_point.x + perp_dx * extension_distance;
    end_left.y = row.end_point.y + perp_dy * extension_distance;
    end_left.z = row.end_point.z;
    
    geometry_msgs::msg::Point end_right;
    end_right.x = row.end_point.x - perp_dx * extension_distance;
    end_right.y = row.end_point.y - perp_dy * extension_distance;
    end_right.z = row.end_point.z;
    
    RCLCPP_INFO(this->get_logger(), "Tree row orientation: %.3f rad (%.1f deg), perpendicular: %.3f rad (%.1f deg)", 
                row.orientation, row.orientation * 180.0 / M_PI, 
                std::atan2(perp_dy, perp_dx), std::atan2(perp_dy, perp_dx) * 180.0 / M_PI);
    
    RCLCPP_INFO(this->get_logger(), "Tree row start: (%.3f, %.3f), end: (%.3f, %.3f)", 
                row.start_point.x, row.start_point.y, row.end_point.x, row.end_point.y);
    
    RCLCPP_INFO(this->get_logger(), "Perpendicular line endpoints:");
    RCLCPP_INFO(this->get_logger(), "  Start left: (%.3f, %.3f) -> (%.3f, %.3f)", 
                row.start_point.x, row.start_point.y, start_left.x, start_left.y);
    RCLCPP_INFO(this->get_logger(), "  Start right: (%.3f, %.3f) -> (%.3f, %.3f)", 
                row.start_point.x, row.start_point.y, start_right.x, start_right.y);
    RCLCPP_INFO(this->get_logger(), "  End left: (%.3f, %.3f) -> (%.3f, %.3f)", 
                row.end_point.x, row.end_point.y, end_left.x, end_left.y);
    RCLCPP_INFO(this->get_logger(), "  End right: (%.3f, %.3f) -> (%.3f, %.3f)", 
                row.end_point.x, row.end_point.y, end_right.x, end_right.y);
    
    // Find intersections for each perpendicular line
    std::vector<geometry_msgs::msg::Point> start_left_intersections = findLineGVDIntersections(row.start_point, start_left);
    std::vector<geometry_msgs::msg::Point> start_right_intersections = findLineGVDIntersections(row.start_point, start_right);
    std::vector<geometry_msgs::msg::Point> end_left_intersections = findLineGVDIntersections(row.end_point, end_left);
    std::vector<geometry_msgs::msg::Point> end_right_intersections = findLineGVDIntersections(row.end_point, end_right);
    
    // Find the first valid intersection for each perpendicular line
    geometry_msgs::msg::Point start_left_intersection = findFirstIntersectionAfterDistance(
        start_left_intersections, row.start_point, min_distance_from_endpoints, 1.0);
    geometry_msgs::msg::Point start_right_intersection = findFirstIntersectionAfterDistance(
        start_right_intersections, row.start_point, min_distance_from_endpoints, 1.0);
    geometry_msgs::msg::Point end_left_intersection = findFirstIntersectionAfterDistance(
        end_left_intersections, row.end_point, min_distance_from_endpoints, 1.0);
    geometry_msgs::msg::Point end_right_intersection = findFirstIntersectionAfterDistance(
        end_right_intersections, row.end_point, min_distance_from_endpoints, 1.0);
    
    // Store perpendicular lines for visualization
    perpendicular_lines_start_left_.push_back(row.start_point);
    perpendicular_lines_start_left_.push_back(start_left);
    
    perpendicular_lines_start_right_.push_back(row.start_point);
    perpendicular_lines_start_right_.push_back(start_right);
    
    perpendicular_lines_end_left_.push_back(row.end_point);
    perpendicular_lines_end_left_.push_back(end_left);
    
    perpendicular_lines_end_right_.push_back(row.end_point);
    perpendicular_lines_end_right_.push_back(end_right);
    
    // Add valid intersections with their types
    // Check if intersection is valid by checking if it's not the default (0,0,0) point
    // or if it's at a reasonable distance from the reference point
    double min_valid_distance = 0.1; // Minimum distance to consider intersection valid
    
    if (std::hypot(start_left_intersection.x - row.start_point.x, start_left_intersection.y - row.start_point.y) >= min_valid_distance) {
      intersections.push_back({start_left_intersection, "start_left"});
      RCLCPP_INFO(this->get_logger(), "Start left intersection: (%.3f, %.3f)", 
                  start_left_intersection.x, start_left_intersection.y);
    }
    
    if (std::hypot(start_right_intersection.x - row.start_point.x, start_right_intersection.y - row.start_point.y) >= min_valid_distance) {
      intersections.push_back({start_right_intersection, "start_right"});
      RCLCPP_INFO(this->get_logger(), "Start right intersection: (%.3f, %.3f)", 
                  start_right_intersection.x, start_right_intersection.y);
    }
    
    if (std::hypot(end_left_intersection.x - row.end_point.x, end_left_intersection.y - row.end_point.y) >= min_valid_distance) {
      intersections.push_back({end_left_intersection, "end_left"});
      RCLCPP_INFO(this->get_logger(), "End left intersection: (%.3f, %.3f)", 
                  end_left_intersection.x, end_left_intersection.y);
    }
    
    if (std::hypot(end_right_intersection.x - row.end_point.x, end_right_intersection.y - row.end_point.y) >= min_valid_distance) {
      intersections.push_back({end_right_intersection, "end_right"});
      RCLCPP_INFO(this->get_logger(), "End right intersection: (%.3f, %.3f)", 
                  end_right_intersection.x, end_right_intersection.y);
    }
    
    RCLCPP_INFO(this->get_logger(), "Found %zu intersections for tree row (start_left: %s, start_right: %s, end_left: %s, end_right: %s)", 
                intersections.size(),
                (std::hypot(start_left_intersection.x - row.start_point.x, start_left_intersection.y - row.start_point.y) >= min_valid_distance) ? "yes" : "no",
                (std::hypot(start_right_intersection.x - row.start_point.x, start_right_intersection.y - row.start_point.y) >= min_valid_distance) ? "yes" : "no",
                (std::hypot(end_left_intersection.x - row.end_point.x, end_left_intersection.y - row.end_point.y) >= min_valid_distance) ? "yes" : "no",
                (std::hypot(end_right_intersection.x - row.end_point.x, end_right_intersection.y - row.end_point.y) >= min_valid_distance) ? "yes" : "no");
    
    return intersections;
  }
  
  // Find the first intersection after a minimum distance from the reference point
  geometry_msgs::msg::Point findFirstIntersectionAfterDistance(
      const std::vector<geometry_msgs::msg::Point>& intersections,
      const geometry_msgs::msg::Point& reference_point,
      double min_distance,
      double /* direction */) {
    
    geometry_msgs::msg::Point result; // Default (0,0,0) means no intersection found
    bool found_valid = false;
    
    for (const auto& intersection : intersections) {
      double distance = std::hypot(intersection.x - reference_point.x, intersection.y - reference_point.y);
      
      // Check if intersection is at least min_distance away from reference point
      if (distance >= min_distance) {
        // For backward direction (-1.0), we want the closest intersection that's far enough
        // For forward direction (1.0), we want the closest intersection that's far enough
        if (!found_valid) {
          result = intersection; // First valid intersection
          found_valid = true;
        } else {
          // Keep the closest valid intersection
          double current_distance = std::hypot(result.x - reference_point.x, result.y - reference_point.y);
          if (distance < current_distance) {
            result = intersection;
          }
        }
      }
    }
    
    return result;
  }
  
  // Find GVD intersections along a line segment
  std::vector<geometry_msgs::msg::Point> findLineGVDIntersections(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end) {
    std::vector<geometry_msgs::msg::Point> intersections;
    
    // Sample points along the line segment
    int num_samples = static_cast<int>(std::ceil(std::hypot(end.x - start.x, end.y - start.y) / 0.05)); // Sample every 0.2m
    num_samples = std::max(10, num_samples); // At least 10 samples
    
    for (int i = 0; i <= num_samples; ++i) {
      double t = static_cast<double>(i) / num_samples;
      geometry_msgs::msg::Point sample_point;
      sample_point.x = start.x + t * (end.x - start.x);
      sample_point.y = start.y + t * (end.y - start.y);
      sample_point.z = start.z + t * (end.z - start.z);
      
      // Find nearest GVD node
      int point_idx = worldToIndex(sample_point.x, sample_point.y);
      int nearest_gvd_node = findNearestNode(point_idx);
      
      if (nearest_gvd_node >= 0 && nearest_gvd_node < static_cast<int>(voro_inds_.size())) {
        // Get the GVD node position
        int gvd_idx = voro_inds_[nearest_gvd_node];
        double gvd_x = origin_x_ + (gvd_idx % width_) * resolution_;
        double gvd_y = origin_y_ + (gvd_idx / width_) * resolution_;
        
        // Check if this GVD node is close enough to be considered an intersection
        double distance = std::hypot(sample_point.x - gvd_x, sample_point.y - gvd_y);
        if (distance < 0.3) { // Within 1 meter of the line
          geometry_msgs::msg::Point intersection;
          intersection.x = gvd_x;
          intersection.y = gvd_y;
          intersection.z = sample_point.z;
          intersections.push_back(intersection);
        }
      }
    }
    
    return intersections;
  }
  
  
  // Plan orchard exploration using tree rows
  void planOrchardExploration(const std_msgs::msg::Header& header) {
    if (tree_rows_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No tree rows available for orchard exploration");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Planning orchard exploration with %zu tree rows", tree_rows_.size());
    
    std::vector<std::pair<double, double>> full_exploration_path;
    used_gvd_nodes_.clear();
    gvd_intersections_.clear(); // Clear previous intersections
    gvd_intersection_visit_order_.clear(); // Clear previous visit orders
    gvd_intersection_types_.clear(); // Clear previous intersection types
    
    // Clear previous perpendicular lines
    perpendicular_lines_start_left_.clear();
    perpendicular_lines_start_right_.clear();
    perpendicular_lines_end_left_.clear();
    perpendicular_lines_end_right_.clear();
    
    // Start from map origin (0, 0)
    full_exploration_path.emplace_back(0.0, 0.0);
    
    // Sort tree rows by their Y position (for bottom-to-top exploration)
    std::vector<TreeRow> sorted_rows = tree_rows_;
    std::sort(sorted_rows.begin(), sorted_rows.end(), [](const TreeRow& a, const TreeRow& b) {
      return a.start_point.y < b.start_point.y; // Sort by Y ascending (bottom to top)
    });
    
    int current_gvd_node = -1;
    
    // Initialize current GVD node from origin
    int origin_idx = worldToIndex(0.0, 0.0);
    current_gvd_node = findNearestNode(origin_idx);
    RCLCPP_INFO(this->get_logger(), "Starting from origin GVD node: %d", current_gvd_node);
    
    int global_visit_order = 1; // Global visit order counter
    std::vector<geometry_msgs::msg::Point> visited_intersections; // Track visited intersections to avoid duplicates
    double min_distance_threshold = 1.0; // Minimum distance between intersections to consider them different (in meters)
    
    // Process each tree row
    for (size_t i = 0; i < sorted_rows.size(); ++i) {
      const auto& row = sorted_rows[i];
      
      // Check if tree row is within exploration area
      if (!isTreeRowInExplorationArea(row)) {
        RCLCPP_INFO(this->get_logger(), "Skipping tree row %zu - outside exploration area", i+1);
        continue;
      }
      
      std::string pattern = (i % 2 == 0) ? "start_right->end_right->end_left->start_left" : "end_right->start_right->start_left->end_left";
      RCLCPP_INFO(this->get_logger(), "Processing tree row %zu: %d trees, orientation: %.3f rad, pattern: %s", 
                  i+1, row.tree_count, row.orientation, pattern.c_str());
      
      // Find GVD intersections for this tree row
      auto intersections = findTreeRowGVDIntersections(row);
      RCLCPP_INFO(this->get_logger(), "Found %zu GVD intersections for row %zu", intersections.size(), i+1);
      
      if (intersections.empty()) {
        RCLCPP_WARN(this->get_logger(), "No GVD intersections found for tree row %zu - generating artificial intersections", i+1);
        
        // Generate artificial intersections at 1.5m intervals along perpendicular lines
        intersections = generateArtificialIntersections(row);
        RCLCPP_INFO(this->get_logger(), "Generated %zu artificial intersections for row %zu", intersections.size(), i+1);
      }
      
      // Sort intersections by type and position - alternating pattern based on row index
      std::sort(intersections.begin(), intersections.end(), [i](const std::pair<geometry_msgs::msg::Point, std::string>& a, const std::pair<geometry_msgs::msg::Point, std::string>& b) {
        // Define priority order for intersection types - alternating pattern
        std::map<std::string, int> type_priority;
        
        if (i % 2 == 0) {
          // Even rows (0, 2, 4...): start_right -> end_right -> end_left -> start_left
          type_priority = {
            {"start_right", 1},
            {"end_right", 2}, 
            {"end_left", 3},
            {"start_left", 4}
          };
        } else {
          // Odd rows (1, 3, 5...): end_right -> start_right -> start_left -> end_left
          type_priority = {
            {"end_right", 1},
            {"start_right", 2},
            {"start_left", 3},
            {"end_left", 4}
          };
        }
        
        int priority_a = type_priority[a.second];
        int priority_b = type_priority[b.second];
        
        if (priority_a != priority_b) {
          return priority_a < priority_b; // Lower priority number first
        }
        
        // If same type, sort by X position (left to right)
        return a.first.x < b.first.x;
      });
      
      // Plan path to each intersection in sequence
      for (size_t j = 0; j < intersections.size(); ++j) {
        const auto& intersection_pair = intersections[j];
        const auto& intersection = intersection_pair.first;
        const auto& intersection_type = intersection_pair.second;
        
        // Check if this intersection is too close to any previously visited intersection
        bool too_close = false;
        for (const auto& visited : visited_intersections) {
          double distance = std::hypot(intersection.x - visited.x, intersection.y - visited.y);
          if (distance < min_distance_threshold) {
            too_close = true;
            RCLCPP_INFO(this->get_logger(), "Skipping intersection %zu (type %s) at (%.3f, %.3f) - too close to visited intersection at (%.3f, %.3f), distance: %.3f m", 
                        j+1, intersection_type.c_str(), intersection.x, intersection.y, visited.x, visited.y, distance);
            break;
          }
        }
        
        if (too_close) {
          continue; // Skip this intersection
        }
        
        RCLCPP_INFO(this->get_logger(), "Planning to intersection %zu (global order %d, type %s): (%.3f, %.3f)", 
                    j+1, global_visit_order, intersection_type.c_str(), intersection.x, intersection.y);
        
        // Add intersection to global list with visit order and type
        gvd_intersections_.push_back(intersection);
        gvd_intersection_visit_order_.push_back(global_visit_order);
        gvd_intersection_types_.push_back(intersection_type);
        visited_intersections.push_back(intersection); // Track this intersection as visited
        global_visit_order++;
        
        int s_node = current_gvd_node;
        int goal_idx = worldToIndex(intersection.x, intersection.y);
        int g_node = findNearestNode(goal_idx);
        
        if (s_node == g_node) {
          RCLCPP_WARN(this->get_logger(), "Start and goal nodes are the same for intersection %zu", j+1);
          current_gvd_node = g_node;
        } else {
          auto raw_path = aStar(s_node, g_node);
          if (raw_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "A* path planning failed for intersection %zu - maintaining current GVD node", j+1);
            // Keep current_gvd_node unchanged when path planning fails
            continue;
          }
          
          // Add used GVD nodes to visualization list
          for (int node_id : raw_path) {
            used_gvd_nodes_.push_back(node_id);
          }
          
          auto smooth_pts = projectAndSmooth(raw_path);
          
          // Add GVD path to full exploration (skip first point to avoid duplication)
          for (size_t k = 1; k < smooth_pts.size(); ++k) {
            full_exploration_path.push_back(smooth_pts[k]);
          }
          
          // Update current GVD node
          current_gvd_node = g_node;
        }
      }
    }
    
    // Add return path to origin (0, 0) after visiting all intersections
    RCLCPP_INFO(this->get_logger(), "Adding return path to origin (0, 0)");
    
    int origin_gvd_node = findNearestNode(origin_idx);
    
    if (current_gvd_node != origin_gvd_node) {
      auto return_path = aStar(current_gvd_node, origin_gvd_node);
      if (return_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "A* path planning failed for return to origin!");
      } else {
        // Add used GVD nodes to visualization list
        for (int node_id : return_path) {
          used_gvd_nodes_.push_back(node_id);
        }
        
        auto smooth_return_pts = projectAndSmooth(return_path);
        
        // Add return path to full exploration (skip first point to avoid duplication)
        for (size_t k = 1; k < smooth_return_pts.size(); ++k) {
          full_exploration_path.push_back(smooth_return_pts[k]);
        }
        
        RCLCPP_INFO(this->get_logger(), "Return path added with %zu points", smooth_return_pts.size());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Already at origin, no return path needed");
    }
    
    // Publish the complete orchard exploration path
    publishWaypointPath(header, full_exploration_path);
    
    RCLCPP_INFO(this->get_logger(), "Orchard exploration completed with %zu total points", full_exploration_path.size());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GvdTopologyNode>());
  rclcpp::shutdown();
  return 0;
}
