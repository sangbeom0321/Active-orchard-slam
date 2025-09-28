#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <algorithm>

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

  std::vector<std::pair<double, double>> projectAndSmooth(const std::vector<int> &nodes) const {
    std::vector<std::pair<double, double>> P;
    P.reserve(nodes.size());
    for (int id : nodes) {
      int idx = voro_inds_[id];
      double x = origin_x_ + (idx % width_) * resolution_;
      double y = origin_y_ + (idx / width_) * resolution_;
      P.emplace_back(x, y);
    }
    
    RCLCPP_INFO(this->get_logger(), "Projected %zu nodes to world coordinates", P.size());
    
    // simple smoothing
    auto S = P;
    const double w_data = 0.5, w_smooth = 0.3;
    for (int iter = 0; iter < 10; ++iter) {
      for (size_t i = 1; i + 1 < P.size(); ++i) {
        S[i].first =
            (w_data * P[i].first + w_smooth * (S[i - 1].first + S[i + 1].first)) / (w_data + 2.0 * w_smooth);
        S[i].second =
            (w_data * P[i].second + w_smooth * (S[i - 1].second + S[i + 1].second)) / (w_data + 2.0 * w_smooth);
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
          RCLCPP_ERROR(this->get_logger(), "A* path planning failed for waypoint %zu!", i+1);
          current_gvd_node = g_node;
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GvdTopologyNode>());
  rclcpp::shutdown();
  return 0;
}
