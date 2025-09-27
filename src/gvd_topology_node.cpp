#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
    this->declare_parameter("connect_radius", 5);
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

    // Publishers
    pub_path_ = create_publisher<nav_msgs::msg::Path>(ns_ + "/path", 10);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(ns_ + "/markers", 10);
    pub_gvd_grid_ = create_publisher<nav_msgs::msg::OccupancyGrid>(ns_ + "/gvd_grid", 10);

    // State
    goal_defined_ = false;
  }

private:
  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_rviz1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_rviz2_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_gvd_grid_;
  rclcpp::QoS qos_odom;

  // Params
  int connect_radius_ = 5; // in cells
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

  // Callbacks
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_defined_ = true;
    RCLCPP_INFO(this->get_logger(), "GVD goal set: (%.3f, %.3f)", goal_x_, goal_y_);
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

    // 6) If goal set, plan path on graph from nearest to origin to nearest to goal
    if (goal_defined_) {
      int start_idx = worldToIndex(origin_x_, origin_y_); // using map origin as start placeholder
      int goal_idx = worldToIndex(goal_x_, goal_y_);
      int s_node = findNearestNode(start_idx);
      int g_node = findNearestNode(goal_idx);
      auto raw_path = aStar(s_node, g_node);
      auto smooth_pts = projectAndSmooth(raw_path);
      publishPath(msg->header, smooth_pts);
    }

    // 7) Publish markers for nodes and edges
    publishMarkers(msg->header);

    // 8) Optionally publish GVD occupancy grid for visualization
    if (publish_gvd_grid_) {
      publishGvdGrid(msg->header);
    }
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
    for (int idx = 0; idx < width_ * height_; ++idx) {
      if (obstacle_[idx] || dist_map_[idx] == 0 || dist_map_[idx] == INF_)
        continue;
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
      }
    }
  }

  void buildGraph() {
    int M = static_cast<int>(voro_inds_.size());
    edges_.assign(M, {});
    for (int i = 0; i < M; ++i) {
      int idx = voro_inds_[i];
      int x = idx % width_;
      int y = idx / width_;
      for (int dy = -connect_radius_; dy <= connect_radius_; ++dy) {
        for (int dx = -connect_radius_; dx <= connect_radius_; ++dx) {
          if (dx == 0 && dy == 0)
            continue;
          if (std::max(std::abs(dx), std::abs(dy)) > connect_radius_)
            continue;
          int nx = x + dx;
          int ny = y + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
            continue;
          int nidx = ny * width_ + nx;
          auto it = idx2node_.find(nidx);
          if (it != idx2node_.end()) {
            edges_[i].push_back(it->second);
          }
        }
      }
    }
  }

  int worldToIndex(double wx, double wy) const {
    int mx = static_cast<int>((wx - origin_x_) / resolution_);
    int my = static_cast<int>((wy - origin_y_) / resolution_);
    return my * width_ + mx;
  }

  int findNearestNode(int idx) const {
    if (voro_inds_.empty())
      return 0;
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
    std::vector<double> gScore(M, std::numeric_limits<double>::infinity());
    std::vector<double> fScore(M, std::numeric_limits<double>::infinity());
    std::vector<int> cameFrom(M, -1);
    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    gScore[s] = 0.0;
    fScore[s] = heuristic(s, g);
    open.push({s, fScore[s]});
    while (!open.empty()) {
      auto top = open.top();
      open.pop();
      int u = top.id;
      if (u == g)
        break;
      for (int v : edges_[u]) {
        double tentative = gScore[u] + edgeCost(u, v);
        if (tentative < gScore[v]) {
          cameFrom[v] = u;
          gScore[v] = tentative;
          fScore[v] = tentative + heuristic(v, g);
          open.push({v, fScore[v]});
        }
      }
    }
    std::vector<int> path;
    for (int cur = g; cur != -1; cur = cameFrom[cur])
      path.push_back(cur);
    std::reverse(path.begin(), path.end());
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
    nav_msgs::msg::Path path;
    path.header = h;
    for (auto &[x, y] : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = h;
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);
  }

  void publishMarkers(const std_msgs::msg::Header &h) {
    visualization_msgs::msg::MarkerArray ma;

    // Nodes
    visualization_msgs::msg::Marker nodes;
    nodes.header = h;
    nodes.ns = ns_ + "_nodes";
    nodes.id = 0;
    nodes.type = visualization_msgs::msg::Marker::POINTS;
    nodes.action = visualization_msgs::msg::Marker::ADD;
    nodes.scale.x = 0.05;
    nodes.scale.y = 0.05;
    nodes.color.g = 1.0f;
    nodes.color.a = 1.0f;
    for (int idx : voro_inds_) {
      geometry_msgs::msg::Point p;
      p.x = origin_x_ + (idx % width_) * resolution_;
      p.y = origin_y_ + (idx / width_) * resolution_;
      p.z = 0.0;
      nodes.points.push_back(p);
    }
    ma.markers.push_back(nodes);

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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GvdTopologyNode>());
  rclcpp::shutdown();
  return 0;
}
