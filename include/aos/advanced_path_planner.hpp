#ifndef ORBIT_PLANNER_ADVANCED_PATH_PLANNER_HPP
#define ORBIT_PLANNER_ADVANCED_PATH_PLANNER_HPP

#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

namespace orbit_planner
{

/**
 * @brief Advanced path planning algorithms for orbit exploration
 * 
 * This class provides multiple path planning algorithms optimized for
 * orchard environments, including RRT*, A* with dynamic weights,
 * and Voronoi-based planning.
 */
class AdvancedPathPlanner
{
public:
  /**
   * @brief Planning algorithm types
   */
  enum class AlgorithmType
  {
    A_STAR,
    RRT_STAR,
    VORONOI,
    HYBRID
  };

  /**
   * @brief Path planning result with additional metadata
   */
  struct PlanningResult
  {
    nav_msgs::msg::Path path;
    bool success;
    double path_length;
    double clearance;
    double curvature;
    double computation_time;
    AlgorithmType algorithm_used;
    
    PlanningResult()
      : success(false)
      , path_length(0.0)
      , clearance(0.0)
      , curvature(0.0)
      , computation_time(0.0)
      , algorithm_used(AlgorithmType::A_STAR)
    {}
  };

  /**
   * @brief Configuration parameters for path planning
   */
  struct PlanningConfig
  {
    double robot_radius;
    double safety_margin;
    double max_curvature;
    double path_resolution;
    double max_planning_time;
    bool enable_smoothing;
    bool enable_optimization;
    int max_iterations;
    double goal_tolerance;
    
    PlanningConfig()
      : robot_radius(0.4)
      , safety_margin(0.1)
      , max_curvature(0.5)
      , path_resolution(0.5)
      , max_planning_time(5.0)
      , enable_smoothing(true)
      , enable_optimization(true)
      , max_iterations(1000)
      , goal_tolerance(1.0)
    {}
  };

  /**
   * @brief Constructor
   * @param config Planning configuration
   */
  explicit AdvancedPathPlanner(const PlanningConfig & config = PlanningConfig());

  /**
   * @brief Destructor
   */
  ~AdvancedPathPlanner();

  /**
   * @brief Plan path using specified algorithm
   * @param start Start pose
   * @param goal Goal pose
   * @param algorithm Algorithm to use
   * @param distance_function Function to query obstacle distances
   * @return Planning result
   */
  PlanningResult planPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    AlgorithmType algorithm,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Plan path with automatic algorithm selection
   * @param start Start pose
   * @param goal Goal pose
   * @param distance_function Function to query obstacle distances
   * @return Planning result
   */
  PlanningResult planPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Smooth path using B-spline or Bezier curves
   * @param path Input path
   * @param distance_function Function to query obstacle distances
   * @return Smoothed path
   */
  nav_msgs::msg::Path smoothPath(
    const nav_msgs::msg::Path & path,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Optimize path for minimal curvature and length
   * @param path Input path
   * @param distance_function Function to query obstacle distances
   * @return Optimized path
   */
  nav_msgs::msg::Path optimizePath(
    const nav_msgs::msg::Path & path,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Generate Voronoi diagram for the environment
   * @param distance_function Function to query obstacle distances
   * @return Voronoi edges as path segments
   */
  std::vector<nav_msgs::msg::Path> generateVoronoiDiagram(
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Check if path is collision-free
   * @param path Path to check
   * @param distance_function Function to query obstacle distances
   * @return true if path is safe
   */
  bool isPathCollisionFree(
    const nav_msgs::msg::Path & path,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Calculate path metrics
   * @param path Path to analyze
   * @return Path metrics (length, curvature, clearance)
   */
  struct PathMetrics
  {
    double length;
    double max_curvature;
    double avg_curvature;
    double min_clearance;
    double avg_clearance;
    int waypoint_count;
  };
  
  PathMetrics calculatePathMetrics(const nav_msgs::msg::Path & path);

  /**
   * @brief Set planning configuration
   * @param config New configuration
   */
  void setConfig(const PlanningConfig & config);

  /**
   * @brief Get current configuration
   * @return Current configuration
   */
  const PlanningConfig & getConfig() const;

private:
  /**
   * @brief A* path planning implementation
   */
  PlanningResult planAStar(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief RRT* path planning implementation
   */
  PlanningResult planRRTStar(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Voronoi-based path planning implementation
   */
  PlanningResult planVoronoi(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Hybrid planning combining multiple algorithms
   */
  PlanningResult planHybrid(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Calculate heuristic for A* search
   */
  double calculateHeuristic(
    const geometry_msgs::msg::Point & current,
    const geometry_msgs::msg::Point & goal);

  /**
   * @brief Generate random sample for RRT*
   */
  geometry_msgs::msg::Point generateRandomSample(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & goal);

  /**
   * @brief Find nearest node in RRT tree
   */
  int findNearestNode(
    const std::vector<geometry_msgs::msg::Point> & nodes,
    const geometry_msgs::msg::Point & sample);

  /**
   * @brief Steer from one point to another
   */
  geometry_msgs::msg::Point steer(
    const geometry_msgs::msg::Point & from,
    const geometry_msgs::msg::Point & to,
    double max_distance);

  /**
   * @brief Find nearby nodes in RRT tree
   */
  std::vector<int> findNearbyNodes(
    const std::vector<geometry_msgs::msg::Point> & nodes,
    const geometry_msgs::msg::Point & sample,
    double radius);

  /**
   * @brief Rewire RRT tree
   */
  void rewireTree(
    std::vector<geometry_msgs::msg::Point> & nodes,
    std::vector<int> & parents,
    std::vector<double> & costs,
    int new_node_index,
    const std::vector<int> & nearby_nodes,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Calculate path curvature
   */
  double calculateCurvature(const nav_msgs::msg::Path & path, size_t index);

  /**
   * @brief Interpolate path with higher resolution
   */
  nav_msgs::msg::Path interpolatePath(
    const nav_msgs::msg::Path & path,
    double resolution);

  // Configuration
  PlanningConfig config_;
  
  // RRT* specific data structures
  struct RRTNode
  {
    geometry_msgs::msg::Point position;
    int parent;
    double cost;
    
    RRTNode() : parent(-1), cost(0.0) {}
  };
  
  std::vector<RRTNode> rrt_nodes_;
  std::vector<std::vector<int>> rrt_children_;
  
  // Voronoi diagram data
  struct VoronoiEdge
  {
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
    double clearance;
  };
  
  std::vector<VoronoiEdge> voronoi_edges_;
  
  // Performance monitoring
  std::chrono::steady_clock::time_point planning_start_time_;
  int total_planning_calls_;
  double total_planning_time_;
};

} // namespace orbit_planner

#endif // ORBIT_PLANNER_ADVANCED_PATH_PLANNER_HPP
