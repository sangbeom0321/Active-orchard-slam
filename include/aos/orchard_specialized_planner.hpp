#ifndef ORBIT_PLANNER_ORCHARD_SPECIALIZED_PLANNER_HPP
#define ORBIT_PLANNER_ORCHARD_SPECIALIZED_PLANNER_HPP

#include <memory>
#include <vector>
#include <unordered_map>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

namespace orbit_planner
{

/**
 * @brief Orchard-specific exploration planner
 * 
 * This planner is specifically designed for orchard environments and includes
 * features like row detection, tree counting, and systematic coverage patterns.
 */
class OrchardSpecializedPlanner
{
public:
  /**
   * @brief Orchard row structure
   */
  struct OrchardRow
  {
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
    double width;
    double length;
    int tree_count;
    bool explored;
    std::vector<geometry_msgs::msg::Point> tree_positions;
    
    OrchardRow() : width(0.0), length(0.0), tree_count(0), explored(false) {}
  };

  /**
   * @brief Tree detection result
   */
  struct TreeDetection
  {
    geometry_msgs::msg::Point position;
    double confidence;
    double estimated_diameter;
    bool verified;
    
    TreeDetection() : confidence(0.0), estimated_diameter(0.0), verified(false) {}
  };

  /**
   * @brief Exploration pattern types
   */
  enum class ExplorationPattern
  {
    ROW_BY_ROW,      // Explore row by row systematically
    SPIRAL,          // Spiral pattern from center
    ZIGZAG,          // Zigzag pattern across rows
    ADAPTIVE,        // Adaptive based on detected structure
    CUSTOM           // User-defined pattern
  };

  /**
   * @brief Orchard configuration
   */
  struct OrchardConfig
  {
    double expected_row_spacing;
    double expected_tree_spacing;
    double tree_diameter;
    double row_width;
    double exploration_margin;
    bool enable_tree_detection;
    bool enable_row_detection;
    ExplorationPattern pattern;
    double min_tree_confidence;
    int max_trees_per_row;
    
    OrchardConfig()
      : expected_row_spacing(3.0)
      , expected_tree_spacing(2.0)
      , tree_diameter(0.5)
      , row_width(1.0)
      , exploration_margin(0.5)
      , enable_tree_detection(true)
      , enable_row_detection(true)
      , pattern(ExplorationPattern::ADAPTIVE)
      , min_tree_confidence(0.7)
      , max_trees_per_row(50)
    {}
  };

  /**
   * @brief Constructor
   * @param config Orchard configuration
   */
  explicit OrchardSpecializedPlanner(const OrchardConfig & config = OrchardConfig());

  /**
   * @brief Destructor
   */
  ~OrchardSpecializedPlanner();

  /**
   * @brief Initialize orchard structure from point cloud
   * @param point_cloud Input point cloud
   * @param distance_function Function to query obstacle distances
   * @return true if initialization successful
   */
  bool initializeOrchardStructure(
    const std::vector<geometry_msgs::msg::Point> & point_cloud,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Detect orchard rows from point cloud
   * @param point_cloud Input point cloud
   * @return Detected rows
   */
  std::vector<OrchardRow> detectOrchardRows(
    const std::vector<geometry_msgs::msg::Point> & point_cloud);

  /**
   * @brief Detect individual trees
   * @param point_cloud Input point cloud
   * @return Detected trees
   */
  std::vector<TreeDetection> detectTrees(
    const std::vector<geometry_msgs::msg::Point> & point_cloud);

  /**
   * @brief Generate exploration waypoints for orchard
   * @param start_pose Starting pose
   * @param exploration_area Exploration polygon
   * @param distance_function Function to query obstacle distances
   * @return Generated waypoints
   */
  std::vector<geometry_msgs::msg::PoseStamped> generateOrchardWaypoints(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PolygonStamped & exploration_area,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Generate row-by-row exploration path
   * @param start_pose Starting pose
   * @param rows Detected orchard rows
   * @param distance_function Function to query obstacle distances
   * @return Exploration path
   */
  nav_msgs::msg::Path generateRowByRowPath(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const std::vector<OrchardRow> & rows,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Generate spiral exploration path
   * @param start_pose Starting pose
   * @param exploration_area Exploration polygon
   * @param distance_function Function to query obstacle distances
   * @return Exploration path
   */
  nav_msgs::msg::Path generateSpiralPath(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PolygonStamped & exploration_area,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Generate zigzag exploration path
   * @param start_pose Starting pose
   * @param rows Detected orchard rows
   * @param distance_function Function to query obstacle distances
   * @return Exploration path
   */
  nav_msgs::msg::Path generateZigzagPath(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const std::vector<OrchardRow> & rows,
    std::function<double(const geometry_msgs::msg::Point &)> distance_function);

  /**
   * @brief Calculate exploration progress
   * @return Progress percentage (0-100)
   */
  double calculateExplorationProgress() const;

  /**
   * @brief Get orchard statistics
   * @return Statistics including tree count, row count, etc.
   */
  struct OrchardStatistics
  {
    int total_trees;
    int total_rows;
    double total_area;
    double explored_area;
    double exploration_percentage;
    double average_tree_spacing;
    double average_row_spacing;
    int verified_trees;
    int unverified_trees;
  };
  
  OrchardStatistics getOrchardStatistics() const;

  /**
   * @brief Update tree verification status
   * @param tree_position Tree position to verify
   * @param verified Verification status
   */
  void updateTreeVerification(
    const geometry_msgs::msg::Point & tree_position,
    bool verified);

  /**
   * @brief Get visualization markers for orchard structure
   * @return Marker array for RViz visualization
   */
  visualization_msgs::msg::MarkerArray getOrchardVisualizationMarkers() const;

  /**
   * @brief Set exploration pattern
   * @param pattern New exploration pattern
   */
  void setExplorationPattern(ExplorationPattern pattern);

  /**
   * @brief Get current exploration pattern
   * @return Current pattern
   */
  ExplorationPattern getExplorationPattern() const;

  /**
   * @brief Set orchard configuration
   * @param config New configuration
   */
  void setConfig(const OrchardConfig & config);

  /**
   * @brief Get current configuration
   * @return Current configuration
   */
  const OrchardConfig & getConfig() const;

private:
  /**
   * @brief Cluster points into potential trees
   */
  std::vector<std::vector<geometry_msgs::msg::Point>> clusterTreePoints(
    const std::vector<geometry_msgs::msg::Point> & point_cloud);

  /**
   * @brief Fit line to points using RANSAC
   */
  bool fitLineRANSAC(
    const std::vector<geometry_msgs::msg::Point> & points,
    geometry_msgs::msg::Point & line_start,
    geometry_msgs::msg::Point & line_end,
    double & line_length);

  /**
   * @brief Calculate distance from point to line
   */
  double pointToLineDistance(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Point & line_start,
    const geometry_msgs::msg::Point & line_end);

  /**
   * @brief Sort rows by distance from start point
   */
  void sortRowsByDistance(
    std::vector<OrchardRow> & rows,
    const geometry_msgs::msg::Point & start_point);

  /**
   * @brief Generate waypoints along a row
   */
  std::vector<geometry_msgs::msg::PoseStamped> generateRowWaypoints(
    const OrchardRow & row,
    double spacing);

  /**
   * @brief Calculate optimal turning radius for row transitions
   */
  double calculateOptimalTurningRadius(
    const geometry_msgs::msg::Point & current_pos,
    const geometry_msgs::msg::Point & next_row_start);

  /**
   * @brief Validate tree detection
   */
  bool validateTreeDetection(
    const TreeDetection & tree,
    const std::vector<geometry_msgs::msg::Point> & nearby_points);

  /**
   * @brief Calculate tree spacing statistics
   */
  void calculateTreeSpacingStatistics();

  // Configuration
  OrchardConfig config_;
  
  // Orchard data
  std::vector<OrchardRow> detected_rows_;
  std::vector<TreeDetection> detected_trees_;
  std::unordered_map<std::string, bool> tree_verification_status_;
  
  // Statistics
  OrchardStatistics statistics_;
  
  // Internal state
  bool orchard_initialized_;
  geometry_msgs::msg::Point orchard_center_;
  double orchard_bounds_[4]; // min_x, max_x, min_y, max_y
  
  // Tree detection parameters
  double tree_cluster_radius_;
  int min_tree_points_;
  double tree_height_threshold_;
  
  // Row detection parameters
  double row_detection_threshold_;
  int min_row_points_;
  double max_row_deviation_;
};

} // namespace orbit_planner

#endif // ORBIT_PLANNER_ORCHARD_SPECIALIZED_PLANNER_HPP
