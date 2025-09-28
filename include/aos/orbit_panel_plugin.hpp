/**
 * @file orbit_panel_plugin.hpp
 * @brief RViz2 Panel plugin for orbit planner control
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This class implements an RViz2 panel plugin that provides a GUI for
 * defining exploration areas and controlling the orbit planner.
 */

#pragma once

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <std_msgs/msg/string.hpp>

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QGroupBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QProgressBar>
#include <QListWidget>
#include <QTableWidget>
#include <QTabWidget>
#include <QComboBox>
#include <QSlider>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>

#include <memory>
#include <vector>

namespace orbit_planner {

class OrbitPanelPlugin : public rviz_common::Panel {
    Q_OBJECT

public:
    OrbitPanelPlugin(QWidget* parent = nullptr);
    ~OrbitPanelPlugin() = default;

    // RViz panel interface
    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

private slots:
    // UI event handlers
    void onAddWaypoint();
    void onClearWaypoints();
    void onPublishWaypoints();
    void onAddPolygonPoint();
    void onClearPolygon();
    void onPublishGrid();
    void onSaveArea();
    void onLoadArea();
    void onStartExploration();
    void onStopExploration();
    void onResetExploration();
    void onParameterChanged();
    void onUpdateStatus();

private:
    // ROS2 communication
    rclcpp::Node::SharedPtr ros_node_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr polygon_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr frontiers_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr orbit_anchors_sub_;
    
    // Services
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_exploration_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_exploration_client_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parameter_pub_;
    
    // UI Components
    QVBoxLayout* main_layout_;
    QTabWidget* tab_widget_;
    
    // Control Tab
    QWidget* control_tab_;
    QPushButton* add_waypoint_btn_;
    QPushButton* clear_waypoints_btn_;
    QPushButton* publish_waypoints_btn_;
    QListWidget* waypoints_list_;
    QLabel* waypoints_count_label_;
    QPushButton* add_point_btn_;
    QPushButton* clear_polygon_btn_;
    QPushButton* publish_grid_btn_;
    QPushButton* save_area_btn_;
    QPushButton* load_area_btn_;
    QPushButton* start_exploration_btn_;
    QPushButton* stop_exploration_btn_;
    QPushButton* reset_btn_;
    
    // Status Tab
    QWidget* status_tab_;
    QLabel* status_label_;
    QLabel* exploration_area_label_;
    QLabel* current_goal_label_;
    QLabel* frontiers_count_label_;
    QLabel* orbit_anchors_count_label_;
    QProgressBar* exploration_progress_;
    QTextEdit* log_text_;
    
    // Visualization Tab
    QWidget* visualization_tab_;
    QCheckBox* show_frontiers_cb_;
    QCheckBox* show_orbit_anchors_cb_;
    QCheckBox* show_visited_cb_;
    QCheckBox* show_trajectory_cb_;
    QCheckBox* show_occupancy_grid_cb_;
    
    // Parameters Tab
    QWidget* parameters_tab_;
    
    // Update rates
    QDoubleSpinBox* map_update_rate_spin_;
    QDoubleSpinBox* planning_rate_spin_;
    
    // Robot parameters
    QDoubleSpinBox* robot_radius_spin_;
    QDoubleSpinBox* safety_margin_spin_;
    
    // Exploration parameters
    QDoubleSpinBox* max_planning_distance_spin_;
    QDoubleSpinBox* frontier_cluster_min_size_spin_;
    QDoubleSpinBox* frontier_cluster_max_distance_spin_;
    QDoubleSpinBox* goal_tolerance_spin_;
    
    // Cost weights
    QDoubleSpinBox* yaw_change_weight_spin_;
    QDoubleSpinBox* frontier_gain_weight_spin_;
    QDoubleSpinBox* distance_weight_spin_;
    
    // Tree detection parameters
    QDoubleSpinBox* tree_height_min_spin_;
    QDoubleSpinBox* tree_height_max_spin_;
    QDoubleSpinBox* tree_cluster_tolerance_spin_;
    QSpinBox* tree_min_cluster_size_spin_;
    
    // Row detection parameters
    QDoubleSpinBox* row_tolerance_spin_;
    QDoubleSpinBox* row_min_length_spin_;
    QSpinBox* row_min_trees_spin_;
    
    // Clustering parameters
    QDoubleSpinBox* center_search_radius_spin_;
    QDoubleSpinBox* clustering_radius_spin_;
    QSpinBox* min_neighbors_in_radius_spin_;
    
    // Orbit anchor parameters
    QDoubleSpinBox* orbit_radius_spin_;
    QDoubleSpinBox* orbit_spacing_spin_;
    
    // Path planning parameters
    QDoubleSpinBox* path_resolution_spin_;
    QDoubleSpinBox* path_smoothing_factor_spin_;
    
    // GVD Topology parameters
    QSpinBox* connect_radius_spin_;
    
    // Data
    std::vector<geometry_msgs::msg::Point> waypoints_;
    std::vector<geometry_msgs::msg::Point> polygon_points_;
    geometry_msgs::msg::PointStamped start_point_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool start_point_selected_;
    bool add_waypoint_mode_;
    bool add_point_mode_;
    bool current_pose_received_;
    bool exploration_active_;
    
    // Status
    std::string current_status_;
    geometry_msgs::msg::PoseStamped current_goal_;
    int frontiers_count_;
    int orbit_anchors_count_;
    double exploration_progress_value_;
    
    // Timer for status updates
    QTimer* status_timer_;
    
    // Callbacks
    void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void frontiersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void orbitAnchorsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    // UI update functions
    void updateStatus();
    void updatePolygonDisplay();
    void updateParameterDisplay();
    void logMessage(const std::string& message);
    
    // Utility functions
    void setupUI();
    void setupControlTab();
    void setupStatusTab();
    void setupVisualizationTab();
    void setupParametersTab();
    void connectSignals();
    void loadParameters();
    void saveParameters();
    
    // ROS2 setup
    void setupROS2();
    void publishPolygon();
    void publishStartPose();
    void publishPolygonMarker();
    void spin();
};

} // namespace orbit_planner