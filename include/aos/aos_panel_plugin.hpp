/**
 * @file aos_panel_plugin.hpp
 * @brief RViz2 Panel plugin for aos planner control
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This class implements an RViz2 panel plugin that provides a GUI for
 * defining exploration areas and controlling the aos planner.
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
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <lio_sam_wo/srv/save_map.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

#include <memory>
#include <vector>

namespace aos_planner {

class AosPanelPlugin : public rviz_common::Panel {
    Q_OBJECT

public:
    AosPanelPlugin(QWidget* parent = nullptr);
    ~AosPanelPlugin();

    // RViz panel interface
    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

private slots:
    // UI event handlers
    void onParameterChanged();
    void onUpdateStatus();
    void onSaveDefaults();
    void onLoadDefaults();
    void onModifyParameters();
    void onRemoteControlOff();
    void onRemoteControlOn();
    void onSaveMap();

private:
    // ROS2 communication
    rclcpp::Node::SharedPtr ros_node_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    // Odometry subscription removed
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr frontiers_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr aos_anchors_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_mod_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cluster_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cluster_index_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_index_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_status_sub_;
    
    // Services
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr remote_control_client_;
    rclcpp::Client<lio_sam_wo::srv::SaveMap>::SharedPtr save_map_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_cluster_info_client_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parameter_pub_;
    
    // UI Components
    QVBoxLayout* main_layout_;
    QTabWidget* tab_widget_;
    
    // Control Tab
    QWidget* control_tab_;
    
    // Remote Control Group
    QGroupBox* remote_control_group_;
    QPushButton* remote_control_off_btn_;
    QPushButton* remote_control_on_btn_;
    QLabel* remote_control_status_label_;
    
    // Save Map Button
    QPushButton* save_map_btn_;
    
    // Status Tab
    QWidget* status_tab_;
    QLabel* status_label_;
    QLabel* topic_status_label_;
    QLabel* current_goal_label_;
    QLabel* frontiers_count_label_;
    QLabel* aos_anchors_count_label_;
    QProgressBar* exploration_progress_;
    QTextEdit* log_text_;
    
    // (Visualization Tab removed)
    
    // Parameters Tab
    QWidget* parameters_tab_;
    
    
    // Occupancy Grid Generator parameters
    QDoubleSpinBox* clipping_minz_spin_;
    QDoubleSpinBox* clipping_maxz_spin_;
    QDoubleSpinBox* clipping_minx_spin_;
    QDoubleSpinBox* clipping_maxx_spin_;
    QDoubleSpinBox* clipping_miny_spin_;
    QDoubleSpinBox* clipping_maxy_spin_;
    QDoubleSpinBox* grid_resolution_spin_;
    QDoubleSpinBox* inflation_radius_spin_;
    QDoubleSpinBox* waypoint_offset_distance_spin_;
    
    // Path Smoothing parameters
    QDoubleSpinBox* interpolation_distance_spin_;
    QSpinBox* smoothing_window_size_spin_;
    
    // GVD Planning parameters
    QDoubleSpinBox* planning_rate_spin_;
    
    // GVD Topology parameters
    QDoubleSpinBox* connect_radius_spin_;
    
    // Default parameters buttons
    QPushButton* save_defaults_btn_;
    QPushButton* load_defaults_btn_;
    QPushButton* modify_params_btn_;
    
    // Planning Status UI Components
    QGroupBox* planning_status_group_;
    QLabel* planning_current_pos_label_;
    QLabel* planning_control_mode_label_;
    QLabel* planning_cluster_label_;
    QLabel* planning_waypoint_label_;
    QLabel* planning_path_status_label_;
    QProgressBar* planning_progress_bar_;
    QLabel* planning_progress_label_;
    
    // Data
    geometry_msgs::msg::PointStamped start_point_;
    geometry_msgs::msg::PoseStamped current_pose_;
    sensor_msgs::msg::NavSatFix current_gps_;
    bool start_point_selected_;
    bool current_pose_received_;
    bool current_gps_received_;
    bool exploration_active_;
    
    // Status
    std::string current_status_;
    geometry_msgs::msg::PoseStamped current_goal_;
    int frontiers_count_;
    int aos_anchors_count_;
    double exploration_progress_value_;
    
    // Topic last receive timestamps
    std::chrono::steady_clock::time_point last_gps_msg_time_;
    // Odometry tracking removed
    std::chrono::steady_clock::time_point last_control_msg_time_;
    int control_mode_;
    
    // Planning Status data
    geometry_msgs::msg::Point current_position_;
    int current_cluster_index_;
    int current_waypoint_index_;
    geometry_msgs::msg::PoseArray cluster_info_;
    int total_clusters_;
    int total_waypoints_;
    
    // Timer for status updates
    QTimer* status_timer_;
    QTimer* spin_timer_;
    QTimer* topic_monitor_timer_;
    
    // Callbacks
    void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void frontiersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void aosAnchorsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void controlModeCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void clusterInfoCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void clusterIndexCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void waypointIndexCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void pathStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    
    // UI update functions
    void updateStatus();
    void updatePolygonDisplay();
    void updateParameterDisplay();
    void updatePlanningProgress();
    void updateClusterWaypointLabels();
    void logMessage(const std::string& message);
    
    // Utility functions
    void setupUI();
    void setupControlTab();
    void setupStatusTab();
    // void setupVisualizationTab(); // removed
    void setupParametersTab();
    void connectSignals();
    void loadParameters();
    void saveParameters();
    void saveDefaultParameters(const std::string& filename);
    void loadDefaultParameters(const std::string& filename);
    
    // ROS2 setup
    void setupROS2();
    void spin();
    void monitorTopics();
};

} // namespace aos_planner