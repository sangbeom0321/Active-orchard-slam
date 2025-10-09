#include "aos/orbit_panel_plugin.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QTimer>
#include <QTime>
#include <QFileDialog>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QStandardPaths>
#include <QDir>
#include <QDateTime>
#include <QScrollArea>
#include <QGroupBox>
#include <QGridLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QListWidget>
#include <QProgressBar>
#include <QTabWidget>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <std_msgs/msg/string.hpp>

namespace orbit_planner {

OrbitPanelPlugin::OrbitPanelPlugin(QWidget* parent) 
    : rviz_common::Panel(parent) {
    
    // Guard against duplicate layout creation (RViz may re-init the panel)
    if (this->layout() == nullptr) {
        auto main_layout = new QVBoxLayout(this);
    }
    
    // Create control buttons
    auto button_layout = new QHBoxLayout();
    
    start_exploration_btn_ = new QPushButton("Start Exploration");
    stop_exploration_btn_ = new QPushButton("Stop Exploration");
    reset_btn_ = new QPushButton("Reset");
    
    button_layout->addWidget(start_exploration_btn_);
    button_layout->addWidget(stop_exploration_btn_);
    button_layout->addWidget(reset_btn_);
    
    // Create status labels
    status_label_ = new QLabel("Status: Idle");
    exploration_area_label_ = new QLabel("Exploration Area: Not defined");
    current_goal_label_ = new QLabel("Current Goal: None");
    frontiers_count_label_ = new QLabel("Frontiers: 0");
    orbit_anchors_count_label_ = new QLabel("Orbit Anchors: 0");
    
    // Create log text area
    log_text_ = new QTextEdit();
    log_text_->setMaximumHeight(100);
    log_text_->setReadOnly(true);
    
    // Add widgets to layout (use existing layout())
    if (auto main_layout = qobject_cast<QVBoxLayout*>(this->layout())) {
        main_layout->addLayout(button_layout);
        main_layout->addWidget(status_label_);
        main_layout->addWidget(exploration_area_label_);
        main_layout->addWidget(current_goal_label_);
        main_layout->addWidget(frontiers_count_label_);
        main_layout->addWidget(orbit_anchors_count_label_);
        main_layout->addWidget(log_text_);
    }
    
    // Connect signals
    connect(start_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStartExploration);
    connect(stop_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStopExploration);
    connect(reset_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onResetExploration);
    
    // ROS2 node will be created in onInitialize()
    
    // Initialize variables
    exploration_active_ = false;
    add_point_mode_ = false;
    add_waypoint_mode_ = false;
    start_point_selected_ = false;
    current_pose_received_ = false;
    frontiers_count_ = 0;
    orbit_anchors_count_ = 0;
    
    // Create timer for ROS2 spinning
    status_timer_ = new QTimer(this);
    connect(status_timer_, &QTimer::timeout, this, &OrbitPanelPlugin::spin);
    status_timer_->start(10); // 100 Hz
}

// OrbitPanelPlugin::~OrbitPanelPlugin() {
//     if (status_timer_) {
//         status_timer_->stop();
//     }
// }

void OrbitPanelPlugin::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
}

void OrbitPanelPlugin::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
}

void OrbitPanelPlugin::onStartExploration() {
    if (!start_exploration_client_->service_is_ready()) {
        logMessage("Start exploration service not available");
        return;
    }
    
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    start_exploration_client_->async_send_request(request);
    
    exploration_active_ = true;
    logMessage("Exploration started from current position");
}

void OrbitPanelPlugin::onStopExploration() {
    if (!stop_exploration_client_->service_is_ready()) {
        logMessage("Stop exploration service not available");
        return;
    }
    
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    stop_exploration_client_->async_send_request(request);
    
    exploration_active_ = false;
    logMessage("Exploration stopped");
}

void OrbitPanelPlugin::onResetExploration() {
    exploration_active_ = false;
    logMessage("Exploration reset");
}

void OrbitPanelPlugin::trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    // Update trajectory display if needed
}

void OrbitPanelPlugin::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_goal_ = *msg;
    updateStatus();
}

void OrbitPanelPlugin::frontiersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    frontiers_count_ = msg->markers.size();
    updateStatus();
}

void OrbitPanelPlugin::orbitAnchorsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    orbit_anchors_count_ = msg->markers.size();
    updateStatus();
}

void OrbitPanelPlugin::updateStatus() {
    if (exploration_active_) {
        status_label_->setText("Status: Exploring");
    } else {
        status_label_->setText("Status: Idle");
    }
    
    frontiers_count_label_->setText(QString("Frontiers: %1").arg(frontiers_count_));
    orbit_anchors_count_label_->setText(QString("Orbit Anchors: %1").arg(orbit_anchors_count_));
    
    if (current_goal_.pose.position.x != 0.0 || current_goal_.pose.position.y != 0.0) {
        QString goal_text = QString("Current Goal: (%.2f, %.2f)")
            .arg(current_goal_.pose.position.x)
            .arg(current_goal_.pose.position.y);
        current_goal_label_->setText(goal_text);
    } else {
        current_goal_label_->setText("Current Goal: None");
    }
}

void OrbitPanelPlugin::logMessage(const std::string& message) {
    QString timestamp = QTime::currentTime().toString("hh:mm:ss");
    log_text_->append(QString("[%1] %2")
        .arg(timestamp)
        .arg(QString::fromStdString(message)));
}

void OrbitPanelPlugin::spin() {
    rclcpp::spin_some(ros_node_);
}

// RViz panel interface implementations
void OrbitPanelPlugin::onInitialize() {
    // Initialize the panel
    setupUI();
    setupROS2();
    loadParameters();
    connectSignals();
    
    // Start status update timer
    status_timer_ = new QTimer(this);
    connect(status_timer_, &QTimer::timeout, this, &OrbitPanelPlugin::onUpdateStatus);
    status_timer_->start(100); // Update every 100ms
    
    logMessage("Orbit Planner Panel initialized");
}


// UI event handlers
// onStartPointClicked function removed - now using current odometry position

void OrbitPanelPlugin::onAddWaypoint() {
    logMessage("onAddWaypoint() called");
    
    // If in polygon mode, exit it first
    if (add_point_mode_) {
        add_point_mode_ = false;
        add_point_btn_->setText("Add Points");
        add_point_btn_->setStyleSheet("");
        logMessage("Exited Polygon Mode");
    }
    
    // Toggle add waypoint mode
    add_waypoint_mode_ = !add_waypoint_mode_;
    
    if (add_waypoint_mode_) {
        add_waypoint_btn_->setText("Stop Adding Waypoints");
        add_waypoint_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
        logMessage("Add Waypoint Mode: ON - Click on the map to add waypoints");
    } else {
        add_waypoint_btn_->setText("Add Waypoint");
        add_waypoint_btn_->setStyleSheet("");
        logMessage("Add Waypoint Mode: OFF - " + std::to_string(waypoints_.size()) + " waypoints collected");
    }
}

void OrbitPanelPlugin::onClearWaypoints() {
    waypoints_.clear();
    waypoints_list_->clear();
    add_waypoint_mode_ = false;
    add_waypoint_btn_->setText("Add Waypoint");
    add_waypoint_btn_->setStyleSheet("");
    
    // Update waypoints count label
    waypoints_count_label_->setText("Waypoints: 0");
    
    logMessage("All waypoints cleared");
}

void OrbitPanelPlugin::onPublishWaypoints() {
    if (waypoints_.empty()) {
        logMessage("No waypoints to publish");
        return;
    }
    
    // Publish waypoints as PoseStamped array
    geometry_msgs::msg::PoseArray waypoints_msg;
    waypoints_msg.header.stamp = ros_node_->now();
    waypoints_msg.header.frame_id = "map";
    
    for (const auto& waypoint : waypoints_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = waypoints_msg.header;
        pose.pose.position = waypoint;
        pose.pose.orientation.w = 1.0;
        waypoints_msg.poses.push_back(pose.pose);
    }
    
    waypoints_pub_->publish(waypoints_msg);
    logMessage("Published " + std::to_string(waypoints_.size()) + " waypoints");
}

void OrbitPanelPlugin::onAddPolygonPoint() {
    logMessage("onAddPolygonPoint() called"); // Debug message
    
    // If in waypoint mode, exit it first
    if (add_waypoint_mode_) {
        add_waypoint_mode_ = false;
        add_waypoint_btn_->setText("Add Waypoint");
        add_waypoint_btn_->setStyleSheet("");
        logMessage("Exited Waypoint Mode");
    }
    
    // If in start point mode, exit it first
    if (start_point_selected_) {
        start_point_selected_ = false;
        logMessage("Exited Start Point Mode");
    }
    
    // Toggle add point mode
    add_point_mode_ = !add_point_mode_;
    
    if (add_point_mode_) {
        add_point_btn_->setText("Stop Adding Points");
        add_point_btn_->setStyleSheet("QPushButton { background-color: #ff6b6b; color: white; }");
        logMessage("Add Point Mode: ON - Click on the map to add polygon vertices");
    } else {
        add_point_btn_->setText("Add Points");
        add_point_btn_->setStyleSheet("");
        logMessage("Add Point Mode: OFF - " + std::to_string(polygon_points_.size()) + " points collected");
    }
}

void OrbitPanelPlugin::onClearPolygon() {
    polygon_points_.clear();
    start_point_selected_ = false;
    add_point_mode_ = false;
    add_point_btn_->setText("Add Points");
    add_point_btn_->setStyleSheet("");
    
    // Update exploration area label
    exploration_area_label_->setText("Exploration Area: Not defined");
    
    // Publish empty polygon and clear marker
    publishPolygon();
    logMessage("Polygon cleared - all points removed");
}

void OrbitPanelPlugin::onParameterChanged() {
    // Save parameters locally first
    saveParameters();
    
    // Create parameter update message
    std_msgs::msg::String param_msg;
    std::ostringstream oss;
    
    // Format parameters as key=value pairs separated by semicolons
    oss << "map_update_rate=" << map_update_rate_spin_->value() << ";";
    oss << "planning_rate=" << planning_rate_spin_->value() << ";";
    oss << "robot_radius=" << robot_radius_spin_->value() << ";";
    oss << "safety_margin=" << safety_margin_spin_->value() << ";";
    oss << "max_planning_distance=" << max_planning_distance_spin_->value() << ";";
    oss << "frontier_cluster_min_size=" << frontier_cluster_min_size_spin_->value() << ";";
    oss << "frontier_cluster_max_distance=" << frontier_cluster_max_distance_spin_->value() << ";";
    oss << "goal_tolerance=" << goal_tolerance_spin_->value() << ";";
    oss << "yaw_change_weight=" << yaw_change_weight_spin_->value() << ";";
    oss << "frontier_gain_weight=" << frontier_gain_weight_spin_->value() << ";";
    oss << "distance_weight=" << distance_weight_spin_->value() << ";";
    oss << "tree_height_min=" << tree_height_min_spin_->value() << ";";
    oss << "tree_height_max=" << tree_height_max_spin_->value() << ";";
    oss << "tree_cluster_tolerance=" << tree_cluster_tolerance_spin_->value() << ";";
    oss << "tree_min_cluster_size=" << tree_min_cluster_size_spin_->value() << ";";
    oss << "row_tolerance=" << row_tolerance_spin_->value() << ";";
    oss << "row_min_length=" << row_min_length_spin_->value() << ";";
    oss << "row_min_trees=" << row_min_trees_spin_->value() << ";";
    oss << "center_search_radius=" << center_search_radius_spin_->value() << ";";
    oss << "clustering_radius=" << clustering_radius_spin_->value() << ";";
    oss << "min_neighbors_in_radius=" << min_neighbors_in_radius_spin_->value() << ";";
    oss << "orbit_radius=" << orbit_radius_spin_->value() << ";";
    oss << "orbit_spacing=" << orbit_spacing_spin_->value() << ";";
    oss << "path_resolution=" << path_resolution_spin_->value() << ";";
    oss << "path_smoothing_factor=" << path_smoothing_factor_spin_->value();
    
    param_msg.data = oss.str();
    
    // Publish parameter update
    parameter_pub_->publish(param_msg);
    logMessage("Parameters published to orbit_planner_node");
}

void OrbitPanelPlugin::onUpdateStatus() {
    updateStatus();
}

// UI setup methods
void OrbitPanelPlugin::setupUI() {
    // Create main layout only if not set
    if (this->layout() == nullptr) {
        main_layout_ = new QVBoxLayout(this);
        setLayout(main_layout_);
    } else {
        main_layout_ = qobject_cast<QVBoxLayout*>(this->layout());
    }
    
    // Set maximum size for the panel to prevent it from being too large
    this->setMaximumWidth(500); // Adjust width as needed
    this->setMaximumHeight(800); // Adjust height as needed
    
    // Create tab widget
    tab_widget_ = new QTabWidget();
    tab_widget_->setMaximumHeight(750); // Limit tab widget height
    main_layout_->addWidget(tab_widget_);
    
    // Setup tabs
    setupControlTab();
    setupStatusTab();
    setupVisualizationTab();
    setupParametersTab();
}

void OrbitPanelPlugin::setupControlTab() {
    control_tab_ = new QWidget();
    tab_widget_->addTab(control_tab_, "Control");
    
    // Create scroll area for control tab
    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    QWidget* scroll_widget = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(scroll_widget);
    
    // Set scroll area as the main widget for control tab
    QVBoxLayout* tab_layout = new QVBoxLayout(control_tab_);
    tab_layout->addWidget(scroll_area);
    scroll_area->setWidget(scroll_widget);
    
    // Start point selection (removed - will use current odometry position)
    
    // Waypoint management
    QGroupBox* waypoint_group = new QGroupBox("Waypoints");
    QVBoxLayout* waypoint_layout = new QVBoxLayout(waypoint_group);
    
    QHBoxLayout* waypoint_btn_layout = new QHBoxLayout();
    add_waypoint_btn_ = new QPushButton("Add Waypoint");
    clear_waypoints_btn_ = new QPushButton("Clear Waypoints");
    publish_waypoints_btn_ = new QPushButton("Publish Waypoints");
    waypoint_btn_layout->addWidget(add_waypoint_btn_);
    waypoint_btn_layout->addWidget(clear_waypoints_btn_);
    waypoint_btn_layout->addWidget(publish_waypoints_btn_);
    waypoint_layout->addLayout(waypoint_btn_layout);
    
    waypoints_list_ = new QListWidget();
    waypoints_list_->setMaximumHeight(100);
    waypoint_layout->addWidget(waypoints_list_);
    
    waypoints_count_label_ = new QLabel("Waypoints: 0");
    waypoint_layout->addWidget(waypoints_count_label_);
    
    layout->addWidget(waypoint_group);
    
    // Polygon management
    QGroupBox* polygon_group = new QGroupBox("Exploration Area");
    QVBoxLayout* polygon_layout = new QVBoxLayout(polygon_group);
    
    QHBoxLayout* polygon_btn_layout = new QHBoxLayout();
    add_point_btn_ = new QPushButton("Add Point");
    clear_polygon_btn_ = new QPushButton("Clear Polygon");
    publish_grid_btn_ = new QPushButton("Publish Empty Map");
    polygon_btn_layout->addWidget(add_point_btn_);
    polygon_btn_layout->addWidget(clear_polygon_btn_);
    polygon_btn_layout->addWidget(publish_grid_btn_);
    polygon_layout->addLayout(polygon_btn_layout);
    
    // Save/Load buttons
    QHBoxLayout* save_load_btn_layout = new QHBoxLayout();
    save_area_btn_ = new QPushButton("Save Area");
    load_area_btn_ = new QPushButton("Load Area");
    save_load_btn_layout->addWidget(save_area_btn_);
    save_load_btn_layout->addWidget(load_area_btn_);
    polygon_layout->addLayout(save_load_btn_layout);
    
    exploration_area_label_ = new QLabel("No exploration area defined");
    polygon_layout->addWidget(exploration_area_label_);
    layout->addWidget(polygon_group);
    
    // Exploration control
    QGroupBox* exploration_group = new QGroupBox("Exploration Control");
    QVBoxLayout* exploration_layout = new QVBoxLayout(exploration_group);
    
    QHBoxLayout* exploration_btn_layout = new QHBoxLayout();
    start_exploration_btn_ = new QPushButton("Start Exploration");
    stop_exploration_btn_ = new QPushButton("Stop Exploration");
    reset_btn_ = new QPushButton("Reset");
    exploration_btn_layout->addWidget(start_exploration_btn_);
    exploration_btn_layout->addWidget(stop_exploration_btn_);
    exploration_btn_layout->addWidget(reset_btn_);
    exploration_layout->addLayout(exploration_btn_layout);
    
    layout->addWidget(exploration_group);
    layout->addStretch();
}

void OrbitPanelPlugin::setupStatusTab() {
    status_tab_ = new QWidget();
    tab_widget_->addTab(status_tab_, "Status");
    
    // Create scroll area for status tab
    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    QWidget* scroll_widget = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(scroll_widget);
    
    // Set scroll area as the main widget for status tab
    QVBoxLayout* tab_layout = new QVBoxLayout(status_tab_);
    tab_layout->addWidget(scroll_area);
    scroll_area->setWidget(scroll_widget);
    
    // Status information
    QGroupBox* status_group = new QGroupBox("Status Information");
    QVBoxLayout* status_layout = new QVBoxLayout(status_group);
    
    status_label_ = new QLabel("Status: Ready");
    current_goal_label_ = new QLabel("Current Goal: None");
    frontiers_count_label_ = new QLabel("Frontiers: 0");
    orbit_anchors_count_label_ = new QLabel("Orbit Anchors: 0");
    
    status_layout->addWidget(status_label_);
    status_layout->addWidget(current_goal_label_);
    status_layout->addWidget(frontiers_count_label_);
    status_layout->addWidget(orbit_anchors_count_label_);
    
    // Progress bar
    exploration_progress_ = new QProgressBar();
    exploration_progress_->setRange(0, 100);
    exploration_progress_->setValue(0);
    status_layout->addWidget(exploration_progress_);
    
    layout->addWidget(status_group);
    
    // Log display
    QGroupBox* log_group = new QGroupBox("Log");
    QVBoxLayout* log_layout = new QVBoxLayout(log_group);
    
    log_text_ = new QTextEdit();
    log_text_->setReadOnly(true);
    log_text_->setMaximumHeight(200);
    log_layout->addWidget(log_text_);
    
    layout->addWidget(log_group);
}

void OrbitPanelPlugin::setupVisualizationTab() {
    visualization_tab_ = new QWidget();
    tab_widget_->addTab(visualization_tab_, "Visualization");
    
    QVBoxLayout* layout = new QVBoxLayout(visualization_tab_);
    
    QGroupBox* viz_group = new QGroupBox("Display Options");
    QVBoxLayout* viz_layout = new QVBoxLayout(viz_group);
    
    show_frontiers_cb_ = new QCheckBox("Show Frontiers");
    show_orbit_anchors_cb_ = new QCheckBox("Show Orbit Anchors");
    show_visited_cb_ = new QCheckBox("Show Visited Areas");
    show_trajectory_cb_ = new QCheckBox("Show Trajectory");
    show_occupancy_grid_cb_ = new QCheckBox("Show Occupancy Grid");
    
    show_frontiers_cb_->setChecked(true);
    show_orbit_anchors_cb_->setChecked(true);
    show_trajectory_cb_->setChecked(true);
    
    viz_layout->addWidget(show_frontiers_cb_);
    viz_layout->addWidget(show_orbit_anchors_cb_);
    viz_layout->addWidget(show_visited_cb_);
    viz_layout->addWidget(show_trajectory_cb_);
    viz_layout->addWidget(show_occupancy_grid_cb_);
    
    layout->addWidget(viz_group);
    layout->addStretch();
}

void OrbitPanelPlugin::setupParametersTab() {
    parameters_tab_ = new QWidget();
    tab_widget_->addTab(parameters_tab_, "Parameters");
    
    // Create scroll area for parameters
    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    // Set maximum height for scroll area to prevent panel from being too tall
    scroll_area->setMaximumHeight(600); // Adjust this value as needed
    scroll_area->setMinimumHeight(400);
    
    QWidget* scroll_widget = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(scroll_widget);
    
    // Set scroll area as the main widget for parameters tab
    QVBoxLayout* tab_layout = new QVBoxLayout(parameters_tab_);
    tab_layout->addWidget(scroll_area);
    scroll_area->setWidget(scroll_widget);
    
    // Update rates group
    QGroupBox* update_rates_group = new QGroupBox("Update Rates");
    QGridLayout* update_rates_layout = new QGridLayout(update_rates_group);
    
    int row = 0;
    update_rates_layout->addWidget(new QLabel("Map Update Rate (Hz):"), row, 0);
    map_update_rate_spin_ = new QDoubleSpinBox();
    map_update_rate_spin_->setRange(0.1, 10.0);
    map_update_rate_spin_->setSingleStep(0.1);
    map_update_rate_spin_->setValue(2.0);
    update_rates_layout->addWidget(map_update_rate_spin_, row++, 1);
    
    update_rates_layout->addWidget(new QLabel("Planning Rate (Hz):"), row, 0);
    planning_rate_spin_ = new QDoubleSpinBox();
    planning_rate_spin_->setRange(0.1, 10.0);
    planning_rate_spin_->setSingleStep(0.1);
    planning_rate_spin_->setValue(1.0);
    update_rates_layout->addWidget(planning_rate_spin_, row++, 1);
    
    layout->addWidget(update_rates_group);
    
    // Robot parameters group
    QGroupBox* robot_group = new QGroupBox("Robot Parameters");
    QGridLayout* robot_layout = new QGridLayout(robot_group);
    
    row = 0;
    robot_layout->addWidget(new QLabel("Robot Radius (m):"), row, 0);
    robot_radius_spin_ = new QDoubleSpinBox();
    robot_radius_spin_->setRange(0.1, 2.0);
    robot_radius_spin_->setSingleStep(0.1);
    robot_radius_spin_->setValue(0.4);
    robot_layout->addWidget(robot_radius_spin_, row++, 1);
    
    robot_layout->addWidget(new QLabel("Safety Margin (m):"), row, 0);
    safety_margin_spin_ = new QDoubleSpinBox();
    safety_margin_spin_->setRange(0.0, 1.0);
    safety_margin_spin_->setSingleStep(0.1);
    safety_margin_spin_->setValue(0.1);
    robot_layout->addWidget(safety_margin_spin_, row++, 1);
    
    layout->addWidget(robot_group);
    
    // Exploration parameters group
    QGroupBox* exploration_group = new QGroupBox("Exploration Parameters");
    QGridLayout* exploration_layout = new QGridLayout(exploration_group);
    
    row = 0;
    exploration_layout->addWidget(new QLabel("Max Planning Distance (m):"), row, 0);
    max_planning_distance_spin_ = new QDoubleSpinBox();
    max_planning_distance_spin_->setRange(1.0, 100.0);
    max_planning_distance_spin_->setSingleStep(1.0);
    max_planning_distance_spin_->setValue(50.0);
    exploration_layout->addWidget(max_planning_distance_spin_, row++, 1);
    
    exploration_layout->addWidget(new QLabel("Frontier Cluster Min Size:"), row, 0);
    frontier_cluster_min_size_spin_ = new QDoubleSpinBox();
    frontier_cluster_min_size_spin_->setRange(1, 100);
    frontier_cluster_min_size_spin_->setSingleStep(1);
    frontier_cluster_min_size_spin_->setValue(5.0);
    exploration_layout->addWidget(frontier_cluster_min_size_spin_, row++, 1);
    
    exploration_layout->addWidget(new QLabel("Frontier Cluster Max Distance (m):"), row, 0);
    frontier_cluster_max_distance_spin_ = new QDoubleSpinBox();
    frontier_cluster_max_distance_spin_->setRange(1.0, 50.0);
    frontier_cluster_max_distance_spin_->setSingleStep(1.0);
    frontier_cluster_max_distance_spin_->setValue(10.0);
    exploration_layout->addWidget(frontier_cluster_max_distance_spin_, row++, 1);
    
    exploration_layout->addWidget(new QLabel("Goal Tolerance (m):"), row, 0);
    goal_tolerance_spin_ = new QDoubleSpinBox();
    goal_tolerance_spin_->setRange(0.1, 5.0);
    goal_tolerance_spin_->setSingleStep(0.1);
    goal_tolerance_spin_->setValue(1.0);
    exploration_layout->addWidget(goal_tolerance_spin_, row++, 1);
    
    layout->addWidget(exploration_group);
    
    // Cost weights group
    QGroupBox* cost_weights_group = new QGroupBox("Cost Weights");
    QGridLayout* cost_weights_layout = new QGridLayout(cost_weights_group);
    
    row = 0;
    cost_weights_layout->addWidget(new QLabel("Yaw Change Weight:"), row, 0);
    yaw_change_weight_spin_ = new QDoubleSpinBox();
    yaw_change_weight_spin_->setRange(0.0, 10.0);
    yaw_change_weight_spin_->setSingleStep(0.1);
    yaw_change_weight_spin_->setValue(0.5);
    cost_weights_layout->addWidget(yaw_change_weight_spin_, row++, 1);
    
    cost_weights_layout->addWidget(new QLabel("Frontier Gain Weight:"), row, 0);
    frontier_gain_weight_spin_ = new QDoubleSpinBox();
    frontier_gain_weight_spin_->setRange(0.0, 10.0);
    frontier_gain_weight_spin_->setSingleStep(0.1);
    frontier_gain_weight_spin_->setValue(1.0);
    cost_weights_layout->addWidget(frontier_gain_weight_spin_, row++, 1);
    
    cost_weights_layout->addWidget(new QLabel("Distance Weight:"), row, 0);
    distance_weight_spin_ = new QDoubleSpinBox();
    distance_weight_spin_->setRange(0.0, 10.0);
    distance_weight_spin_->setSingleStep(0.1);
    distance_weight_spin_->setValue(1.0);
    cost_weights_layout->addWidget(distance_weight_spin_, row++, 1);
    
    layout->addWidget(cost_weights_group);
    
    // Tree detection parameters group
    QGroupBox* tree_detection_group = new QGroupBox("Tree Detection Parameters");
    QGridLayout* tree_detection_layout = new QGridLayout(tree_detection_group);
    
    row = 0;
    tree_detection_layout->addWidget(new QLabel("Tree Height Min (m):"), row, 0);
    tree_height_min_spin_ = new QDoubleSpinBox();
    tree_height_min_spin_->setRange(0.1, 2.0);
    tree_height_min_spin_->setSingleStep(0.1);
    tree_height_min_spin_->setValue(0.4);
    tree_detection_layout->addWidget(tree_height_min_spin_, row++, 1);
    
    tree_detection_layout->addWidget(new QLabel("Tree Height Max (m):"), row, 0);
    tree_height_max_spin_ = new QDoubleSpinBox();
    tree_height_max_spin_->setRange(0.1, 3.0);
    tree_height_max_spin_->setSingleStep(0.1);
    tree_height_max_spin_->setValue(0.7);
    tree_detection_layout->addWidget(tree_height_max_spin_, row++, 1);
    
    tree_detection_layout->addWidget(new QLabel("Tree Cluster Tolerance (m):"), row, 0);
    tree_cluster_tolerance_spin_ = new QDoubleSpinBox();
    tree_cluster_tolerance_spin_->setRange(0.1, 2.0);
    tree_cluster_tolerance_spin_->setSingleStep(0.1);
    tree_cluster_tolerance_spin_->setValue(0.5);
    tree_detection_layout->addWidget(tree_cluster_tolerance_spin_, row++, 1);
    
    tree_detection_layout->addWidget(new QLabel("Tree Min Cluster Size:"), row, 0);
    tree_min_cluster_size_spin_ = new QSpinBox();
    tree_min_cluster_size_spin_->setRange(1, 100);
    tree_min_cluster_size_spin_->setValue(10);
    tree_detection_layout->addWidget(tree_min_cluster_size_spin_, row++, 1);
    
    layout->addWidget(tree_detection_group);
    
    // Row detection parameters group
    QGroupBox* row_detection_group = new QGroupBox("Row Detection Parameters");
    QGridLayout* row_detection_layout = new QGridLayout(row_detection_group);
    
    row = 0;
    row_detection_layout->addWidget(new QLabel("Row Tolerance (m):"), row, 0);
    row_tolerance_spin_ = new QDoubleSpinBox();
    row_tolerance_spin_->setRange(0.5, 5.0);
    row_tolerance_spin_->setSingleStep(0.1);
    row_tolerance_spin_->setValue(1.5);
    row_detection_layout->addWidget(row_tolerance_spin_, row++, 1);
    
    row_detection_layout->addWidget(new QLabel("Row Min Length (m):"), row, 0);
    row_min_length_spin_ = new QDoubleSpinBox();
    row_min_length_spin_->setRange(1.0, 20.0);
    row_min_length_spin_->setSingleStep(0.5);
    row_min_length_spin_->setValue(5.0);
    row_detection_layout->addWidget(row_min_length_spin_, row++, 1);
    
    row_detection_layout->addWidget(new QLabel("Row Min Trees:"), row, 0);
    row_min_trees_spin_ = new QSpinBox();
    row_min_trees_spin_->setRange(1, 20);
    row_min_trees_spin_->setValue(3);
    row_detection_layout->addWidget(row_min_trees_spin_, row++, 1);
    
    layout->addWidget(row_detection_group);
    
    // Radius search clustering parameters group
    QGroupBox* clustering_group = new QGroupBox("Clustering Parameters");
    QGridLayout* clustering_layout = new QGridLayout(clustering_group);
    
    row = 0;
    clustering_layout->addWidget(new QLabel("Center Search Radius (m):"), row, 0);
    center_search_radius_spin_ = new QDoubleSpinBox();
    center_search_radius_spin_->setRange(0.01, 1.0);
    center_search_radius_spin_->setSingleStep(0.01);
    center_search_radius_spin_->setValue(0.15);
    clustering_layout->addWidget(center_search_radius_spin_, row++, 1);
    
    clustering_layout->addWidget(new QLabel("Clustering Radius (m):"), row, 0);
    clustering_radius_spin_ = new QDoubleSpinBox();
    clustering_radius_spin_->setRange(0.1, 5.0);
    clustering_radius_spin_->setSingleStep(0.1);
    clustering_radius_spin_->setValue(0.8);
    clustering_layout->addWidget(clustering_radius_spin_, row++, 1);
    
    clustering_layout->addWidget(new QLabel("Min Neighbors in Radius:"), row, 0);
    min_neighbors_in_radius_spin_ = new QSpinBox();
    min_neighbors_in_radius_spin_->setRange(0, 20);
    min_neighbors_in_radius_spin_->setValue(0);
    clustering_layout->addWidget(min_neighbors_in_radius_spin_, row++, 1);
    
    layout->addWidget(clustering_group);
    
    // Orbit anchor parameters group
    QGroupBox* orbit_group = new QGroupBox("Orbit Anchor Parameters");
    QGridLayout* orbit_layout = new QGridLayout(orbit_group);
    
    row = 0;
    orbit_layout->addWidget(new QLabel("Orbit Radius (m):"), row, 0);
    orbit_radius_spin_ = new QDoubleSpinBox();
    orbit_radius_spin_->setRange(0.5, 10.0);
    orbit_radius_spin_->setSingleStep(0.1);
    orbit_radius_spin_->setValue(2.0);
    orbit_layout->addWidget(orbit_radius_spin_, row++, 1);
    
    orbit_layout->addWidget(new QLabel("Orbit Spacing (m):"), row, 0);
    orbit_spacing_spin_ = new QDoubleSpinBox();
    orbit_spacing_spin_->setRange(0.1, 5.0);
    orbit_spacing_spin_->setSingleStep(0.1);
    orbit_spacing_spin_->setValue(1.0);
    orbit_layout->addWidget(orbit_spacing_spin_, row++, 1);
    
    layout->addWidget(orbit_group);
    
    // Path planning parameters group
    QGroupBox* path_planning_group = new QGroupBox("Path Planning Parameters");
    QGridLayout* path_planning_layout = new QGridLayout(path_planning_group);
    
    row = 0;
    path_planning_layout->addWidget(new QLabel("Path Resolution (m):"), row, 0);
    path_resolution_spin_ = new QDoubleSpinBox();
    path_resolution_spin_->setRange(0.01, 1.0);
    path_resolution_spin_->setSingleStep(0.01);
    path_resolution_spin_->setValue(0.1);
    path_planning_layout->addWidget(path_resolution_spin_, row++, 1);
    
    path_planning_layout->addWidget(new QLabel("Path Smoothing Factor:"), row, 0);
    path_smoothing_factor_spin_ = new QDoubleSpinBox();
    path_smoothing_factor_spin_->setRange(0.0, 2.0);
    path_smoothing_factor_spin_->setSingleStep(0.1);
    path_smoothing_factor_spin_->setValue(0.5);
    path_planning_layout->addWidget(path_smoothing_factor_spin_, row++, 1);
    
    layout->addWidget(path_planning_group);
    
    // GVD Topology parameters group
    QGroupBox* gvd_group = new QGroupBox("GVD Topology Parameters");
    QGridLayout* gvd_layout = new QGridLayout(gvd_group);
    
    row = 0;
    gvd_layout->addWidget(new QLabel("Connect Radius (cells):"), row, 0);
    connect_radius_spin_ = new QSpinBox();
    connect_radius_spin_->setRange(1, 50);
    connect_radius_spin_->setValue(10);
    gvd_layout->addWidget(connect_radius_spin_, row++, 1);
    
    layout->addWidget(gvd_group);
    
    // Default parameters group
    QGroupBox* defaults_group = new QGroupBox("Default Parameters");
    QHBoxLayout* defaults_layout = new QHBoxLayout(defaults_group);
    
    save_defaults_btn_ = new QPushButton("Save as Defaults");
    load_defaults_btn_ = new QPushButton("Load Defaults");
    
    save_defaults_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
    load_defaults_btn_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }");
    
    defaults_layout->addWidget(save_defaults_btn_);
    defaults_layout->addWidget(load_defaults_btn_);
    
    layout->addWidget(defaults_group);
    
    layout->addStretch();
}

void OrbitPanelPlugin::setupROS2() {
    // Create ROS2 node with unique name (only if not already created)
    if (!ros_node_) {
        static int node_counter = 0;
        std::string node_name = "orbit_panel_plugin_" + std::to_string(node_counter++);
        ros_node_ = rclcpp::Node::make_shared(node_name);
    }
    
    // Create publishers
    clicked_point_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PointStamped>(
        "/orbit_planner/clicked_point", 10);
    polygon_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "/orbit_planner/exploration_area", 10);
    start_pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/orbit_planner/start_pose", 10);
    polygon_marker_pub_ = ros_node_->create_publisher<visualization_msgs::msg::Marker>(
        "/orbit_planner/polygon_marker", 10);
    occupancy_grid_pub_ = ros_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/orbit_planner/occupancy", 10);
    waypoints_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseArray>(
        "/gvd/waypoints", 10);
    
    // Subscribe to odometry for current position
    odometry_sub_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>(
        "/lio_sam/mapping/odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){
            current_pose_.header = msg->header;
            current_pose_.pose = msg->pose.pose;
            current_pose_received_ = true;
        });
    
    // Subscribe to RViz default clicked point to collect polygon vertices and waypoints
    clicked_point_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg){
            // When user is in Add Waypoint mode, add waypoints
            if (add_waypoint_mode_) {
                geometry_msgs::msg::Point p;
                p.x = msg->point.x;
                p.y = msg->point.y;
                p.z = 0.0;  // 2D waypoint - force z=0
                waypoints_.push_back(p);
                
                // Update waypoints list widget
                QString waypoint_text = QString("WP%1: (%.2f, %.2f)")
                    .arg(waypoints_.size())
                    .arg(p.x)
                    .arg(p.y);
                waypoints_list_->addItem(waypoint_text);
                
                // Update waypoints count label
                QString count_text = QString("Waypoints: %1").arg(waypoints_.size());
                waypoints_count_label_->setText(count_text);
                
                logMessage("Added waypoint " + std::to_string(waypoints_.size()) + 
                          " at (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")");
            }
            // When user is in Add Point mode, push points
            else if (add_point_mode_) {
                geometry_msgs::msg::Point p;
                p.x = msg->point.x;
                p.y = msg->point.y;
                p.z = 0.0;  // 2D polygon - force z=0
                polygon_points_.push_back(p);
                publishPolygon();
                logMessage("Added polygon vertex " + std::to_string(polygon_points_.size()) + 
                          " at (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")");
                
                // Update exploration area label
                QString area_text = QString("Exploration Area: %1 points").arg(polygon_points_.size());
                exploration_area_label_->setText(area_text);
            }
        });
    
    // Create subscribers
    trajectory_sub_ = ros_node_->create_subscription<nav_msgs::msg::Path>(
        "/orbit_planner/trajectory", 10,
        std::bind(&OrbitPanelPlugin::trajectoryCallback, this, std::placeholders::_1));
    goal_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/orbit_planner/current_goal", 10,
        std::bind(&OrbitPanelPlugin::goalCallback, this, std::placeholders::_1));
    frontiers_sub_ = ros_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/frontiers", 10,
        std::bind(&OrbitPanelPlugin::frontiersCallback, this, std::placeholders::_1));
    orbit_anchors_sub_ = ros_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/orbit_anchors", 10,
        std::bind(&OrbitPanelPlugin::orbitAnchorsCallback, this, std::placeholders::_1));
    
    // Create service clients
    start_exploration_client_ = ros_node_->create_client<std_srvs::srv::Empty>("/orbit_planner/start_exploration");
    stop_exploration_client_ = ros_node_->create_client<std_srvs::srv::Empty>("/orbit_planner/stop_exploration");
    
    // Create parameter publisher for real-time parameter updates
    parameter_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("/orbit_planner/parameter_update", 10);
}

void OrbitPanelPlugin::connectSignals() {
    // Connect button signals
    connect(add_waypoint_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onAddWaypoint);
    connect(clear_waypoints_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onClearWaypoints);
    connect(publish_waypoints_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onPublishWaypoints);
    connect(add_point_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onAddPolygonPoint);
    connect(clear_polygon_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onClearPolygon);
    connect(publish_grid_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onPublishGrid);
    connect(save_area_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onSaveArea);
    connect(load_area_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onLoadArea);
    connect(start_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStartExploration);
    connect(stop_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStopExploration);
    connect(reset_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onResetExploration);
    
    // Connect parameter change signals
    connect(map_update_rate_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(planning_rate_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(robot_radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(safety_margin_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(max_planning_distance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(frontier_cluster_min_size_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(frontier_cluster_max_distance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(goal_tolerance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(yaw_change_weight_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(frontier_gain_weight_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(distance_weight_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(tree_height_min_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(tree_height_max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(tree_cluster_tolerance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(tree_min_cluster_size_spin_, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(row_tolerance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(row_min_length_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(row_min_trees_spin_, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(center_search_radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(clustering_radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(min_neighbors_in_radius_spin_, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(orbit_radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(orbit_spacing_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(path_resolution_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(path_smoothing_factor_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(connect_radius_spin_, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    
    // Connect default parameters buttons
    connect(save_defaults_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onSaveDefaults);
    connect(load_defaults_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onLoadDefaults);
}

void OrbitPanelPlugin::loadParameters() {
    // Load parameters from ROS2 parameter server with default values from YAML
    ros_node_->declare_parameter("map_update_rate", 2.0);
    ros_node_->declare_parameter("planning_rate", 1.0);
    ros_node_->declare_parameter("robot_radius", 0.4);
    ros_node_->declare_parameter("safety_margin", 0.1);
    ros_node_->declare_parameter("max_planning_distance", 50.0);
    ros_node_->declare_parameter("frontier_cluster_min_size", 5.0);
    ros_node_->declare_parameter("frontier_cluster_max_distance", 10.0);
    ros_node_->declare_parameter("goal_tolerance", 1.0);
    ros_node_->declare_parameter("yaw_change_weight", 0.5);
    ros_node_->declare_parameter("frontier_gain_weight", 1.0);
    ros_node_->declare_parameter("distance_weight", 1.0);
    ros_node_->declare_parameter("tree_height_min", 0.4);
    ros_node_->declare_parameter("tree_height_max", 0.7);
    ros_node_->declare_parameter("tree_cluster_tolerance", 0.5);
    ros_node_->declare_parameter("tree_min_cluster_size", 10);
    ros_node_->declare_parameter("row_tolerance", 1.5);
    ros_node_->declare_parameter("row_min_length", 5.0);
    ros_node_->declare_parameter("row_min_trees", 3);
    ros_node_->declare_parameter("center_search_radius", 0.15);
    ros_node_->declare_parameter("clustering_radius", 0.8);
    ros_node_->declare_parameter("min_neighbors_in_radius", 0);
    ros_node_->declare_parameter("orbit_radius", 2.0);
    ros_node_->declare_parameter("orbit_spacing", 1.0);
    ros_node_->declare_parameter("path_resolution", 0.1);
    ros_node_->declare_parameter("path_smoothing_factor", 0.5);
    ros_node_->declare_parameter("connect_radius", 10);
    
    // Update UI with loaded parameters
    map_update_rate_spin_->setValue(ros_node_->get_parameter("map_update_rate").as_double());
    planning_rate_spin_->setValue(ros_node_->get_parameter("planning_rate").as_double());
    robot_radius_spin_->setValue(ros_node_->get_parameter("robot_radius").as_double());
    safety_margin_spin_->setValue(ros_node_->get_parameter("safety_margin").as_double());
    max_planning_distance_spin_->setValue(ros_node_->get_parameter("max_planning_distance").as_double());
    frontier_cluster_min_size_spin_->setValue(ros_node_->get_parameter("frontier_cluster_min_size").as_double());
    frontier_cluster_max_distance_spin_->setValue(ros_node_->get_parameter("frontier_cluster_max_distance").as_double());
    goal_tolerance_spin_->setValue(ros_node_->get_parameter("goal_tolerance").as_double());
    yaw_change_weight_spin_->setValue(ros_node_->get_parameter("yaw_change_weight").as_double());
    frontier_gain_weight_spin_->setValue(ros_node_->get_parameter("frontier_gain_weight").as_double());
    distance_weight_spin_->setValue(ros_node_->get_parameter("distance_weight").as_double());
    tree_height_min_spin_->setValue(ros_node_->get_parameter("tree_height_min").as_double());
    tree_height_max_spin_->setValue(ros_node_->get_parameter("tree_height_max").as_double());
    tree_cluster_tolerance_spin_->setValue(ros_node_->get_parameter("tree_cluster_tolerance").as_double());
    tree_min_cluster_size_spin_->setValue(ros_node_->get_parameter("tree_min_cluster_size").as_int());
    row_tolerance_spin_->setValue(ros_node_->get_parameter("row_tolerance").as_double());
    row_min_length_spin_->setValue(ros_node_->get_parameter("row_min_length").as_double());
    row_min_trees_spin_->setValue(ros_node_->get_parameter("row_min_trees").as_int());
    center_search_radius_spin_->setValue(ros_node_->get_parameter("center_search_radius").as_double());
    clustering_radius_spin_->setValue(ros_node_->get_parameter("clustering_radius").as_double());
    min_neighbors_in_radius_spin_->setValue(ros_node_->get_parameter("min_neighbors_in_radius").as_int());
    orbit_radius_spin_->setValue(ros_node_->get_parameter("orbit_radius").as_double());
    orbit_spacing_spin_->setValue(ros_node_->get_parameter("orbit_spacing").as_double());
    path_resolution_spin_->setValue(ros_node_->get_parameter("path_resolution").as_double());
    path_smoothing_factor_spin_->setValue(ros_node_->get_parameter("path_smoothing_factor").as_double());
    connect_radius_spin_->setValue(ros_node_->get_parameter("connect_radius").as_int());
}

void OrbitPanelPlugin::saveParameters() {
    // Save parameters to ROS2 parameter server
    ros_node_->set_parameter(rclcpp::Parameter("map_update_rate", map_update_rate_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("planning_rate", planning_rate_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("robot_radius", robot_radius_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("safety_margin", safety_margin_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("max_planning_distance", max_planning_distance_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("frontier_cluster_min_size", frontier_cluster_min_size_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("frontier_cluster_max_distance", frontier_cluster_max_distance_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("goal_tolerance", goal_tolerance_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("yaw_change_weight", yaw_change_weight_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("frontier_gain_weight", frontier_gain_weight_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("distance_weight", distance_weight_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("tree_height_min", tree_height_min_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("tree_height_max", tree_height_max_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("tree_cluster_tolerance", tree_cluster_tolerance_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("tree_min_cluster_size", tree_min_cluster_size_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("row_tolerance", row_tolerance_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("row_min_length", row_min_length_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("row_min_trees", row_min_trees_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("center_search_radius", center_search_radius_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("clustering_radius", clustering_radius_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("min_neighbors_in_radius", min_neighbors_in_radius_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("orbit_radius", orbit_radius_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("orbit_spacing", orbit_spacing_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("path_resolution", path_resolution_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("path_smoothing_factor", path_smoothing_factor_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("connect_radius", connect_radius_spin_->value()));
}

void OrbitPanelPlugin::publishPolygon() {
    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = ros_node_->now();
    polygon_msg.header.frame_id = "map";
    
    for (const auto& point : polygon_points_) {
        geometry_msgs::msg::Point32 p32;
        p32.x = point.x;
        p32.y = point.y;
        p32.z = point.z;
        polygon_msg.polygon.points.push_back(p32);
    }
    
    polygon_pub_->publish(polygon_msg);
    
    // Publish polygon marker for visualization
    publishPolygonMarker();
}

void OrbitPanelPlugin::publishStartPose() {
    if (start_point_selected_) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = ros_node_->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position = start_point_.point;
        pose_msg.pose.orientation.w = 1.0;
        
        start_pose_pub_->publish(pose_msg);
    }
}

void OrbitPanelPlugin::publishPolygonMarker() {
    if (polygon_points_.size() < 3) {
        // Clear marker if not enough points
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = ros_node_->now();
        marker.header.frame_id = "map";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        polygon_marker_pub_->publish(marker);
        return;
    }
    
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = ros_node_->now();
    marker.header.frame_id = "map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set marker properties
    marker.scale.x = 0.1; // Line width
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // Fully opaque
    
    // Add polygon points
    for (const auto& point : polygon_points_) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }
    
    // Close the polygon by adding the first point at the end
    if (polygon_points_.size() >= 3) {
        geometry_msgs::msg::Point first_point;
        first_point.x = polygon_points_[0].x;
        first_point.y = polygon_points_[0].y;
        first_point.z = polygon_points_[0].z;
        marker.points.push_back(first_point);
    }
    
    polygon_marker_pub_->publish(marker);
}

void OrbitPanelPlugin::onPublishGrid() {
    // Create a simple occupancy grid for testing
    nav_msgs::msg::OccupancyGrid grid;
    
    // Set header
    grid.header.stamp = ros_node_->now();
    grid.header.frame_id = "map";
    
    // Set grid info
    grid.info.resolution = 0.1;  // 10cm per cell
    grid.info.width = 800;       // 80m width (8x larger)
    grid.info.height = 800;      // 80m height (8x larger)
    
    // Set origin (center the grid)
    grid.info.origin.position.x = -40.0;  // -40m (8x larger)
    grid.info.origin.position.y = -40.0;  // -40m (8x larger)
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    // Create grid data (800x800 = 640000 cells)
    grid.data.resize(800 * 800, 0);  // Initialize all cells as unknown (0)
    
    // Add some obstacles for demonstration
    // Create a border
    for (int i = 0; i < 800; i++) {
        // Top border
        grid.data[i] = 100;  // Occupied
        // Bottom border
        grid.data[799 * 800 + i] = 100;  // Occupied
        // Left border
        grid.data[i * 800] = 100;  // Occupied
        // Right border
        grid.data[i * 800 + 799] = 100;  // Occupied
    }
    
    // Add some internal obstacles
    for (int x = 160; x < 240; x++) {  // 8x larger obstacles
        for (int y = 160; y < 240; y++) {
            grid.data[y * 800 + x] = 100;  // Occupied
        }
    }
    
    for (int x = 560; x < 640; x++) {  // 8x larger obstacles
        for (int y = 560; y < 640; y++) {
            grid.data[y * 800 + x] = 100;  // Occupied
        }
    }
    
    // Publish the grid
    occupancy_grid_pub_->publish(grid);
    
    logMessage("Published empty map: 800x800 cells, 0.1m resolution (80m x 80m)");
}

void OrbitPanelPlugin::onSaveArea() {
    if (polygon_points_.size() < 3) {
        QMessageBox::warning(this, "Save Area", "Please define at least 3 points for the exploration area before saving.");
        return;
    }
    
    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save Exploration Area",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/exploration_area.json",
        "JSON Files (*.json)"
    );
    
    if (fileName.isEmpty()) {
        return;
    }
    
    QJsonObject json;
    json["name"] = "Exploration Area";
    json["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    json["frame_id"] = "map";
    
    QJsonArray pointsArray;
    for (const auto& point : polygon_points_) {
        QJsonObject pointObj;
        pointObj["x"] = point.x;
        pointObj["y"] = point.y;
        pointObj["z"] = point.z;
        pointsArray.append(pointObj);
    }
    json["points"] = pointsArray;
    
    QJsonDocument doc(json);
    
    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        logMessage("Exploration area saved to: " + fileName.toStdString());
        QMessageBox::information(this, "Save Area", "Exploration area saved successfully!");
    } else {
        logMessage("Failed to save exploration area to: " + fileName.toStdString());
        QMessageBox::critical(this, "Save Area", "Failed to save exploration area!");
    }
}

void OrbitPanelPlugin::onLoadArea() {
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "Load Exploration Area",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        "JSON Files (*.json)"
    );
    
    if (fileName.isEmpty()) {
        return;
    }
    
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        logMessage("Failed to open file: " + fileName.toStdString());
        QMessageBox::critical(this, "Load Area", "Failed to open file!");
        return;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    
    if (error.error != QJsonParseError::NoError) {
        logMessage("JSON parse error: " + error.errorString().toStdString());
        QMessageBox::critical(this, "Load Area", "Invalid JSON file!");
        return;
    }
    
    QJsonObject json = doc.object();
    
    if (!json.contains("points") || !json["points"].isArray()) {
        logMessage("Invalid JSON format: missing 'points' array");
        QMessageBox::critical(this, "Load Area", "Invalid file format!");
        return;
    }
    
    QJsonArray pointsArray = json["points"].toArray();
    if (pointsArray.size() < 3) {
        logMessage("Invalid area: need at least 3 points");
        QMessageBox::critical(this, "Load Area", "Area must have at least 3 points!");
        return;
    }
    
    // Clear current polygon
    polygon_points_.clear();
    
    // Load new points
    for (const auto& value : pointsArray) {
        QJsonObject pointObj = value.toObject();
        if (pointObj.contains("x") && pointObj.contains("y")) {
            geometry_msgs::msg::Point point;
            point.x = pointObj["x"].toDouble();
            point.y = pointObj["y"].toDouble();
            point.z = pointObj.contains("z") ? pointObj["z"].toDouble() : 0.0;
            polygon_points_.push_back(point);
        }
    }
    
    // Update UI
    QString area_text = QString("Exploration Area: %1 points").arg(polygon_points_.size());
    exploration_area_label_->setText(area_text);
    
    // Publish the loaded polygon
    publishPolygon();
    
    logMessage("Exploration area loaded from: " + fileName.toStdString() + 
              " (" + std::to_string(polygon_points_.size()) + " points)");
    QMessageBox::information(this, "Load Area", "Exploration area loaded successfully!");
}

void OrbitPanelPlugin::onSaveDefaults() {
    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save Default Parameters",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/orbit_planner_defaults.json",
        "JSON Files (*.json)"
    );
    
    if (fileName.isEmpty()) {
        return;
    }
    
    saveDefaultParameters(fileName.toStdString());
}

void OrbitPanelPlugin::onLoadDefaults() {
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "Load Default Parameters",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        "JSON Files (*.json)"
    );
    
    if (fileName.isEmpty()) {
        return;
    }
    
    loadDefaultParameters(fileName.toStdString());
}

void OrbitPanelPlugin::saveDefaultParameters(const std::string& filename) {
    QJsonObject json;
    json["name"] = "Orbit Planner Default Parameters";
    json["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    json["version"] = "1.0";
    
    // Save all parameter values
    json["map_update_rate"] = map_update_rate_spin_->value();
    json["planning_rate"] = planning_rate_spin_->value();
    json["robot_radius"] = robot_radius_spin_->value();
    json["safety_margin"] = safety_margin_spin_->value();
    json["max_planning_distance"] = max_planning_distance_spin_->value();
    json["frontier_cluster_min_size"] = frontier_cluster_min_size_spin_->value();
    json["frontier_cluster_max_distance"] = frontier_cluster_max_distance_spin_->value();
    json["goal_tolerance"] = goal_tolerance_spin_->value();
    json["yaw_change_weight"] = yaw_change_weight_spin_->value();
    json["frontier_gain_weight"] = frontier_gain_weight_spin_->value();
    json["distance_weight"] = distance_weight_spin_->value();
    json["tree_height_min"] = tree_height_min_spin_->value();
    json["tree_height_max"] = tree_height_max_spin_->value();
    json["tree_cluster_tolerance"] = tree_cluster_tolerance_spin_->value();
    json["tree_min_cluster_size"] = tree_min_cluster_size_spin_->value();
    json["row_tolerance"] = row_tolerance_spin_->value();
    json["row_min_length"] = row_min_length_spin_->value();
    json["row_min_trees"] = row_min_trees_spin_->value();
    json["center_search_radius"] = center_search_radius_spin_->value();
    json["clustering_radius"] = clustering_radius_spin_->value();
    json["min_neighbors_in_radius"] = min_neighbors_in_radius_spin_->value();
    json["orbit_radius"] = orbit_radius_spin_->value();
    json["orbit_spacing"] = orbit_spacing_spin_->value();
    json["path_resolution"] = path_resolution_spin_->value();
    json["path_smoothing_factor"] = path_smoothing_factor_spin_->value();
    json["connect_radius"] = connect_radius_spin_->value();
    
    QJsonDocument doc(json);
    
    QFile file(QString::fromStdString(filename));
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        logMessage("Default parameters saved to: " + filename);
        QMessageBox::information(this, "Save Defaults", "Default parameters saved successfully!");
    } else {
        logMessage("Failed to save default parameters to: " + filename);
        QMessageBox::critical(this, "Save Defaults", "Failed to save default parameters!");
    }
}

void OrbitPanelPlugin::loadDefaultParameters(const std::string& filename) {
    QFile file(QString::fromStdString(filename));
    if (!file.open(QIODevice::ReadOnly)) {
        logMessage("Failed to open file: " + filename);
        QMessageBox::critical(this, "Load Defaults", "Failed to open file!");
        return;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    
    if (error.error != QJsonParseError::NoError) {
        logMessage("JSON parse error: " + error.errorString().toStdString());
        QMessageBox::critical(this, "Load Defaults", "Invalid JSON file!");
        return;
    }
    
    QJsonObject json = doc.object();
    
    // Load parameter values
    if (json.contains("map_update_rate")) map_update_rate_spin_->setValue(json["map_update_rate"].toDouble());
    if (json.contains("planning_rate")) planning_rate_spin_->setValue(json["planning_rate"].toDouble());
    if (json.contains("robot_radius")) robot_radius_spin_->setValue(json["robot_radius"].toDouble());
    if (json.contains("safety_margin")) safety_margin_spin_->setValue(json["safety_margin"].toDouble());
    if (json.contains("max_planning_distance")) max_planning_distance_spin_->setValue(json["max_planning_distance"].toDouble());
    if (json.contains("frontier_cluster_min_size")) frontier_cluster_min_size_spin_->setValue(json["frontier_cluster_min_size"].toDouble());
    if (json.contains("frontier_cluster_max_distance")) frontier_cluster_max_distance_spin_->setValue(json["frontier_cluster_max_distance"].toDouble());
    if (json.contains("goal_tolerance")) goal_tolerance_spin_->setValue(json["goal_tolerance"].toDouble());
    if (json.contains("yaw_change_weight")) yaw_change_weight_spin_->setValue(json["yaw_change_weight"].toDouble());
    if (json.contains("frontier_gain_weight")) frontier_gain_weight_spin_->setValue(json["frontier_gain_weight"].toDouble());
    if (json.contains("distance_weight")) distance_weight_spin_->setValue(json["distance_weight"].toDouble());
    if (json.contains("tree_height_min")) tree_height_min_spin_->setValue(json["tree_height_min"].toDouble());
    if (json.contains("tree_height_max")) tree_height_max_spin_->setValue(json["tree_height_max"].toDouble());
    if (json.contains("tree_cluster_tolerance")) tree_cluster_tolerance_spin_->setValue(json["tree_cluster_tolerance"].toDouble());
    if (json.contains("tree_min_cluster_size")) tree_min_cluster_size_spin_->setValue(json["tree_min_cluster_size"].toInt());
    if (json.contains("row_tolerance")) row_tolerance_spin_->setValue(json["row_tolerance"].toDouble());
    if (json.contains("row_min_length")) row_min_length_spin_->setValue(json["row_min_length"].toDouble());
    if (json.contains("row_min_trees")) row_min_trees_spin_->setValue(json["row_min_trees"].toInt());
    if (json.contains("center_search_radius")) center_search_radius_spin_->setValue(json["center_search_radius"].toDouble());
    if (json.contains("clustering_radius")) clustering_radius_spin_->setValue(json["clustering_radius"].toDouble());
    if (json.contains("min_neighbors_in_radius")) min_neighbors_in_radius_spin_->setValue(json["min_neighbors_in_radius"].toInt());
    if (json.contains("orbit_radius")) orbit_radius_spin_->setValue(json["orbit_radius"].toDouble());
    if (json.contains("orbit_spacing")) orbit_spacing_spin_->setValue(json["orbit_spacing"].toDouble());
    if (json.contains("path_resolution")) path_resolution_spin_->setValue(json["path_resolution"].toDouble());
    if (json.contains("path_smoothing_factor")) path_smoothing_factor_spin_->setValue(json["path_smoothing_factor"].toDouble());
    if (json.contains("connect_radius")) connect_radius_spin_->setValue(json["connect_radius"].toInt());
    
    // Save parameters to ROS2 parameter server
    saveParameters();
    
    logMessage("Default parameters loaded from: " + filename);
    QMessageBox::information(this, "Load Defaults", "Default parameters loaded successfully!");
}

} // namespace orbit_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(orbit_planner::OrbitPanelPlugin, rviz_common::Panel)