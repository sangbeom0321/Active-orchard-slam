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
#include <rclcpp/rclcpp.hpp>

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

void OrbitPanelPlugin::onAddPolygonPoint() {
    logMessage("onAddPolygonPoint() called"); // Debug message
    
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
    // Save parameters when changed
    saveParameters();
    logMessage("Parameters updated");
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
    
    // Create tab widget
    tab_widget_ = new QTabWidget();
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
    
    QVBoxLayout* layout = new QVBoxLayout(control_tab_);
    
    // Start point selection (removed - will use current odometry position)
    
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
    
    QVBoxLayout* layout = new QVBoxLayout(status_tab_);
    
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
    
    QVBoxLayout* layout = new QVBoxLayout(parameters_tab_);
    
    QGroupBox* params_group = new QGroupBox("Exploration Parameters");
    QGridLayout* params_layout = new QGridLayout(params_group);
    
    int row = 0;
    
    // Robot parameters
    params_layout->addWidget(new QLabel("Robot Radius (m):"), row, 0);
    robot_radius_spin_ = new QDoubleSpinBox();
    robot_radius_spin_->setRange(0.1, 2.0);
    robot_radius_spin_->setSingleStep(0.1);
    robot_radius_spin_->setValue(0.5);
    params_layout->addWidget(robot_radius_spin_, row++, 1);
    
    params_layout->addWidget(new QLabel("Safety Margin (m):"), row, 0);
    safety_margin_spin_ = new QDoubleSpinBox();
    safety_margin_spin_->setRange(0.0, 1.0);
    safety_margin_spin_->setSingleStep(0.1);
    safety_margin_spin_->setValue(0.2);
    params_layout->addWidget(safety_margin_spin_, row++, 1);
    
    params_layout->addWidget(new QLabel("Max Planning Distance (m):"), row, 0);
    max_planning_distance_spin_ = new QDoubleSpinBox();
    max_planning_distance_spin_->setRange(1.0, 50.0);
    max_planning_distance_spin_->setSingleStep(1.0);
    max_planning_distance_spin_->setValue(10.0);
    params_layout->addWidget(max_planning_distance_spin_, row++, 1);
    
    // Frontier parameters
    params_layout->addWidget(new QLabel("Frontier Cluster Min Size:"), row, 0);
    frontier_cluster_min_size_spin_ = new QDoubleSpinBox();
    frontier_cluster_min_size_spin_->setRange(1, 100);
    frontier_cluster_min_size_spin_->setSingleStep(1);
    frontier_cluster_min_size_spin_->setValue(5);
    params_layout->addWidget(frontier_cluster_min_size_spin_, row++, 1);
    
    params_layout->addWidget(new QLabel("Yaw Change Weight:"), row, 0);
    yaw_change_weight_spin_ = new QDoubleSpinBox();
    yaw_change_weight_spin_->setRange(0.0, 10.0);
    yaw_change_weight_spin_->setSingleStep(0.1);
    yaw_change_weight_spin_->setValue(1.0);
    params_layout->addWidget(yaw_change_weight_spin_, row++, 1);
    
    params_layout->addWidget(new QLabel("Frontier Gain Weight:"), row, 0);
    frontier_gain_weight_spin_ = new QDoubleSpinBox();
    frontier_gain_weight_spin_->setRange(0.0, 10.0);
    frontier_gain_weight_spin_->setSingleStep(0.1);
    frontier_gain_weight_spin_->setValue(1.0);
    params_layout->addWidget(frontier_gain_weight_spin_, row++, 1);
    
    // Orbit parameters
    params_layout->addWidget(new QLabel("Orbit Radius (m):"), row, 0);
    orbit_radius_spin_ = new QDoubleSpinBox();
    orbit_radius_spin_->setRange(0.5, 10.0);
    orbit_radius_spin_->setSingleStep(0.1);
    orbit_radius_spin_->setValue(2.0);
    params_layout->addWidget(orbit_radius_spin_, row++, 1);
    
    params_layout->addWidget(new QLabel("Orbit Spacing (m):"), row, 0);
    orbit_spacing_spin_ = new QDoubleSpinBox();
    orbit_spacing_spin_->setRange(0.1, 5.0);
    orbit_spacing_spin_->setSingleStep(0.1);
    orbit_spacing_spin_->setValue(1.0);
    params_layout->addWidget(orbit_spacing_spin_, row++, 1);
    
    layout->addWidget(params_group);
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
    
    // Subscribe to odometry for current position
    odometry_sub_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>(
        "/lio_sam/mapping/odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){
            current_pose_.header = msg->header;
            current_pose_.pose = msg->pose.pose;
            current_pose_received_ = true;
        });
    
    // Subscribe to RViz default clicked point to collect polygon vertices
    clicked_point_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg){
            // When user is in Add Point mode, push points
            if (add_point_mode_) {
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
}

void OrbitPanelPlugin::connectSignals() {
    // Connect button signals
    connect(add_point_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onAddPolygonPoint);
    connect(clear_polygon_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onClearPolygon);
    connect(publish_grid_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onPublishGrid);
    connect(save_area_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onSaveArea);
    connect(load_area_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onLoadArea);
    connect(start_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStartExploration);
    connect(stop_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStopExploration);
    connect(reset_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onResetExploration);
    
    // Connect parameter change signals
    connect(robot_radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(safety_margin_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(max_planning_distance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(frontier_cluster_min_size_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(yaw_change_weight_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(frontier_gain_weight_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(orbit_radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
    connect(orbit_spacing_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &OrbitPanelPlugin::onParameterChanged);
}

void OrbitPanelPlugin::loadParameters() {
    // Load parameters from ROS2 parameter server
    ros_node_->declare_parameter("robot_radius", 0.5);
    ros_node_->declare_parameter("safety_margin", 0.2);
    ros_node_->declare_parameter("max_planning_distance", 10.0);
    ros_node_->declare_parameter("frontier_cluster_min_size", 5.0);
    ros_node_->declare_parameter("yaw_change_weight", 1.0);
    ros_node_->declare_parameter("frontier_gain_weight", 1.0);
    ros_node_->declare_parameter("orbit_radius", 2.0);
    ros_node_->declare_parameter("orbit_spacing", 1.0);
    
    // Update UI with loaded parameters
    robot_radius_spin_->setValue(ros_node_->get_parameter("robot_radius").as_double());
    safety_margin_spin_->setValue(ros_node_->get_parameter("safety_margin").as_double());
    max_planning_distance_spin_->setValue(ros_node_->get_parameter("max_planning_distance").as_double());
    frontier_cluster_min_size_spin_->setValue(ros_node_->get_parameter("frontier_cluster_min_size").as_double());
    yaw_change_weight_spin_->setValue(ros_node_->get_parameter("yaw_change_weight").as_double());
    frontier_gain_weight_spin_->setValue(ros_node_->get_parameter("frontier_gain_weight").as_double());
    orbit_radius_spin_->setValue(ros_node_->get_parameter("orbit_radius").as_double());
    orbit_spacing_spin_->setValue(ros_node_->get_parameter("orbit_spacing").as_double());
}

void OrbitPanelPlugin::saveParameters() {
    // Save parameters to ROS2 parameter server
    ros_node_->set_parameter(rclcpp::Parameter("robot_radius", robot_radius_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("safety_margin", safety_margin_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("max_planning_distance", max_planning_distance_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("frontier_cluster_min_size", frontier_cluster_min_size_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("yaw_change_weight", yaw_change_weight_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("frontier_gain_weight", frontier_gain_weight_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("orbit_radius", orbit_radius_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("orbit_spacing", orbit_spacing_spin_->value()));
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

} // namespace orbit_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(orbit_planner::OrbitPanelPlugin, rviz_common::Panel)