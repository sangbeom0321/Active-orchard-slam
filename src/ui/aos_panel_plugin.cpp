#include "aos/aos_panel_plugin.hpp"
#include <QTimer>
#include <QTime>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>

namespace aos_planner {

AosPanelPlugin::AosPanelPlugin(QWidget* parent) 
    : rviz_common::Panel(parent) {
    
    // Initialize variables first
    exploration_active_ = false;
    start_point_selected_ = false;
    current_gps_received_ = false;
    frontiers_count_ = 0;
    aos_anchors_count_ = 0;
    last_gps_msg_time_ = std::chrono::steady_clock::time_point{};
    last_control_msg_time_ = std::chrono::steady_clock::time_point{};
    control_mode_ = -1;
    
    // Initialize pointers to nullptr
    main_layout_ = nullptr;
    tab_widget_ = nullptr;
    control_tab_ = nullptr;
    status_tab_ = nullptr;
    parameters_tab_ = nullptr;
    status_timer_ = nullptr;
    ros_node_ = nullptr;
    
    // Initialize remote control UI components to nullptr
    remote_control_group_ = nullptr;
    remote_control_off_btn_ = nullptr;
    remote_control_on_btn_ = nullptr;
    remote_control_status_label_ = nullptr;
    save_map_btn_ = nullptr;
    
    // Initialize Status tab UI pointers to nullptr
    status_label_ = nullptr;
    topic_status_label_ = nullptr;
    current_goal_label_ = nullptr;
    frontiers_count_label_ = nullptr;
    aos_anchors_count_label_ = nullptr;
    exploration_progress_ = nullptr;
    log_text_ = nullptr;
    
    // Initialize Planning Status UI pointers to nullptr
    planning_status_group_ = nullptr;
    planning_current_pos_label_ = nullptr;
    planning_control_mode_label_ = nullptr;
    planning_cluster_label_ = nullptr;
    planning_waypoint_label_ = nullptr;
    planning_path_status_label_ = nullptr;
    planning_progress_bar_ = nullptr;
    planning_progress_label_ = nullptr;
    current_cluster_index_ = -1;
    current_waypoint_index_ = -1;
    total_clusters_ = 0;
    total_waypoints_ = 0;
}

AosPanelPlugin::~AosPanelPlugin() {
    // Stop timers safely
    if (status_timer_) {
        status_timer_->stop();
        status_timer_->deleteLater();
    }
    if (spin_timer_) {
        spin_timer_->stop();
        spin_timer_->deleteLater();
    }
    if (topic_monitor_timer_) {
        topic_monitor_timer_->stop();
        topic_monitor_timer_->deleteLater();
    }
    
    // Clear ROS2 node reference
    ros_node_.reset();
}

void AosPanelPlugin::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
}

void AosPanelPlugin::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
}

void AosPanelPlugin::onInitialize() {
    try {
        // Initialize the panel in correct order
        setupUI();
        setupROS2();
        loadParameters();
        connectSignals();
        
        // Start status update timer
        if (!status_timer_) {
            status_timer_ = new QTimer(this);
            connect(status_timer_, &QTimer::timeout, this, &AosPanelPlugin::onUpdateStatus);
            status_timer_->start(100); // Update every 100ms
        }
        
        // Start ROS2 spin timer with higher frequency for bag files
        if (!spin_timer_) {
            spin_timer_ = new QTimer(this);
            connect(spin_timer_, &QTimer::timeout, this, &AosPanelPlugin::spin);
            spin_timer_->start(1); // Spin every 1ms for better bag file compatibility
        }
        
        // Start topic monitoring timer
        if (!topic_monitor_timer_) {
            topic_monitor_timer_ = new QTimer(this);
            connect(topic_monitor_timer_, &QTimer::timeout, this, &AosPanelPlugin::monitorTopics);
            topic_monitor_timer_->start(2000); // Check every 2 seconds
        }
        
        logMessage("Orbit Planner Panel initialized");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("aos_panel"), "Failed to initialize panel: %s", e.what());
    }
}

void AosPanelPlugin::logMessage(const std::string& message) {
    if (!log_text_) {
        return; // UI not ready yet
    }
    
    QString timestamp = QTime::currentTime().toString("hh:mm:ss");
    log_text_->append(QString("[%1] %2")
        .arg(timestamp)
        .arg(QString::fromStdString(message)));
}

void AosPanelPlugin::onUpdateStatus() {
    updateStatus();
}

void AosPanelPlugin::onParameterChanged() {
    // Save parameters locally first
    saveParameters();
    
    // Create parameter update message
    std_msgs::msg::String param_msg;
    std::ostringstream oss;
    
    oss << "connect_radius=" << connect_radius_spin_->value();
    
    param_msg.data = oss.str();
    
    // Publish parameter update
    parameter_pub_->publish(param_msg);
    logMessage("Parameters published to aos_planner_node");
}

} // namespace aos_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aos_planner::AosPanelPlugin, rviz_common::Panel)
