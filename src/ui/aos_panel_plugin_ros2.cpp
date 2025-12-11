#include "aos/aos_panel_plugin.hpp"
#include <QFile>
#include <QMetaObject>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <fstream>
#include "lio_sam_wo/srv/save_map.hpp"
#include <std_srvs/srv/empty.hpp>

namespace aos_planner {

void AosPanelPlugin::setupROS2() {
    // Create ROS2 node with unique name (only if not already created)
    if (!ros_node_) {
        static int node_counter = 0;
        std::string node_name = "aos_panel_plugin_" + std::to_string(node_counter++);
        // Load parameters from YAML at startup
        rclcpp::NodeOptions options;
        // Compose YAML path (package-relative first, then absolute workspace fallback)
        QString yaml_path_qt = ":/../src/aos/config/aos_planner_params.yaml";
        if (!QFile::exists(yaml_path_qt)) {
            yaml_path_qt = QString::fromStdString(std::string("/root/ros2_ws/src/aos/config/aos_planner_params.yaml"));
        }
        if (QFile::exists(yaml_path_qt)) {
            options.arguments({"--ros-args", "--params-file", yaml_path_qt.toStdString()});
            logMessage("Loading parameters from YAML: " + yaml_path_qt.toStdString());
        } else {
            logMessage("Parameter YAML not found; using built-in defaults");
        }
        ros_node_ = rclcpp::Node::make_shared(node_name, options);
    }
    
    // Subscribe to RViz default clicked point to collect polygon vertices
    clicked_point_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg){
            logMessage("Clicked point received: (" + std::to_string(msg->point.x) + ", " + std::to_string(msg->point.y) + ")");
        });
    
    // Create subscribers with bag-friendly QoS
    rclcpp::QoS bag_qos(10);
    bag_qos.best_effort();  // Use best_effort for bag file compatibility
    bag_qos.durability_volatile();
    bag_qos.keep_last(1);  // Keep only last message for bag files
    
    trajectory_sub_ = ros_node_->create_subscription<nav_msgs::msg::Path>(
        "/aos_planner/trajectory", bag_qos,
        std::bind(&AosPanelPlugin::trajectoryCallback, this, std::placeholders::_1));
    goal_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aos_planner/current_goal", bag_qos,
        std::bind(&AosPanelPlugin::goalCallback, this, std::placeholders::_1));
    frontiers_sub_ = ros_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/aos_planner/frontiers", bag_qos,
        std::bind(&AosPanelPlugin::frontiersCallback, this, std::placeholders::_1));
    aos_anchors_sub_ = ros_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/aos_planner/aos_anchors", bag_qos,
        std::bind(&AosPanelPlugin::aosAnchorsCallback, this, std::placeholders::_1));
    
    // Subscribe to GPS fix with reliable QoS (must not drop data)
    rclcpp::QoS gps_qos(10);
    gps_qos.reliable();
    gps_qos.durability_volatile();
    gps_qos.keep_last(1);  // Keep only last message for bag files
    gps_sub_ = ros_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fix", gps_qos,
        std::bind(&AosPanelPlugin::gpsCallback, this, std::placeholders::_1));
    
    // Subscribe to control mode
    rclcpp::QoS control_qos(10);
    control_qos.best_effort();
    control_qos.durability_volatile();
    control_qos.keep_last(1);
    control_mod_sub_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
        "/Control/mod", control_qos,
        std::bind(&AosPanelPlugin::controlModeCallback, this, std::placeholders::_1));
    
    // Subscribe to odometry for current position
    rclcpp::QoS odom_qos(10);
    odom_qos.best_effort();
    odom_qos.durability_volatile();
    odom_qos.keep_last(1);
    odom_sub_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_baselink", odom_qos,
        std::bind(&AosPanelPlugin::odomCallback, this, std::placeholders::_1));
    
    // Subscribe to cluster info to get total cluster count
    rclcpp::QoS cluster_qos(10);
    cluster_qos.best_effort();
    cluster_qos.durability_volatile();
    cluster_qos.keep_last(1);
    cluster_info_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PoseArray>(
        "/cluster_info", cluster_qos,
        std::bind(&AosPanelPlugin::clusterInfoCallback, this, std::placeholders::_1));
    
    // Subscribe to planning status from gvd_cluster_planning_node
    rclcpp::QoS status_qos(10);
    status_qos.best_effort();
    status_qos.durability_volatile();
    status_qos.keep_last(1);
    cluster_index_sub_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
        "/aos/current_cluster_index", status_qos,
        std::bind(&AosPanelPlugin::clusterIndexCallback, this, std::placeholders::_1));
    waypoint_index_sub_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
        "/aos/current_waypoint_index", status_qos,
        std::bind(&AosPanelPlugin::waypointIndexCallback, this, std::placeholders::_1));
    path_status_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "/aos/path_planning_status", status_qos,
        std::bind(&AosPanelPlugin::pathStatusCallback, this, std::placeholders::_1));
    
    // Create service clients (use station's deactivate service)
    remote_control_client_ = ros_node_->create_client<std_srvs::srv::SetBool>("/station/deactivate_remote_contoller");
    save_map_client_ = ros_node_->create_client<lio_sam_wo::srv::SaveMap>("/lio_sam/save_map");
    save_cluster_info_client_ = ros_node_->create_client<std_srvs::srv::Empty>("/gvd/save_cluster_info");
    
    // Create parameter publisher for real-time parameter updates
    parameter_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("/aos_planner/parameter_update", 10);
}

void AosPanelPlugin::spin() {
    if (ros_node_ && rclcpp::ok()) {
        try {
            rclcpp::spin_some(ros_node_);
        } catch (const rclcpp::exceptions::RCLError& e) {
            // ROS2 context is shutting down, ignore
        }
    }
}

void AosPanelPlugin::monitorTopics() {
    if (!ros_node_) return;
    
    // Determine topic status based on last receive times
    using namespace std::chrono;
    const auto now = steady_clock::now();
    const auto stale_threshold = seconds(5); // consider NO DATA if older than 5s

    auto gps_ok = last_gps_msg_time_.time_since_epoch().count() != 0 && (now - last_gps_msg_time_) < stale_threshold;
    std::string status = "Topic Status - GPS: " + std::string(gps_ok ? "OK" : "NO DATA");

    if (topic_status_label_) {
        topic_status_label_->setText(QString::fromStdString(status));
    }
}

void AosPanelPlugin::trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    // Update trajectory display if needed
}

void AosPanelPlugin::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_goal_ = *msg;
    updateStatus();
}

void AosPanelPlugin::frontiersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    frontiers_count_ = msg->markers.size();
    updateStatus();
}

void AosPanelPlugin::aosAnchorsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    aos_anchors_count_ = msg->markers.size();
    updateStatus();
}

void AosPanelPlugin::controlModeCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg) {
        return;
    }
    control_mode_ = msg->data;
    last_control_msg_time_ = std::chrono::steady_clock::now();
    
    // Update control mode label
    if (planning_control_mode_label_) {
        QString mode_text = "Control Mode: ";
        switch (control_mode_) {
            case 0: mode_text += "NORMAL"; break;
            case 1: mode_text += "DOCKING"; break;
            case 2: mode_text += "ARRIVAL"; break;
            case 3: mode_text += "STOP"; break;
            default: mode_text += QString("Unknown (%1)").arg(control_mode_); break;
        }
        planning_control_mode_label_->setText(mode_text);
    }
    
    updateStatus();
    updatePlanningProgress();
}

void AosPanelPlugin::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!msg) {
        return;
    }
    
    // Update current position
    current_position_ = msg->pose.pose.position;
    
    // Update UI label
    if (planning_current_pos_label_) {
        QString pos_text = QString("Current Position: (%1, %2)")
            .arg(current_position_.x, 0, 'f', 2)
            .arg(current_position_.y, 0, 'f', 2);
        planning_current_pos_label_->setText(pos_text);
    }
    
    // Update progress bar
    updatePlanningProgress();
}

void AosPanelPlugin::clusterInfoCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (!msg) {
        return;
    }
    
    cluster_info_ = *msg;
    
    // Calculate total clusters and waypoints
    if (msg->poses.size() >= 4) {
        total_clusters_ = msg->poses.size() / 4;
        
        // Calculate total waypoints: last cluster has 3, others have 2
        total_waypoints_ = 0;
        for (int i = 0; i < total_clusters_; i++) {
            if (i == total_clusters_ - 1) {
                total_waypoints_ += 3;  // Last cluster has 3 waypoints
            } else {
                total_waypoints_ += 2;  // Other clusters have 2 waypoints
            }
        }
        
        // Update cluster and waypoint labels
        updateClusterWaypointLabels();
        
        // Update progress bar
        updatePlanningProgress();
    }
}

void AosPanelPlugin::clusterIndexCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg) {
        return;
    }
    current_cluster_index_ = msg->data;
    updateClusterWaypointLabels();
    updatePlanningProgress();
}

void AosPanelPlugin::waypointIndexCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg) {
        return;
    }
    current_waypoint_index_ = msg->data;
    updateClusterWaypointLabels();
    updatePlanningProgress();
}

void AosPanelPlugin::pathStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!msg || !planning_path_status_label_) {
        return;
    }
    
    QString status_text = QString("Path Planning Status: %1")
        .arg(QString::fromStdString(msg->data));
    planning_path_status_label_->setText(status_text);
    
    // Change color based on status
    if (msg->data == "Success") {
        planning_path_status_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
    } else if (msg->data == "Failed") {
        planning_path_status_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    } else if (msg->data == "Returning...") {
        planning_path_status_label_->setStyleSheet("QLabel { color: blue; font-weight: bold; }");
        logMessage("Exploration Complete");
    } else if (msg->data == "Exploration Complete") {
        planning_path_status_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
        logMessage("Exploration Complete");
        // Reset cluster and waypoint indices
        current_cluster_index_ = -1;
        current_waypoint_index_ = -1;
        updateClusterWaypointLabels();
    } else {
        planning_path_status_label_->setStyleSheet("QLabel { color: orange; font-weight: bold; }");
    }
}

void AosPanelPlugin::updateClusterWaypointLabels() {
    if (planning_cluster_label_) {
        QString cluster_text;
        if (current_cluster_index_ >= 0 && total_clusters_ > 0) {
            cluster_text = QString("Cluster %1 of %2")
                .arg(current_cluster_index_ + 1)  // 1-based display
                .arg(total_clusters_);
        } else {
            cluster_text = "Cluster - of -";
        }
        planning_cluster_label_->setText(cluster_text);
    }
    
    if (planning_waypoint_label_) {
        QString waypoint_text;
        if (current_waypoint_index_ >= 0 && total_waypoints_ > 0) {
            waypoint_text = QString("WP%1 of %2")
                .arg(current_waypoint_index_)  // 0-based display
                .arg(total_waypoints_);
        } else {
            waypoint_text = "WP- of -";
        }
        planning_waypoint_label_->setText(waypoint_text);
    }
}

void AosPanelPlugin::updateStatus() {
    // If control mode topic not received recently, show IDLE
    using namespace std::chrono;
    const auto now = steady_clock::now();
    const auto stale_threshold = seconds(5);
    bool control_fresh = last_control_msg_time_.time_since_epoch().count() != 0 &&
                         (now - last_control_msg_time_) < stale_threshold;

    QString mode_text = "IDLE";
    if (control_fresh) {
        switch (control_mode_) {
            case 0: mode_text = "NORMAL"; break;
            case 1: mode_text = "DOCKING"; break;
            case 2: mode_text = "ARRIVAL"; break;
            case 3: mode_text = "STOP"; break;
            default: mode_text = "UNKNOWN"; break;
        }
    }
    status_label_->setText("Status: " + mode_text);
    // Only update goal label if it exists (it may be removed from UI)
    if (current_goal_label_) {
        if (current_goal_.pose.position.x != 0.0 || current_goal_.pose.position.y != 0.0) {
            QString goal_text = QString("Current Goal: (%.2f, %.2f)")
                .arg(current_goal_.pose.position.x)
                .arg(current_goal_.pose.position.y);
            current_goal_label_->setText(goal_text);
        } else {
            current_goal_label_->setText("Current Goal: None");
        }
    }
}

void AosPanelPlugin::updatePlanningProgress() {
    if (!planning_progress_bar_ || !planning_progress_label_) {
        return;
    }
    
    // If no cluster info or waypoints, show 0%
    if (total_waypoints_ == 0 || total_clusters_ == 0) {
        planning_progress_bar_->setValue(0);
        planning_progress_label_->setText("Progress: 0% (Waiting for cluster info)");
        return;
    }
    
    // Calculate progress based on current cluster and waypoint
    int current_cluster = current_cluster_index_;
    int current_waypoint = current_waypoint_index_;
    
    // Calculate completed waypoints
    int completed_waypoints = 0;
    
    if (current_cluster >= 0 && current_cluster < total_clusters_) {
        // Count waypoints in completed clusters
        for (int i = 0; i < current_cluster; i++) {
            if (i == total_clusters_ - 1) {
                completed_waypoints += 3;  // Last cluster has 3 waypoints
            } else {
                completed_waypoints += 2;  // Other clusters have 2 waypoints
            }
        }
        
        // Add current waypoint index (0-based, so add 1 for completed)
        if (current_waypoint >= 0) {
            completed_waypoints += current_waypoint + 1;
        }
    }
    
    // Calculate percentage
    int total_with_first = total_waypoints_ + 1;  // +1 for initial waypoint (8.0, 0.0)
    int completed_with_first = completed_waypoints + 1;  // +1 for first waypoint
    
    int progress_percent = (completed_with_first * 100) / total_with_first;
    progress_percent = std::min(100, std::max(0, progress_percent));  // Clamp between 0-100
    
    planning_progress_bar_->setValue(progress_percent);
    
    QString progress_text = QString("Progress: %1% (%2/%3 waypoints completed)")
        .arg(progress_percent)
        .arg(completed_with_first)
        .arg(total_with_first);
    planning_progress_label_->setText(progress_text);
}

void AosPanelPlugin::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    current_gps_ = *msg;
    current_gps_received_ = true;
    last_gps_msg_time_ = std::chrono::steady_clock::now();
}

void AosPanelPlugin::onSaveMap() {
    if (!save_map_client_) {
        logMessage("Save map service client not initialized");
        return;
    }
    
    if (!save_map_client_->wait_for_service(std::chrono::seconds(1))) {
        logMessage("Save map service not available");
        return;
    }
    
    auto request = std::make_shared<lio_sam_wo::srv::SaveMap::Request>();
    auto future = save_map_client_->async_send_request(request);
    
    // Wait for result (with timeout)
    if (rclcpp::spin_until_future_complete(ros_node_, future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            logMessage("Map Saved");
            
            // Save cluster information after map is saved
            if (save_cluster_info_client_ && save_cluster_info_client_->wait_for_service(std::chrono::seconds(1))) {
                auto cluster_request = std::make_shared<std_srvs::srv::Empty::Request>();
                auto cluster_future = save_cluster_info_client_->async_send_request(cluster_request);
                
                if (rclcpp::spin_until_future_complete(ros_node_, cluster_future, std::chrono::seconds(5)) ==
                    rclcpp::FutureReturnCode::SUCCESS) {
                    logMessage("Cluster information saved");
                } else {
                    logMessage("Failed to save cluster information (timeout or error)");
                }
            } else {
                logMessage("Save cluster info service not available");
            }
        } else {
            logMessage("Failed to save map");
        }
    } else {
        logMessage("Failed to save map (timeout or error)");
    }
}

} // namespace aos_planner

