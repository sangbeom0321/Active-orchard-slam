#include "aos/aos_panel_plugin.hpp"
#include <QFile>
#include <QMessageBox>
#include <QRegularExpression>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

namespace aos_planner {

void AosPanelPlugin::loadParameters() {
    // Load parameters from ROS2 parameter server with default values from YAML
    ros_node_->declare_parameter("clipping_minz", -0.4);
    ros_node_->declare_parameter("clipping_maxz", 0.1);
    ros_node_->declare_parameter("clipping_minx", -5.0);
    ros_node_->declare_parameter("clipping_maxx", 72.0);
    ros_node_->declare_parameter("clipping_miny", -3.0);
    ros_node_->declare_parameter("clipping_maxy", 10.0);
    ros_node_->declare_parameter("grid_resolution", 0.05);
    ros_node_->declare_parameter("inflation_radius", 0.5);
    ros_node_->declare_parameter("waypoint_offset_distance", 2.0);
    ros_node_->declare_parameter("interpolation_distance", 0.05);
    ros_node_->declare_parameter("smoothing_window_size", 3);
    ros_node_->declare_parameter("planning_rate", 1.0);
    ros_node_->declare_parameter("connect_radius", 1.5);
    
    // Update UI with loaded parameters
    clipping_minz_spin_->setValue(ros_node_->get_parameter("clipping_minz").as_double());
    clipping_maxz_spin_->setValue(ros_node_->get_parameter("clipping_maxz").as_double());
    clipping_minx_spin_->setValue(ros_node_->get_parameter("clipping_minx").as_double());
    clipping_maxx_spin_->setValue(ros_node_->get_parameter("clipping_maxx").as_double());
    clipping_miny_spin_->setValue(ros_node_->get_parameter("clipping_miny").as_double());
    clipping_maxy_spin_->setValue(ros_node_->get_parameter("clipping_maxy").as_double());
    grid_resolution_spin_->setValue(ros_node_->get_parameter("grid_resolution").as_double());
    inflation_radius_spin_->setValue(ros_node_->get_parameter("inflation_radius").as_double());
    waypoint_offset_distance_spin_->setValue(ros_node_->get_parameter("waypoint_offset_distance").as_double());
    interpolation_distance_spin_->setValue(ros_node_->get_parameter("interpolation_distance").as_double());
    smoothing_window_size_spin_->setValue(ros_node_->get_parameter("smoothing_window_size").as_int());
    planning_rate_spin_->setValue(ros_node_->get_parameter("planning_rate").as_double());
    connect_radius_spin_->setValue(ros_node_->get_parameter("connect_radius").as_double());
}

void AosPanelPlugin::saveParameters() {
    // Save parameters to ROS2 parameter server
    ros_node_->set_parameter(rclcpp::Parameter("clipping_minz", clipping_minz_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("clipping_maxz", clipping_maxz_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("clipping_minx", clipping_minx_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("clipping_maxx", clipping_maxx_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("clipping_miny", clipping_miny_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("clipping_maxy", clipping_maxy_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("grid_resolution", grid_resolution_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("inflation_radius", inflation_radius_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("waypoint_offset_distance", waypoint_offset_distance_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("interpolation_distance", interpolation_distance_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("smoothing_window_size", smoothing_window_size_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("planning_rate", planning_rate_spin_->value()));
    ros_node_->set_parameter(rclcpp::Parameter("connect_radius", connect_radius_spin_->value()));
}

void AosPanelPlugin::onModifyParameters() {
    // Compose YAML path (package-relative)
    QString yaml_path = ":/../src/aos/config/aos_planner_params.yaml";
    // Fallback to workspace path if needed
    if (!QFile::exists(yaml_path)) {
        yaml_path = QString::fromStdString(std::string("/root/ros2_ws/src/aos/config/aos_planner_params.yaml"));
    }

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly)) {
        logMessage("Failed to open YAML for reading: " + yaml_path.toStdString());
        QMessageBox::critical(this, "Modify Parameters", "Failed to open YAML for reading.");
        return;
    }

    QByteArray raw = file.readAll();
    file.close();

    QString text = QString::fromUtf8(raw);

    // Simple YAML patch by string replacement (keeps indentation and other keys)
    auto replaceScalar = [&](const QString& key, const QString& value){
        QRegularExpression re("(^\\s*" + QRegularExpression::escape(key) + ":)\\s*.*$", QRegularExpression::MultilineOption);
        text.replace(re, "\\1 " + value);
    };

    // Apply current UI values into YAML text
    replaceScalar("clipping_minz", QString::number(clipping_minz_spin_->value(), 'f', 2));
    replaceScalar("clipping_maxz", QString::number(clipping_maxz_spin_->value(), 'f', 2));
    replaceScalar("clipping_minx", QString::number(clipping_minx_spin_->value(), 'f', 2));
    replaceScalar("clipping_maxx", QString::number(clipping_maxx_spin_->value(), 'f', 2));
    replaceScalar("clipping_miny", QString::number(clipping_miny_spin_->value(), 'f', 2));
    replaceScalar("clipping_maxy", QString::number(clipping_maxy_spin_->value(), 'f', 2));
    replaceScalar("grid_resolution", QString::number(grid_resolution_spin_->value(), 'f', 2));
    replaceScalar("inflation_radius", QString::number(inflation_radius_spin_->value(), 'f', 2));
    replaceScalar("waypoint_offset_distance", QString::number(waypoint_offset_distance_spin_->value(), 'f', 2));
    replaceScalar("interpolation_distance", QString::number(interpolation_distance_spin_->value(), 'f', 2));
    replaceScalar("smoothing_window_size", QString::number(smoothing_window_size_spin_->value()));
    replaceScalar("planning_rate", QString::number(planning_rate_spin_->value(), 'f', 2));
    replaceScalar("connect_radius", QString::number(connect_radius_spin_->value(), 'f', 2));

    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        logMessage("Failed to open YAML for writing: " + yaml_path.toStdString());
        QMessageBox::critical(this, "Modify Parameters", "Failed to open YAML for writing.");
        return;
    }

    file.write(text.toUtf8());
    file.close();

    logMessage("Parameters written to YAML: " + yaml_path.toStdString());
    QMessageBox::information(this, "Modify Parameters", "Parameters saved to YAML. Restart nodes to apply.");

    // Try to push updated parameters live to target nodes (best-effort)
    try {
        // Push connect_radius to GVD topology node if available
        auto client = std::make_shared<rclcpp::SyncParametersClient>(ros_node_, "gvd_topology_node");
        if (client->wait_for_service(std::chrono::milliseconds(500))) {
            client->set_parameters({ rclcpp::Parameter("connect_radius", connect_radius_spin_->value()) });
            logMessage("connect_radius pushed to gvd_topology_node");
        } else {
            logMessage("gvd_topology_node parameter service not available (skipping live update)");
        }
    } catch (const std::exception& e) {
        logMessage(std::string("Live parameter update failed: ") + e.what());
    }
}

} // namespace aos_planner

