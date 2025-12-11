#include "aos/aos_panel_plugin.hpp"
#include <QFileDialog>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QStandardPaths>
#include <QDateTime>
#include <QFile>
#include <geometry_msgs/msg/point.hpp>

namespace aos_planner {

void AosPanelPlugin::onSaveDefaults() {
    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save Default Parameters",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/aos_planner_defaults.json",
        "JSON Files (*.json)"
    );
    
    if (fileName.isEmpty()) {
        return;
    }
    
    saveDefaultParameters(fileName.toStdString());
}

void AosPanelPlugin::onLoadDefaults() {
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

void AosPanelPlugin::saveDefaultParameters(const std::string& filename) {
    QJsonObject json;
    json["name"] = "Orbit Planner Default Parameters";
    json["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    json["version"] = "1.0";
    
    // Save all parameter values
    json["clipping_minz"] = clipping_minz_spin_->value();
    json["clipping_maxz"] = clipping_maxz_spin_->value();
    json["clipping_minx"] = clipping_minx_spin_->value();
    json["clipping_maxx"] = clipping_maxx_spin_->value();
    json["clipping_miny"] = clipping_miny_spin_->value();
    json["clipping_maxy"] = clipping_maxy_spin_->value();
    json["grid_resolution"] = grid_resolution_spin_->value();
    json["inflation_radius"] = inflation_radius_spin_->value();
    json["waypoint_offset_distance"] = waypoint_offset_distance_spin_->value();
    json["interpolation_distance"] = interpolation_distance_spin_->value();
    json["smoothing_window_size"] = smoothing_window_size_spin_->value();
    json["planning_rate"] = planning_rate_spin_->value();
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

void AosPanelPlugin::loadDefaultParameters(const std::string& filename) {
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
    if (json.contains("clipping_minz")) clipping_minz_spin_->setValue(json["clipping_minz"].toDouble());
    if (json.contains("clipping_maxz")) clipping_maxz_spin_->setValue(json["clipping_maxz"].toDouble());
    if (json.contains("clipping_minx")) clipping_minx_spin_->setValue(json["clipping_minx"].toDouble());
    if (json.contains("clipping_maxx")) clipping_maxx_spin_->setValue(json["clipping_maxx"].toDouble());
    if (json.contains("clipping_miny")) clipping_miny_spin_->setValue(json["clipping_miny"].toDouble());
    if (json.contains("clipping_maxy")) clipping_maxy_spin_->setValue(json["clipping_maxy"].toDouble());
    if (json.contains("grid_resolution")) grid_resolution_spin_->setValue(json["grid_resolution"].toDouble());
    if (json.contains("inflation_radius")) inflation_radius_spin_->setValue(json["inflation_radius"].toDouble());
    if (json.contains("waypoint_offset_distance")) waypoint_offset_distance_spin_->setValue(json["waypoint_offset_distance"].toDouble());
    if (json.contains("interpolation_distance")) interpolation_distance_spin_->setValue(json["interpolation_distance"].toDouble());
    if (json.contains("smoothing_window_size")) smoothing_window_size_spin_->setValue(json["smoothing_window_size"].toInt());
    if (json.contains("planning_rate")) planning_rate_spin_->setValue(json["planning_rate"].toDouble());
    if (json.contains("connect_radius")) connect_radius_spin_->setValue(json["connect_radius"].toDouble());
    
    // Save parameters to ROS2 parameter server
    saveParameters();
    
    logMessage("Default parameters loaded from: " + filename);
    QMessageBox::information(this, "Load Defaults", "Default parameters loaded successfully!");
}

} // namespace aos_planner


