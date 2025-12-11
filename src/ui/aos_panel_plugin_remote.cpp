#include "aos/aos_panel_plugin.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>

namespace aos_planner {

void AosPanelPlugin::onRemoteControlOff() {
    if (!remote_control_client_) {
        logMessage("Remote control client not available");
        return;
    }
    
    if (!remote_control_client_->service_is_ready()) {
        logMessage("Remote control service not ready");
        remote_control_status_label_->setText("Status: Service Unavailable");
        remote_control_status_label_->setStyleSheet("QLabel { color: red; }");
        return;
    }
    
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;  // true = deactivate remote control
    
    // Simple synchronous call with timeout
    try {
        auto future = remote_control_client_->async_send_request(request);
        auto status = future.wait_for(std::chrono::seconds(1));
        
        if (status == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                remote_control_status_label_->setText("Status: OFF");
                remote_control_status_label_->setStyleSheet("QLabel { color: red; }");
                
                logMessage("Remote control deactivated successfully");
            } else {
                remote_control_status_label_->setText("Status: Error");
                remote_control_status_label_->setStyleSheet("QLabel { color: red; }");
                
                logMessage("Failed to deactivate remote control");
            }
        } else {
            remote_control_status_label_->setText("Status: OFF (Timeout)");
            remote_control_status_label_->setStyleSheet("QLabel { color: orange; }");
            
            logMessage("Remote control service call timeout");
        }
    } catch (const std::exception& e) {
        remote_control_status_label_->setText("Status: Error");
        remote_control_status_label_->setStyleSheet("QLabel { color: red; }");
        
        logMessage("Remote control service call failed: " + std::string(e.what()));
    }
}

void AosPanelPlugin::onRemoteControlOn() {
    if (!remote_control_client_) {
        logMessage("Remote control client not available");
        return;
    }
    
    if (!remote_control_client_->service_is_ready()) {
        logMessage("Remote control service not ready");
        remote_control_status_label_->setText("Status: Service Unavailable");
        remote_control_status_label_->setStyleSheet("QLabel { color: red; }");
        return;
    }
    
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;  // false = activate remote control
    
    // Simple synchronous call with timeout
    try {
        auto future = remote_control_client_->async_send_request(request);
        auto status = future.wait_for(std::chrono::seconds(1));
        
        if (status == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                remote_control_status_label_->setText("Status: ON");
                remote_control_status_label_->setStyleSheet("QLabel { color: green; }");
                
                logMessage("Remote control activated successfully");
            } else {
                remote_control_status_label_->setText("Status: Error");
                remote_control_status_label_->setStyleSheet("QLabel { color: red; }");
                
                logMessage("Failed to activate remote control");
            }
        } else {
            remote_control_status_label_->setText("Status: ON (Timeout)");
            remote_control_status_label_->setStyleSheet("QLabel { color: orange; }");
            
            logMessage("Remote control service call timeout");
        }
    } catch (const std::exception& e) {
        remote_control_status_label_->setText("Status: Error");
        remote_control_status_label_->setStyleSheet("QLabel { color: red; }");
        
        logMessage("Remote control service call failed: " + std::string(e.what()));
    }
}

} // namespace aos_planner

