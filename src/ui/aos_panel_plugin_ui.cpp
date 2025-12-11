#include "aos/aos_panel_plugin.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QScrollArea>
#include <QGroupBox>
#include <QGridLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QProgressBar>
#include <QTabWidget>
#include <QTextOption>

namespace aos_planner {

void AosPanelPlugin::setupUI() {
    // Create main layout only if not already created
    if (!main_layout_) {
        main_layout_ = new QVBoxLayout(this);
        setLayout(main_layout_);
    }
    
    // Set size for the panel to prevent horizontal scrolling
    this->setMinimumWidth(600); // Increase minimum width
    this->setMaximumWidth(800); // Increase maximum width
    this->setMaximumHeight(1200); // Increased height to reduce scrolling
    
    // Create tab widget only if not already created
    if (!tab_widget_) {
        tab_widget_ = new QTabWidget();
        tab_widget_->setMaximumHeight(1150); // Increased tab widget height
        main_layout_->addWidget(tab_widget_);
    }
    
    // Setup tabs only if not already created (order: Control - Exploration Area - Parameters)
    if (!status_tab_) {
        setupStatusTab();
    }
    if (!control_tab_) {
        setupControlTab();
    }
    if (!parameters_tab_) {
        setupParametersTab();
    }
}

void AosPanelPlugin::setupControlTab() {
    control_tab_ = new QWidget();
    tab_widget_->addTab(control_tab_, "Control");
    
    // Create scroll area for control tab
    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); // Disable horizontal scroll
    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    QWidget* scroll_widget = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(scroll_widget);
    
    // Set scroll area as the main widget for control tab
    QVBoxLayout* tab_layout = new QVBoxLayout(control_tab_);
    tab_layout->addWidget(scroll_area);
    scroll_area->setWidget(scroll_widget);
    
    layout->addStretch();
}

void AosPanelPlugin::setupStatusTab() {
    status_tab_ = new QWidget();
    tab_widget_->addTab(status_tab_, "Control");
    
    // Create scroll area for status tab
    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); // Disable horizontal scroll
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
    status_layout->addWidget(status_label_);
    topic_status_label_ = new QLabel("Topic Status - GPS: NO DATA");
    status_layout->addWidget(topic_status_label_);
    
    // Remote Control Group (moved from Control tab)
    remote_control_group_ = new QGroupBox("Remote Control");
    QVBoxLayout* remote_layout = new QVBoxLayout(remote_control_group_);
    QHBoxLayout* remote_buttons = new QHBoxLayout();
    remote_control_off_btn_ = new QPushButton("Remote Control OFF");
    remote_control_off_btn_->setStyleSheet("QPushButton { background-color: #ff6b6b; color: white; }");
    remote_control_on_btn_ = new QPushButton("Remote Control ON");
    remote_control_on_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
    remote_buttons->addWidget(remote_control_off_btn_);
    remote_buttons->addWidget(remote_control_on_btn_);
    remote_layout->addLayout(remote_buttons);
    remote_control_status_label_ = new QLabel("Status: Unknown");
    remote_control_status_label_->setStyleSheet("QLabel { color: orange; }");
    remote_layout->addWidget(remote_control_status_label_);
    status_layout->addWidget(remote_control_group_);

    layout->addWidget(status_group);
    
    // Save Map Group
    QGroupBox* save_map_group = new QGroupBox("Map Management");
    QVBoxLayout* save_map_layout = new QVBoxLayout(save_map_group);
    save_map_btn_ = new QPushButton("Save Map");
    save_map_btn_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }");
    save_map_layout->addWidget(save_map_btn_);
    layout->addWidget(save_map_group);
    
    // Planning Status Group
    planning_status_group_ = new QGroupBox("Planning Status");
    QVBoxLayout* planning_status_layout = new QVBoxLayout(planning_status_group_);
    
    planning_current_pos_label_ = new QLabel("Current Position: (0.00, 0.00)");
    planning_status_layout->addWidget(planning_current_pos_label_);
    
    planning_control_mode_label_ = new QLabel("Control Mode: Unknown");
    planning_status_layout->addWidget(planning_control_mode_label_);
    
    planning_cluster_label_ = new QLabel("Current Cluster: -");
    planning_status_layout->addWidget(planning_cluster_label_);
    
    planning_waypoint_label_ = new QLabel("Current Waypoint: -");
    planning_status_layout->addWidget(planning_waypoint_label_);
    
    planning_path_status_label_ = new QLabel("Path Planning Status: Unknown");
    planning_path_status_label_->setStyleSheet("QLabel { color: orange; font-weight: bold; }");
    planning_status_layout->addWidget(planning_path_status_label_);
    
    // Progress bar for remaining waypoints
    planning_progress_label_ = new QLabel("Progress: 0%");
    planning_status_layout->addWidget(planning_progress_label_);
    
    planning_progress_bar_ = new QProgressBar();
    planning_progress_bar_->setRange(0, 100);
    planning_progress_bar_->setValue(0);
    planning_progress_bar_->setTextVisible(true);
    planning_progress_bar_->setFormat("%p%");
    planning_status_layout->addWidget(planning_progress_bar_);
    
    layout->addWidget(planning_status_group_);
    
    // Log display
    QGroupBox* log_group = new QGroupBox("Log");
    QVBoxLayout* log_layout = new QVBoxLayout(log_group);
    
    log_text_ = new QTextEdit();
    log_text_->setReadOnly(true);
    log_text_->setMinimumHeight(400);
    log_text_->setMaximumHeight(600); // Increased height to show more log content
    log_text_->setWordWrapMode(QTextOption::WordWrap);
    log_layout->addWidget(log_text_);
    
    layout->addWidget(log_group);
}

void AosPanelPlugin::setupParametersTab() {
    parameters_tab_ = new QWidget();
    tab_widget_->addTab(parameters_tab_, "Parameters");
    
    // Create scroll area for parameters
    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); // Disable horizontal scroll
    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    // Set maximum height for scroll area to prevent panel from being too tall
    scroll_area->setMaximumHeight(1000); // Increased height to reduce scrolling
    scroll_area->setMinimumHeight(600); // Increased minimum height
    
    QWidget* scroll_widget = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(scroll_widget);
    
    // Set scroll area as the main widget for parameters tab
    QVBoxLayout* tab_layout = new QVBoxLayout(parameters_tab_);
    tab_layout->addWidget(scroll_area);
    scroll_area->setWidget(scroll_widget);
    
    int row = 0;
    
    // Occupancy Grid Generator Parameters
    QGroupBox* occupancy_group = new QGroupBox("Occupancy Grid Generator Parameters");
    QGridLayout* occupancy_layout = new QGridLayout(occupancy_group);
    
    row = 0;
    occupancy_layout->addWidget(new QLabel("Clipping Min Z (m):"), row, 0);
    clipping_minz_spin_ = new QDoubleSpinBox();
    clipping_minz_spin_->setRange(-2.0, 0.0);
    clipping_minz_spin_->setSingleStep(0.1);
    clipping_minz_spin_->setValue(-0.4);
    occupancy_layout->addWidget(clipping_minz_spin_, row++, 1);
    
    occupancy_layout->addWidget(new QLabel("Clipping Max Z (m):"), row, 0);
    clipping_maxz_spin_ = new QDoubleSpinBox();
    clipping_maxz_spin_->setRange(0.0, 2.0);
    clipping_maxz_spin_->setSingleStep(0.1);
    clipping_maxz_spin_->setValue(0.1);
    occupancy_layout->addWidget(clipping_maxz_spin_, row++, 1);
    
    occupancy_layout->addWidget(new QLabel("Clipping Min X (m):"), row, 0);
    clipping_minx_spin_ = new QDoubleSpinBox();
    clipping_minx_spin_->setRange(-100.0, 0.0);
    clipping_minx_spin_->setSingleStep(1.0);
    clipping_minx_spin_->setValue(-5.0);
    occupancy_layout->addWidget(clipping_minx_spin_, row++, 1);
    
    occupancy_layout->addWidget(new QLabel("Clipping Max X (m):"), row, 0);
    clipping_maxx_spin_ = new QDoubleSpinBox();
    clipping_maxx_spin_->setRange(0.0, 200.0);
    clipping_maxx_spin_->setSingleStep(1.0);
    clipping_maxx_spin_->setValue(72.0);
    occupancy_layout->addWidget(clipping_maxx_spin_, row++, 1);
    
    occupancy_layout->addWidget(new QLabel("Clipping Min Y (m):"), row, 0);
    clipping_miny_spin_ = new QDoubleSpinBox();
    clipping_miny_spin_->setRange(-100.0, 0.0);
    clipping_miny_spin_->setSingleStep(1.0);
    clipping_miny_spin_->setValue(-3.0);
    occupancy_layout->addWidget(clipping_miny_spin_, row++, 1);
    
    occupancy_layout->addWidget(new QLabel("Clipping Max Y (m):"), row, 0);
    clipping_maxy_spin_ = new QDoubleSpinBox();
    clipping_maxy_spin_->setRange(0.0, 100.0);
    clipping_maxy_spin_->setSingleStep(1.0);
    clipping_maxy_spin_->setValue(10.0);
    occupancy_layout->addWidget(clipping_maxy_spin_, row++, 1);
    
    occupancy_layout->addWidget(new QLabel("Grid Resolution (m):"), row, 0);
    grid_resolution_spin_ = new QDoubleSpinBox();
    grid_resolution_spin_->setRange(0.01, 1.0);
    grid_resolution_spin_->setSingleStep(0.01);
    grid_resolution_spin_->setValue(0.05);
    occupancy_layout->addWidget(grid_resolution_spin_, row++, 1);
    
    occupancy_layout->addWidget(new QLabel("Inflation Radius (m):"), row, 0);
    inflation_radius_spin_ = new QDoubleSpinBox();
    inflation_radius_spin_->setRange(0.1, 5.0);
    inflation_radius_spin_->setSingleStep(0.1);
    inflation_radius_spin_->setValue(0.5);
    occupancy_layout->addWidget(inflation_radius_spin_, row++, 1);
    
    occupancy_layout->addWidget(new QLabel("Waypoint Offset Distance (m):"), row, 0);
    waypoint_offset_distance_spin_ = new QDoubleSpinBox();
    waypoint_offset_distance_spin_->setRange(0.1, 10.0);
    waypoint_offset_distance_spin_->setSingleStep(0.1);
    waypoint_offset_distance_spin_->setValue(2.0);
    occupancy_layout->addWidget(waypoint_offset_distance_spin_, row++, 1);
    
    layout->addWidget(occupancy_group);
    
    // Path Smoothing Parameters
    QGroupBox* smoothing_group = new QGroupBox("Path Smoothing Parameters");
    QGridLayout* smoothing_layout = new QGridLayout(smoothing_group);
    
    row = 0;
    smoothing_layout->addWidget(new QLabel("Interpolation Distance (m):"), row, 0);
    interpolation_distance_spin_ = new QDoubleSpinBox();
    interpolation_distance_spin_->setRange(0.01, 1.0);
    interpolation_distance_spin_->setSingleStep(0.01);
    interpolation_distance_spin_->setValue(0.05);
    smoothing_layout->addWidget(interpolation_distance_spin_, row++, 1);
    
    smoothing_layout->addWidget(new QLabel("Smoothing Window Size:"), row, 0);
    smoothing_window_size_spin_ = new QSpinBox();
    smoothing_window_size_spin_->setRange(1, 10);
    smoothing_window_size_spin_->setValue(3);
    smoothing_layout->addWidget(smoothing_window_size_spin_, row++, 1);
    
    layout->addWidget(smoothing_group);
    
    // GVD Planning Parameters
    QGroupBox* planning_group = new QGroupBox("GVD Planning Parameters");
    QGridLayout* planning_layout = new QGridLayout(planning_group);
    
    row = 0;
    planning_layout->addWidget(new QLabel("Planning Rate (Hz):"), row, 0);
    planning_rate_spin_ = new QDoubleSpinBox();
    planning_rate_spin_->setRange(0.1, 10.0);
    planning_rate_spin_->setSingleStep(0.1);
    planning_rate_spin_->setValue(1.0);
    planning_layout->addWidget(planning_rate_spin_, row++, 1);
    
    layout->addWidget(planning_group);
    
    // GVD Topology parameters group
    QGroupBox* gvd_group = new QGroupBox("GVD Topology Parameters");
    QGridLayout* gvd_layout = new QGridLayout(gvd_group);
    gvd_group->setToolTip("Connectivity options for the GVD graph (radius in meters).");
    
    row = 0;
    gvd_layout->addWidget(new QLabel("Connect Radius (m):"), row, 0);
    connect_radius_spin_ = new QDoubleSpinBox();
    connect_radius_spin_->setRange(0.1, 5.0);
    connect_radius_spin_->setSingleStep(0.1);
    connect_radius_spin_->setValue(1.5);
    connect_radius_spin_->setToolTip("Max radius (in meters) to allow edges between GVD nodes.");
    gvd_layout->addWidget(connect_radius_spin_, row++, 1);
    
    layout->addWidget(gvd_group);
    
    // Modify parameters button
    modify_params_btn_ = new QPushButton("Modify Parameters");
    layout->addWidget(modify_params_btn_);
    
    layout->addStretch();
}

void AosPanelPlugin::connectSignals() {
    // Connect button signals
    connect(remote_control_off_btn_, &QPushButton::clicked, this, &AosPanelPlugin::onRemoteControlOff);
    connect(remote_control_on_btn_, &QPushButton::clicked, this, &AosPanelPlugin::onRemoteControlOn);
    connect(save_map_btn_, &QPushButton::clicked, this, &AosPanelPlugin::onSaveMap);
    
    // Modify button
    connect(modify_params_btn_, &QPushButton::clicked, this, &AosPanelPlugin::onModifyParameters);
}

} // namespace aos_planner

