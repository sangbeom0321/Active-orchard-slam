#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "aos/orbit_voxblox_interface.hpp"

class ESDFTestNode : public rclcpp::Node {
public:
    ESDFTestNode() : Node("esdf_test_node") {
        // Initialize voxblox interface
        voxblox_interface_ = std::make_unique<orbit_planner::OrbitVoxbloxInterface>();
        voxblox_interface_->initialize(0.1, 0.2);  // 10cm voxel, 20cm truncation
        
        // Create subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lio_sam/mapping/map_global", 10,
            std::bind(&ESDFTestNode::pointCloudCallback, this, std::placeholders::_1));
        
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lio_sam/mapping/odometry", 10,
            std::bind(&ESDFTestNode::odometryCallback, this, std::placeholders::_1));
        
        // Create publishers
        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/esdf_test/occupancy_grid", 10);
        
        esdf_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/esdf_test/esdf_markers", 10);
        
        // Create timer for periodic ESDF update and visualization
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 2Hz
            std::bind(&ESDFTestNode::updateAndVisualize, this));
        
        RCLCPP_INFO(this->get_logger(), "ESDF Test Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "  - /lio_sam/mapping/map_global");
        RCLCPP_INFO(this->get_logger(), "  - /lio_sam/mapping/odometry");
        RCLCPP_INFO(this->get_logger(), "Publishing to:");
        RCLCPP_INFO(this->get_logger(), "  - /esdf_test/occupancy_grid");
        RCLCPP_INFO(this->get_logger(), "  - /esdf_test/esdf_markers");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->size());
        
        // Get robot pose
        geometry_msgs::msg::PoseStamped robot_pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!robot_pose_received_) {
                RCLCPP_WARN(this->get_logger(), "Robot pose not available, skipping point cloud integration");
                return;
            }
            robot_pose = current_robot_pose_;
            robot_pose.header = msg->header;  // Use point cloud timestamp
        }
        
        // Integrate point cloud
        voxblox_interface_->integratePointCloud(cloud, robot_pose);
        
        // Update ESDF
        voxblox_interface_->updateESDF();
        
        RCLCPP_INFO(this->get_logger(), "Point cloud integrated and ESDF updated");
    }
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_robot_pose_.header = msg->header;
        current_robot_pose_.pose = msg->pose.pose;
        robot_pose_received_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "Robot pose updated: (%.2f, %.2f, %.2f)", 
                    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }
    
    void updateAndVisualize() {
        if (!robot_pose_received_) {
            return;
        }
        
        // Generate occupancy grid
        geometry_msgs::msg::Point origin;
        origin.x = -50.0;
        origin.y = -50.0;
        origin.z = 0.0;
        
        auto occupancy_grid = voxblox_interface_->generateOccupancyGrid(origin, 0.1, 1000, 1000);
        occupancy_grid_pub_->publish(occupancy_grid);
        
        // Generate ESDF markers for visualization
        publishESDFMarkers();
        
        RCLCPP_DEBUG(this->get_logger(), "ESDF visualization updated");
    }
    
    void publishESDFMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Get map bounds
        geometry_msgs::msg::Point min_point, max_point;
        voxblox_interface_->getMapBounds(min_point, max_point);
        
        // Create markers for occupied voxels
        visualization_msgs::msg::Marker occupied_marker;
        occupied_marker.header.frame_id = "map";
        occupied_marker.header.stamp = this->now();
        occupied_marker.ns = "esdf_occupied";
        occupied_marker.id = 0;
        occupied_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        occupied_marker.action = visualization_msgs::msg::Marker::ADD;
        occupied_marker.scale.x = 0.1;  // Voxel size
        occupied_marker.scale.y = 0.1;
        occupied_marker.scale.z = 0.1;
        occupied_marker.color.r = 1.0;
        occupied_marker.color.g = 0.0;
        occupied_marker.color.b = 0.0;
        occupied_marker.color.a = 0.8;
        
        // Sample points in the map and check if they are occupied
        double resolution = 0.2;  // 20cm sampling
        for (double x = min_point.x; x <= max_point.x; x += resolution) {
            for (double y = min_point.y; y <= max_point.y; y += resolution) {
                for (double z = min_point.z; z <= max_point.z; z += resolution) {
                    geometry_msgs::msg::Point test_point;
                    test_point.x = x;
                    test_point.y = y;
                    test_point.z = z;
                    
                    double distance = voxblox_interface_->getDistance(test_point);
                    if (distance < 0.5) {  // Close to obstacle
                        occupied_marker.points.push_back(test_point);
                    }
                }
            }
        }
        
        marker_array.markers.push_back(occupied_marker);
        
        // Create markers for free space
        visualization_msgs::msg::Marker free_marker;
        free_marker.header.frame_id = "map";
        free_marker.header.stamp = this->now();
        free_marker.ns = "esdf_free";
        free_marker.id = 1;
        free_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        free_marker.action = visualization_msgs::msg::Marker::ADD;
        free_marker.scale.x = 0.1;
        free_marker.scale.y = 0.1;
        free_marker.scale.z = 0.1;
        free_marker.color.r = 0.0;
        free_marker.color.g = 1.0;
        free_marker.color.b = 0.0;
        free_marker.color.a = 0.3;
        
        // Sample points for free space
        for (double x = min_point.x; x <= max_point.x; x += resolution) {
            for (double y = min_point.y; y <= max_point.y; y += resolution) {
                for (double z = min_point.z; z <= max_point.z; z += resolution) {
                    geometry_msgs::msg::Point test_point;
                    test_point.x = x;
                    test_point.y = y;
                    test_point.z = z;
                    
                    double distance = voxblox_interface_->getDistance(test_point);
                    if (distance >= 0.5 && distance < 2.0) {  // Free space
                        free_marker.points.push_back(test_point);
                    }
                }
            }
        }
        
        marker_array.markers.push_back(free_marker);
        
        esdf_marker_pub_->publish(marker_array);
    }
    
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr esdf_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Voxblox interface
    std::unique_ptr<orbit_planner::OrbitVoxbloxInterface> voxblox_interface_;
    
    // Robot pose
    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseStamped current_robot_pose_;
    bool robot_pose_received_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ESDFTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
