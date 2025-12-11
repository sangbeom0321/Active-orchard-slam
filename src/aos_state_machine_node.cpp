#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/qos.hpp>
#include <cmath>
#include <math.h>

class AosStateMachineNode : public rclcpp::Node {
public:
    AosStateMachineNode() : Node("aos_state_machine_node") {
        // Parameters
        this->declare_parameter<std::string>("path_topic", "/plan");
        this->declare_parameter<std::string>("lidar_odom_topic", "/odometry/imu");
        this->declare_parameter<std::string>("base_link_odom_topic", "/odom_baselink");
        this->declare_parameter<std::string>("control_mod_topic", "/Control/mod");
        this->declare_parameter<std::string>("goal_point_topic", "/Planning/goal_point");
        
        std::string path_topic, lidar_odom_topic, base_link_odom_topic;
        std::string control_mod_topic, goal_point_topic;
        this->get_parameter("path_topic", path_topic);
        this->get_parameter("lidar_odom_topic", lidar_odom_topic);
        this->get_parameter("base_link_odom_topic", base_link_odom_topic);
        this->get_parameter("control_mod_topic", control_mod_topic);
        this->get_parameter("goal_point_topic", goal_point_topic);
        
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Publishers
        mod_pub_ = this->create_publisher<std_msgs::msg::Int32>(control_mod_topic, 10);
        goal_point_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(goal_point_topic, 10);

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic, 10, std::bind(&AosStateMachineNode::pathCallback, this, std::placeholders::_1));
        lidar_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            lidar_odom_topic, qos, std::bind(&AosStateMachineNode::lidarOdomCallback, this, std::placeholders::_1));
        base_link_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            base_link_odom_topic, qos, std::bind(&AosStateMachineNode::baseLinkOdomCallback, this, std::placeholders::_1));

        // Initialize parameters
        current_mode_ = 0;
        is_path_received = false;
        is_precise_task = false;
        skipping_hz_ = 5;
        odom_cnt_ = 0;
        is_goal_initialized_ = false;
        last_goal_point_ = geometry_msgs::msg::Pose();
        last_goal_point_.position.x = 0.0;
        last_goal_point_.position.y = 0.0;
        
        last_three_points.clear();
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg->poses.empty()) {
            last_goal_point_ = goal_point_;
            if (last_goal_point_ != msg->poses.back().pose) {
                goal_point_ = msg->poses.back().pose;  

                global_path_ = *msg;
                
                size_t path_size = msg->poses.size();
                last_three_points.clear();
            	for (size_t i = path_size > 3 ? path_size - 3 : 0; i < path_size; ++i) {
                	last_three_points.push_back(msg->poses[i].pose);
   		}
                is_path_received = true;
                is_goal_initialized_ = true;  // Initialization complete when path is received
            }
        }
    }

    void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        lidar_current_pose_ = msg->pose.pose;
    }
    
    void baseLinkOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_cnt_++;
        if ((odom_cnt_ % skipping_hz_) == 0) {
            odom_cnt_ = 0;
            current_pose_ = msg->pose.pose;
            if (!is_goal_initialized_) {
                // Initial state: path not yet received
                std_msgs::msg::Int32 mod_msg;
                mod_msg.data = 3;
                mod_pub_->publish(mod_msg);
            } else {
                // Path received, update mode normally (works even if goal is (0,0))
                updateControlMode();
            }
        }
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat) {
        tf2::Quaternion tf2_quat;
        tf2::fromMsg(quat, tf2_quat);
        double roll, pitch, yaw;

        tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void updateControlMode() {
        // Calculate the Euclidean distance and yaw difference to the goal
        double distance_to_goal = std::hypot(
            goal_point_.position.x - current_pose_.position.x,
            goal_point_.position.y - current_pose_.position.y);

        double yaw_goal_ = getYawFromQuaternion(goal_point_.orientation);
        double yaw_current = getYawFromQuaternion(current_pose_.orientation);
        double yaw_diff = std::fabs(normalized_angle(yaw_goal_ - yaw_current));

        // Determine the mode based on the conditions
        if ((distance_to_goal < 0.05 && yaw_diff < 0.0524) && (current_mode_ == 1 && is_path_received)) {  // ~3 degrees in radians
            current_mode_ = 3;  // Mod3: Stop mode
            is_path_received = false;
        } else if ((distance_to_goal < 0.1 && yaw_diff < 0.0873) && (current_mode_ == 2 && is_path_received)) {  // ~5 degrees in radians
            current_mode_ = 3;  // Mod3: Stop mode (for Mod2 conditions)
            is_path_received = false;
        } else if ((distance_to_goal < 0.5 && current_mode_ != 3) || findClosestIndex(global_path_, current_pose_) == global_path_.poses.size()) {
            current_mode_ = is_precise_task ? 1 : 2;  // Mod1: Precise, Mod2: Semi-Precise
        } else if ((current_mode_ != 2 && current_mode_ != 1) && is_path_received) {
            current_mode_ = 0;  // Mod0: Path Follower
        }
        
        // Publish control mode (mod)
        std_msgs::msg::Int32 mod_msg;
        mod_msg.data = current_mode_;
        mod_pub_->publish(mod_msg);

        // Publish goal point
        std_msgs::msg::Float64MultiArray goal_point_msg;
        goal_point_msg.data = {goal_point_.position.x, goal_point_.position.y, yaw_goal_};
        goal_point_pub_->publish(goal_point_msg);
    }
    
    int findClosestIndex(const nav_msgs::msg::Path& path, const geometry_msgs::msg::Pose& current_pose) {
        int closest_index = 0;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < path.poses.size(); ++i) {
            // Calculate distance to each point
            double dx = path.poses[i].pose.position.x - current_pose.position.x;
            double dy = path.poses[i].pose.position.y - current_pose.position.y;
            double distance = std::hypot(dx, dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
        
        return closest_index;
    }

    bool areLastThreePointsClosest() {
        if (last_three_points.empty() || !is_path_received) {
            return false;
        }

        // Find the three closest points to the current odom position
        std::vector<std::pair<double, geometry_msgs::msg::Pose>> distances;
        for (const auto& pose : last_three_points) {
            double distance = std::hypot(
                pose.position.x - lidar_current_pose_.position.x,
                pose.position.y - lidar_current_pose_.position.y
            );
            distances.emplace_back(distance, pose);
        }

        // Sort distances to find the three closest points
        std::sort(distances.begin(), distances.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        std::vector<geometry_msgs::msg::Pose> closest_three_points;
        for (size_t i = 0; i < 3 && i < distances.size(); ++i) {
            closest_three_points.push_back(distances[i].second);
        }

        // Check if the stored last three points are the same as the closest three points
        for (const auto& last_point : last_three_points) {
            if (std::find(closest_three_points.begin(), closest_three_points.end(), last_point) == closest_three_points.end()) {
                return false;
            }
        }

        return true;
    }
    
    double normalized_angle(double angle) {
    	if (angle > M_PI) {
    		return angle - 2 * M_PI;
    	}
    	if (angle < -M_PI) {
    		return angle + 2 * M_PI;
    	}
    	return angle;
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mod_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_point_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_link_odom_sub_;

    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose lidar_current_pose_;
    geometry_msgs::msg::Pose goal_point_;
    geometry_msgs::msg::Pose last_goal_point_;

    nav_msgs::msg::Path global_path_;

    int current_mode_;
    int skipping_hz_;
    int odom_cnt_;
    double yaw_goal_;
    bool is_path_received;
    bool is_precise_task;
    bool is_goal_initialized_;  // Flag to check if goal is initialized
    
    std::vector<geometry_msgs::msg::Pose> last_three_points;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AosStateMachineNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}