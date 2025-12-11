#include <memory>
#include <string>
#include <queue>
#include <cmath>
#include <vector>
#include <limits>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <nlohmann/json.hpp>

using std::placeholders::_1;
using GeographicLib::UTMUPS;

// Structure to store UTM coordinates
struct UtmData {
  double x;
  double y;
  double z;
  rclcpp::Time timestamp;
  
  UtmData() : x(0.0), y(0.0), z(0.0), timestamp(0, 0) {}
  
  UtmData(double utm_x, double utm_y, double utm_z, const rclcpp::Time& ts)
    : x(utm_x), y(utm_y), z(utm_z), timestamp(ts) {}
};

// 2D transformation parameters (UTM -> base_link)
struct Transform2D {
  double tx;      // Translation in x direction
  double ty;      // Translation in y direction
  double theta;   // Rotation angle (radians)
  
  Transform2D() : tx(0.0), ty(0.0), theta(0.0) {}
  Transform2D(double t_x, double t_y, double t_theta) 
    : tx(t_x), ty(t_y), theta(t_theta) {}
  
  // Transform UTM coordinates to base_link coordinates
  void transform(double utm_x, double utm_y, double& base_x, double& base_y) const {
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    base_x = cos_t * utm_x - sin_t * utm_y + tx;
    base_y = sin_t * utm_x + cos_t * utm_y + ty;
  }
};

class GpsToUtmNode : public rclcpp::Node
{
public:
  GpsToUtmNode()
  : Node("gps_to_utm_node")
  {
    // Parameters
    this->declare_parameter<int>("utm_zone", 52);
    this->declare_parameter<std::string>("gps_topic", "/fix");
    this->declare_parameter<std::string>("odom_topic", "/odom_baselink");
    this->declare_parameter<int>("queue_size", 100);
    this->declare_parameter<double>("gps_offset_x", -0.65);
    this->declare_parameter<double>("gps_offset_y", 0.55);
    this->declare_parameter<double>("timestamp_offset", 19379697.032363);
    this->declare_parameter<std::string>("exploration_area_topic", "/aos_planner/exploration_area");
    this->declare_parameter<std::string>("gps_polygon_json_path", "/root/ros2_ws/src/aos/config/gps_polygon.json");

    utm_zone_ = this->get_parameter("utm_zone").as_int();
    gps_topic_ = this->get_parameter("gps_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    queue_size_ = this->get_parameter("queue_size").as_int();
    gps_offset_x_ = this->get_parameter("gps_offset_x").as_double();
    gps_offset_y_ = this->get_parameter("gps_offset_y").as_double();
    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
    exploration_area_topic_ = this->get_parameter("exploration_area_topic").as_string();
    gps_polygon_json_path_ = this->get_parameter("gps_polygon_json_path").as_string();

    RCLCPP_INFO(get_logger(), "Subscribing GPS: %s", gps_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing Odometry: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing Exploration Area: %s", exploration_area_topic_.c_str());

    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_, 10, std::bind(&GpsToUtmNode::gpsCallback, this, _1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&GpsToUtmNode::odomCallback, this, _1));

    pub_exploration_area_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      exploration_area_topic_, 10);

    // Check if file exists
    std::ifstream test_file(gps_polygon_json_path_);
    if (!test_file.is_open()) {
      RCLCPP_WARN(get_logger(), "GPS polygon JSON file does not exist at: %s", gps_polygon_json_path_.c_str());
    } else {
      test_file.close();
      RCLCPP_INFO(get_logger(), "GPS polygon JSON file found: %s", gps_polygon_json_path_.c_str());
    }

    // Initialize
    initial_transform_computed_ = false;
    first_base_link_received_ = false;
    first_utm_received_ = false;
    polygon_converted_ = false;
  }

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "GPS has no fix, skipping...");
      return;
    }

    double lat = msg->latitude;
    double lon = msg->longitude;

    // Basic validation
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
      RCLCPP_WARN(get_logger(), "Invalid lat/lon: (%.10f, %.10f)", lat, lon);
      return;
    }

    // NaN check
    if (std::isnan(lat) || std::isnan(lon)) {
      RCLCPP_WARN(get_logger(), "GPS coordinates are NaN");
      return;
    }

    // Skip if coordinates are (0,0) (uninitialized GPS)
    if (std::abs(lat) < 1e-6 && std::abs(lon) < 1e-6) {
      RCLCPP_DEBUG(get_logger(), "GPS coordinates not initialized (0,0)");
      return;
    }

    double x = 0.0;
    double y = 0.0;
    int zone = utm_zone_;
    bool northp = true;

    try {
      if (utm_zone_ == 0) {
        UTMUPS::Forward(lat, lon, zone, northp, x, y, UTMUPS::STANDARD);
      } else {
        UTMUPS::Forward(lat, lon, zone, northp, x, y, utm_zone_);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "UTM conversion failed: %s", e.what());
      return;
    }

    rclcpp::Time gps_time(msg->header.stamp);
    rclcpp::Time adjusted_gps_time = gps_time + rclcpp::Duration::from_seconds(timestamp_offset_);

    // Store first UTM data
    if (!first_utm_received_) {
      first_utm_.x = x;
      first_utm_.y = y;
      first_utm_.z = msg->altitude;
      first_utm_.timestamp = adjusted_gps_time;
      first_utm_received_ = true;
      RCLCPP_INFO(get_logger(), "First UTM data stored: (%.3f, %.3f)", x, y);
    }

    // Store UTM data in queue
    if (utm_queue_.size() >= static_cast<size_t>(queue_size_)) {
      utm_queue_.pop();
    }
    utm_queue_.push(UtmData(x, y, msg->altitude, adjusted_gps_time));
  }

  // Apply GPS offset considering base_link orientation
  void applyGpsOffset(double offset_x, double offset_y, double& rotated_x, double& rotated_y) const
  {
    double qx = latest_base_link_orientation_.qx;
    double qy = latest_base_link_orientation_.qy;
    double qz = latest_base_link_orientation_.qz;
    double qw = latest_base_link_orientation_.qw;
    
    // Consider only 2D rotation (z-axis rotation, yaw)
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    
    rotated_x = cos_yaw * offset_x - sin_yaw * offset_y;
    rotated_y = sin_yaw * offset_x + cos_yaw * offset_y;
  }

  // Convert GPS longitude/latitude to base_link coordinate system using RT transformation matrix
  bool gpsToBaseLinkCoordinates(double lon, double lat, double& base_x, double& base_y) const
  {
    // Validate GPS coordinates
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
      return false;
    }
    
    if (std::isnan(lat) || std::isnan(lon)) {
      return false;
    }
    
    // GPS -> UTM conversion
    double utm_x = 0.0;
    double utm_y = 0.0;
    int zone = utm_zone_;
    bool northp = true;
    
    try {
      if (utm_zone_ == 0) {
        UTMUPS::Forward(lat, lon, zone, northp, utm_x, utm_y, UTMUPS::STANDARD);
      } else {
        UTMUPS::Forward(lat, lon, zone, northp, utm_x, utm_y, utm_zone_);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "UTM conversion failed: %s", e.what());
      return false;
    }
    
    if (!initial_transform_computed_) {
      return false;
    }
    
    // Transform UTM -> base_link using RT transformation matrix
    utm_to_base_transform_.transform(utm_x, utm_y, base_x, base_y);
    
    return true;
  }

  // Read GPS polygon JSON file and convert
  void convertGpsPolygonFromJson()
  {
    std::ifstream file(gps_polygon_json_path_);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open GPS polygon JSON file: %s", gps_polygon_json_path_.c_str());
      return;
    }
    
    try {
      nlohmann::json json_data;
      file >> json_data;
      file.close();
      
      if (!json_data.contains("points") || !json_data["points"].is_array()) {
        RCLCPP_ERROR(get_logger(), "Invalid JSON format: 'points' array not found");
        return;
      }
      
      std::vector<double> latitudes;
      std::vector<double> longitudes;
      std::vector<double> base_x;
      std::vector<double> base_y;
      std::vector<bool> conversion_success;
      
      // Extract GPS coordinates
      for (const auto& point : json_data["points"]) {
        if (point.contains("latitude") && point.contains("longitude")) {
          double lat = point["latitude"].get<double>();
          double lon = point["longitude"].get<double>();
          latitudes.push_back(lat);
          longitudes.push_back(lon);
        }
      }
      
      if (latitudes.empty()) {
        RCLCPP_WARN(get_logger(), "No valid GPS points found in JSON file");
        return;
      }
      
      RCLCPP_INFO(get_logger(), "Reading %zu GPS points from JSON file", latitudes.size());
      
      // Convert each coordinate
      base_x.resize(latitudes.size());
      base_y.resize(latitudes.size());
      conversion_success.resize(latitudes.size());
      
      size_t success_count = 0;
      for (size_t i = 0; i < latitudes.size(); ++i) {
        double base_x_val, base_y_val;
        bool base_success = gpsToBaseLinkCoordinates(longitudes[i], latitudes[i], base_x_val, base_y_val);
        
        if (base_success) {
          base_x[i] = base_x_val;
          base_y[i] = base_y_val;
          conversion_success[i] = true;
          success_count++;
        } else {
          base_x[i] = 0.0;
          base_y[i] = 0.0;
          conversion_success[i] = false;
        }
      }
      
      RCLCPP_INFO(get_logger(), 
        "GPS polygon conversion completed: %zu/%zu points converted successfully", 
        success_count, latitudes.size());
      
      // Publish converted coordinates as PolygonStamped
      publishExplorationAreaPolygon(base_x, base_y, conversion_success);
      
      polygon_converted_ = true;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error parsing JSON file: %s", e.what());
      file.close();
    }
  }

  // Publish converted coordinates as PolygonStamped (exploration area)
  void publishExplorationAreaPolygon(
    const std::vector<double>& base_x,
    const std::vector<double>& base_y,
    const std::vector<bool>& conversion_success)
  {
    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = this->now();
    polygon_msg.header.frame_id = "odom";
    
    // Add only successfully converted points
    for (size_t i = 0; i < base_x.size(); ++i) {
      if (conversion_success[i] && base_x[i] != 0.0 && base_y[i] != 0.0) {
        geometry_msgs::msg::Point32 point;
        point.x = static_cast<float>(base_x[i]);
        point.y = static_cast<float>(base_y[i]);
        point.z = 0.0f;
        polygon_msg.polygon.points.push_back(point);
      }
    }
    
    if (polygon_msg.polygon.points.size() >= 3) {
      pub_exploration_area_->publish(polygon_msg);
      RCLCPP_INFO(get_logger(), 
        "Published exploration area polygon with %zu points", 
        polygon_msg.polygon.points.size());
    } else {
      RCLCPP_WARN(get_logger(), 
        "Not enough valid points (%zu) to publish exploration area polygon (need at least 3)", 
        polygon_msg.polygon.points.size());
    }
  }

  // Odometry callback function
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time odom_time(msg->header.stamp);
    double base_x = msg->pose.pose.position.x;
    double base_y = msg->pose.pose.position.y;
    
    // Store latest base_link orientation
    latest_base_link_orientation_.qx = msg->pose.pose.orientation.x;
    latest_base_link_orientation_.qy = msg->pose.pose.orientation.y;
    latest_base_link_orientation_.qz = msg->pose.pose.orientation.z;
    latest_base_link_orientation_.qw = msg->pose.pose.orientation.w;
    
    // Apply GPS offset considering base_link orientation
    double rotated_offset_x, rotated_offset_y;
    applyGpsOffset(gps_offset_x_, gps_offset_y_, rotated_offset_x, rotated_offset_y);
    
    // GPS receiver position = base_link position + rotated offset
    double gps_receiver_x = base_x + rotated_offset_x;
    double gps_receiver_y = base_y + rotated_offset_y;
    
    // Store first GPS receiver position
    if (!first_base_link_received_) {
      initial_base_link_.x = gps_receiver_x;
      initial_base_link_.y = gps_receiver_y;
      initial_base_link_.timestamp = odom_time;
      first_base_link_received_ = true;
      RCLCPP_INFO(get_logger(), "First GPS receiver position stored: (%.3f, %.3f)", 
        gps_receiver_x, gps_receiver_y);
    }
    
    // Check if initial transform is not yet computed and moved more than 5m
    if (!initial_transform_computed_ && first_base_link_received_ && first_utm_received_) {
      double dx = gps_receiver_x - initial_base_link_.x;
      double dy = gps_receiver_y - initial_base_link_.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      
      if (distance >= 5.0) {
        RCLCPP_INFO(get_logger(), 
          "GPS receiver moved more than 5m (distance: %.3f m). Computing initial transform...", distance);
        
        // Find UTM for current position (matching by timestamp)
        UtmData current_utm;
        bool utm_found = findMatchingUtm(odom_time, current_utm);
        
        if (utm_found) {
          computeInitialTransform(
            initial_base_link_.x, initial_base_link_.y,
            first_utm_.x, first_utm_.y,
            gps_receiver_x, gps_receiver_y,
            current_utm.x, current_utm.y
          );
          initial_transform_computed_ = true;
          RCLCPP_INFO(get_logger(), 
            "Initial transform computed: tx=%.3f, ty=%.3f, theta=%.6f (deg: %.3f)",
            utm_to_base_transform_.tx, 
            utm_to_base_transform_.ty,
            utm_to_base_transform_.theta,
            utm_to_base_transform_.theta * 180.0 / M_PI);
        } else {
          RCLCPP_WARN(get_logger(), 
            "Cannot find matching UTM data. Odom time: %.6f, UTM queue size: %zu",
            odom_time.seconds(), utm_queue_.size());
        }
      }
    }
    
    // Read and convert GPS polygon JSON file after initial transform is complete
    if (initial_transform_computed_ && !polygon_converted_) {
      convertGpsPolygonFromJson();
    }
  }

  // Find UTM data closest to timestamp
  bool findMatchingUtm(const rclcpp::Time& target_time, UtmData& result)
  {
    if (utm_queue_.empty()) {
      return false;
    }
    
    double min_diff = std::numeric_limits<double>::max();
    UtmData best_match = utm_queue_.front();
    
    std::queue<UtmData> temp_queue = utm_queue_;
    while (!temp_queue.empty()) {
      UtmData utm = temp_queue.front();
      temp_queue.pop();
      
      double time_diff = std::abs((target_time - utm.timestamp).seconds());
      if (time_diff < min_diff) {
        min_diff = time_diff;
        best_match = utm;
      }
    }
    
    result = best_match;
    return true;
  }

  // Compute initial transformation
  void computeInitialTransform(
    double base_x0, double base_y0,
    double utm_x0, double utm_y0,
    double base_x1, double base_y1,
    double utm_x1, double utm_y1)
  {
    // Compute transformation using two points
    double base_dx = base_x1 - base_x0;
    double base_dy = base_y1 - base_y0;
    
    double utm_dx = utm_x1 - utm_x0;
    double utm_dy = utm_y1 - utm_y0;
    
    // Calculate rotation angle
    double base_angle = std::atan2(base_dy, base_dx);
    double utm_angle = std::atan2(utm_dy, utm_dx);
    double theta = base_angle - utm_angle;
    
    // Calculate translation
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    
    double tx0 = base_x0 - (cos_t * utm_x0 - sin_t * utm_y0);
    double ty0 = base_y0 - (sin_t * utm_x0 + cos_t * utm_y0);
    
    double tx1 = base_x1 - (cos_t * utm_x1 - sin_t * utm_y1);
    double ty1 = base_y1 - (sin_t * utm_x1 + cos_t * utm_y1);
    
    double tx = (tx0 + tx1) / 2.0;
    double ty = (ty0 + ty1) / 2.0;
    
    utm_to_base_transform_ = Transform2D(tx, ty, theta);
  }

  int utm_zone_;
  std::string gps_topic_;
  std::string odom_topic_;
  std::string exploration_area_topic_;
  std::string gps_polygon_json_path_;
  int queue_size_;
  double gps_offset_x_;
  double gps_offset_y_;
  double timestamp_offset_;
  bool polygon_converted_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_exploration_area_;

  // UTM data queue
  std::queue<UtmData> utm_queue_;

  // Transformation-related variables
  Transform2D utm_to_base_transform_;
  bool initial_transform_computed_;
  bool first_base_link_received_;
  bool first_utm_received_;
  
  struct {
    double x, y;
    rclcpp::Time timestamp;
  } initial_base_link_;
  
  UtmData first_utm_;
  
  // Latest base_link orientation
  struct Orientation {
    double qx, qy, qz, qw;
    Orientation() : qx(0.0), qy(0.0), qz(0.0), qw(1.0) {}
  } latest_base_link_orientation_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsToUtmNode>());
  rclcpp::shutdown();
  return 0;
}
