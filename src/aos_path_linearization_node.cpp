#include <cmath>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class AosPathLinearizationNode : public rclcpp::Node {
public:
  AosPathLinearizationNode() : rclcpp::Node("aos_path_linearization_node") {
    // Parameters
    this->declare_parameter("input_path_topic", std::string("/aos/path"));
    this->declare_parameter("output_path_topic", std::string("/plan"));
    
    this->get_parameter("input_path_topic", input_path_topic_);
    this->get_parameter("output_path_topic", output_path_topic_);
    
    // Subscribers
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        input_path_topic_, 10,
        std::bind(&AosPathLinearizationNode::pathCallback, this, std::placeholders::_1));
    
    // Publishers
    pub_path_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, 10);

    publish_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AosPathLinearizationNode::publishCachedPath, this));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  std::string input_path_topic_;
  std::string output_path_topic_;
  nav_msgs::msg::Path last_path_;
  bool has_path_ = false;
  
  // Linear regression result
  struct LinearFit {
    double slope;      // a in y = ax + b
    double intercept;  // b in y = ax + b
    double error;      // Mean squared error
  };
  
  // Perform linear regression on a segment of points
  LinearFit linearRegression(const std::vector<geometry_msgs::msg::PoseStamped>& points, 
                             size_t start_idx, size_t end_idx) {
    LinearFit result;
    
    if (end_idx <= start_idx || end_idx - start_idx < 2) {
      result.slope = 0.0;
      result.intercept = 0.0;
      result.error = 0.0;
      return result;
    }
    
    size_t n = end_idx - start_idx + 1;
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
    
    for (size_t i = start_idx; i <= end_idx; i++) {
      double x = points[i].pose.position.x;
      double y = points[i].pose.position.y;
      sum_x += x;
      sum_y += y;
      sum_xy += x * y;
      sum_x2 += x * x;
    }
    
    // Calculate slope and intercept: y = ax + b
    double denominator = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denominator) < 1e-9) {
      // Vertical line or single point
      result.slope = 0.0;
      result.intercept = sum_y / n;
    } else {
      result.slope = (n * sum_xy - sum_x * sum_y) / denominator;
      result.intercept = (sum_y - result.slope * sum_x) / n;
    }
    
    // Calculate mean squared error
    double total_error = 0.0;
    for (size_t i = start_idx; i <= end_idx; i++) {
      double x = points[i].pose.position.x;
      double y = points[i].pose.position.y;
      double predicted_y = result.slope * x + result.intercept;
      double error = y - predicted_y;
      total_error += error * error;
    }
    result.error = total_error / n;
    
    return result;
  }
  
  // Find the best split point for a segment
  size_t findBestSplitPoint(const std::vector<geometry_msgs::msg::PoseStamped>& points,
                             size_t start_idx, size_t end_idx) {
    if (end_idx <= start_idx + 1) {
      return end_idx;
    }
    
    size_t best_split = start_idx + 1;
    double min_total_error = std::numeric_limits<double>::max();
    
    // Try each possible split point
    for (size_t split = start_idx + 1; split < end_idx; split++) {
      LinearFit fit1 = linearRegression(points, start_idx, split);
      LinearFit fit2 = linearRegression(points, split, end_idx);
      
      // Weighted average error
      size_t n1 = split - start_idx + 1;
      size_t n2 = end_idx - split + 1;
      double total_error = (fit1.error * n1 + fit2.error * n2) / (n1 + n2);
      
      if (total_error < min_total_error) {
        min_total_error = total_error;
        best_split = split;
      }
    }
    
    return best_split;
  }
  
  // Recursively split path into linear segments (max 4 segments)
  void splitPathRecursive(const std::vector<geometry_msgs::msg::PoseStamped>& input,
                          size_t start_idx, size_t end_idx,
                          std::vector<size_t>& breakpoints,
                          int max_segments) {
    if (end_idx <= start_idx || max_segments <= 1) {
      return;
    }
    
    // Fit linear regression to current segment
    LinearFit fit = linearRegression(input, start_idx, end_idx);
    
    // Check if we need to split
    // Calculate maximum perpendicular distance from line
    double max_distance = 0.0;
    size_t max_dist_idx = start_idx;
    
    for (size_t i = start_idx + 1; i < end_idx; i++) {
      double x = input[i].pose.position.x;
      double y = input[i].pose.position.y;
      double predicted_y = fit.slope * x + fit.intercept;
      double distance = std::abs(y - predicted_y);
      
      if (distance > max_distance) {
        max_distance = distance;
        max_dist_idx = i;
      }
    }
    
    // If error is small enough or we've reached max segments, don't split
    if (max_distance < 0.1 || breakpoints.size() >= max_segments - 1) {
      return;
    }
    
    // Find best split point around the point with maximum error
    size_t search_start = std::max(start_idx + 1, max_dist_idx - 2);
    size_t search_end = std::min(end_idx - 1, max_dist_idx + 2);
    size_t best_split = findBestSplitPoint(input, start_idx, end_idx);
    
    // Add breakpoint if not already present
    if (std::find(breakpoints.begin(), breakpoints.end(), best_split) == breakpoints.end()) {
      breakpoints.push_back(best_split);
      std::sort(breakpoints.begin(), breakpoints.end());
    }
    
    // Recursively split both segments
    if (breakpoints.size() < max_segments - 1) {
      splitPathRecursive(input, start_idx, best_split, breakpoints, max_segments);
      splitPathRecursive(input, best_split, end_idx, breakpoints, max_segments);
    }
  }
  
  // Calculate distance between two points
  double calculateDistance(const geometry_msgs::msg::Point& p1, 
                          const geometry_msgs::msg::Point& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  
  // Interpolate points along a line segment with 5cm spacing
  // skip_start: if true, skip adding the start point (already in output)
  void interpolateSegment(const geometry_msgs::msg::PoseStamped& p1,
                          const geometry_msgs::msg::PoseStamped& p2,
                          std::vector<geometry_msgs::msg::PoseStamped>& output,
                          double spacing = 0.05,
                          bool skip_start = false) {
    double dx = p2.pose.position.x - p1.pose.position.x;
    double dy = p2.pose.position.y - p1.pose.position.y;
    double dz = p2.pose.position.z - p1.pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    if (distance < 1e-6) {
      // Points are the same
      if (!skip_start) {
        output.push_back(p1);
      }
      return;
    }
    
    // Calculate yaw for orientation
    double yaw = std::atan2(dy, dx);
    
    // Number of interpolated points needed (excluding start and end)
    int num_intermediate = static_cast<int>(std::floor(distance / spacing));
    
    // Add start point if not skipping
    if (!skip_start) {
      geometry_msgs::msg::PoseStamped start_pose = p1;
      start_pose.pose.orientation.w = std::cos(yaw / 2.0);
      start_pose.pose.orientation.z = std::sin(yaw / 2.0);
      output.push_back(start_pose);
    }
    
    // Interpolate intermediate points at 5cm intervals
    for (int i = 1; i <= num_intermediate; i++) {
      double t = static_cast<double>(i) * spacing / distance;
      if (t >= 1.0) {
        break;  // Don't add points beyond the end point
      }
      
      geometry_msgs::msg::PoseStamped interpolated;
      interpolated.header = p1.header;
      interpolated.pose.position.x = p1.pose.position.x + t * dx;
      interpolated.pose.position.y = p1.pose.position.y + t * dy;
      interpolated.pose.position.z = p1.pose.position.z + t * dz;
      interpolated.pose.orientation.w = std::cos(yaw / 2.0);
      interpolated.pose.orientation.z = std::sin(yaw / 2.0);
      
      output.push_back(interpolated);
    }
    
    // Add end point
    geometry_msgs::msg::PoseStamped end_pose = p2;
    end_pose.pose.orientation.w = std::cos(yaw / 2.0);
    end_pose.pose.orientation.z = std::sin(yaw / 2.0);
    output.push_back(end_pose);
  }
  
  // Convert path to linear segments using linear regression and interpolate with 5cm spacing
  void convertToLinearSegments(const std::vector<geometry_msgs::msg::PoseStamped>& input,
                               std::vector<geometry_msgs::msg::PoseStamped>& output) {
    if (input.empty()) {
      return;
    }
    
    if (input.size() == 1) {
      output = input;
      return;
    }
    
    // Preserve start and end points
    const auto& start_point = input[0];
    const auto& end_point = input.back();
    
    // Check if last point is (0,0) - indicates long-distance path
    bool is_long_distance = (std::abs(end_point.pose.position.x) < 1e-6 && 
                             std::abs(end_point.pose.position.y) < 1e-6);
    
    // Determine max segments: 10 for long-distance, 4 for normal
    int max_segments = is_long_distance ? 10 : 4;
    
    if (input.size() == 2) {
      // Simple case: just interpolate between start and end
      interpolateSegment(start_point, end_point, output, 0.05);
      // Ensure exact start and end points
      if (!output.empty()) {
        output[0] = start_point;
        output.back() = end_point;
      }
      return;
    }
    
    // If 4 points or less, skip linear regression and just interpolate between consecutive points
    if (input.size() <= 4) {
      for (size_t i = 0; i < input.size() - 1; i++) {
        bool skip_start = (i > 0);
        interpolateSegment(input[i], input[i + 1], output, 0.05, skip_start);
      }
      // Ensure exact start and end points
      if (!output.empty()) {
        output[0] = start_point;
        output.back() = end_point;
      }
      return;
    }
    
    // Find breakpoints (max_segments segments)
    std::vector<size_t> breakpoints;
    size_t start_idx = 0;
    size_t end_idx = input.size() - 1;
    
    splitPathRecursive(input, start_idx, end_idx, breakpoints, max_segments);
    
    // Ensure start and end are included
    if (breakpoints.empty() || breakpoints[0] != start_idx) {
      breakpoints.insert(breakpoints.begin(), start_idx);
    }
    if (breakpoints.empty() || breakpoints.back() != end_idx) {
      breakpoints.push_back(end_idx);
    }
    
    // Remove duplicates and sort
    std::sort(breakpoints.begin(), breakpoints.end());
    breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()), breakpoints.end());
    
    // Build output path with linear segments, interpolated at 5cm intervals
    for (size_t i = 0; i < breakpoints.size() - 1; i++) {
      size_t seg_start_idx = breakpoints[i];
      size_t seg_end_idx = breakpoints[i + 1];
      
      // Get start and end points of this segment
      const auto& p1 = input[seg_start_idx];
      const auto& p2 = input[seg_end_idx];
      
      // Interpolate this segment with 5cm spacing
      // Skip start point for subsequent segments (already added as end of previous segment)
      bool skip_start = (i > 0);
      interpolateSegment(p1, p2, output, 0.05, skip_start);
    }
    
    // Ensure first and last points exactly match original (preserve start and end)
    if (!output.empty() && !input.empty()) {
      output[0] = start_point;
      output.back() = end_point;
    }
    
    // Verify order: ensure all points are in forward direction (no backtracking)
    if (output.size() > 2) {
      std::vector<geometry_msgs::msg::PoseStamped> ordered_output;
      ordered_output.push_back(output[0]);
      
      for (size_t i = 1; i < output.size(); i++) {
        // Check direction consistency
        if (i > 1) {
          const auto& prev_prev = ordered_output[ordered_output.size() - 2];
          const auto& prev = ordered_output.back();
          const auto& curr = output[i];
          
          double dx1 = prev.pose.position.x - prev_prev.pose.position.x;
          double dy1 = prev.pose.position.y - prev_prev.pose.position.y;
          double dx2 = curr.pose.position.x - prev.pose.position.x;
          double dy2 = curr.pose.position.y - prev.pose.position.y;
          
          double dot_product = dx1 * dx2 + dy1 * dy2;
          
          // If direction reverses significantly, skip this point
          if (dot_product < -0.01) {
            continue;  // Skip backward point
          }
        }
        
        ordered_output.push_back(output[i]);
      }
      
      // Ensure last point is preserved
      if (!ordered_output.empty() && !output.empty()) {
        ordered_output.back() = end_point;
      }
      
      output = ordered_output;
    }
  }
  
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      return;
    }
    
    // Check if long-distance path (last point is (0,0))
    const auto& end_point = msg->poses.back();
    bool is_long_distance = (std::abs(end_point.pose.position.x) < 1e-6 && 
                             std::abs(end_point.pose.position.y) < 1e-6);
    
    // Convert path to linear segments using linear regression
    std::vector<geometry_msgs::msg::PoseStamped> linear_path;
    convertToLinearSegments(msg->poses, linear_path);
    
    // Build output path
    nav_msgs::msg::Path output_path;
    output_path.header = msg->header;
    output_path.poses = linear_path;
    
    last_path_ = output_path;
    has_path_ = true;
    
    pub_path_->publish(last_path_);
  }

  void publishCachedPath() {
    if (!has_path_) {
      return;
    }
    last_path_.header.stamp = this->now();
    for (auto &pose : last_path_.poses) {
      pose.header.stamp = last_path_.header.stamp;
    }
    pub_path_->publish(last_path_);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AosPathLinearizationNode>());
  rclcpp::shutdown();
  return 0;
}

