#include "aos/voronoi_diagram.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <unordered_map>
#include <set>

namespace aos {

VoronoiDiagram::VoronoiDiagram() {}

VoronoiDiagram::~VoronoiDiagram() {}

void VoronoiDiagram::compute(const std::vector<Eigen::Vector2d>& seeds,
                            double min_x, double max_x,
                            double min_y, double max_y) {
  seeds_ = seeds;
  edges_.clear();
  vertices_.clear();

  if (seeds.empty()) {
    return;
  }

  // Validate boundary values
  if (!std::isfinite(min_x) || !std::isfinite(max_x) || !std::isfinite(min_y) || !std::isfinite(max_y)) {
    return;
  }

  // Normalize boundary values (ensure min < max)
  if (min_x > max_x) std::swap(min_x, max_x);
  if (min_y > max_y) std::swap(min_y, max_y);

  // Ensure minimum size (expand if too small)
  const double min_size = 1.0;
  if (max_x - min_x < min_size) {
    double center_x = (min_x + max_x) / 2.0;
    min_x = center_x - min_size / 2.0;
    max_x = center_x + min_size / 2.0;
  }
  if (max_y - min_y < min_size) {
    double center_y = (min_y + max_y) / 2.0;
    min_y = center_y - min_size / 2.0;
    max_y = center_y + min_size / 2.0;
  }

  // Compute Voronoi diagram using OpenCV Subdiv2D (reference code approach)
  // Add 1 to each side for sufficient computation area
  cv::Rect2f bounding_rect(
    static_cast<float>(min_x - 1.0),
    static_cast<float>(min_y - 1.0),
    static_cast<float>(std::abs(max_x - min_x) + 2.0),
    static_cast<float>(std::abs(max_y - min_y) + 2.0)
  );

  // Validate bounding rectangle
  if (bounding_rect.width <= 0 || bounding_rect.height <= 0) {
    return;
  }

  cv::Subdiv2D subdiv(bounding_rect);

  // Add seeds to Subdiv2D (with validation)
  for (const auto& seed : seeds_) {
    // Check for NaN/Inf values
    if (!std::isfinite(seed.x()) || !std::isfinite(seed.y())) {
      continue;
    }

    // Check if seed is inside bounding rectangle and clip
    float x = static_cast<float>(seed.x());
    float y = static_cast<float>(seed.y());
    
    // Clip to bounding rectangle boundary (with some margin)
    const float margin = 0.1f;
    x = std::max(bounding_rect.x + margin, std::min(bounding_rect.x + bounding_rect.width - margin, x));
    y = std::max(bounding_rect.y + margin, std::min(bounding_rect.y + bounding_rect.height - margin, y));

    cv::Point2f pt(x, y);
    
    try {
      subdiv.insert(pt);
    } catch (const cv::Exception& e) {
      // Ignore individual seed insertion failures and continue
      continue;
    }
  }

  // Extract Voronoi facet list
  std::vector<std::vector<cv::Point2f>> facets;
  std::vector<cv::Point2f> centers;
  subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);

  // Convert facets to edges
  for (const auto& facet : facets) {
    if (facet.size() < 2) {
      continue;
    }

    // Add adjacent points of each facet as edges
    for (size_t i = 0; i < facet.size(); ++i) {
      size_t next_i = (i + 1) % facet.size();
      
      VoronoiEdge edge;
      edge.start = Eigen::Vector2d(facet[i].x, facet[i].y);
      edge.end = Eigen::Vector2d(facet[next_i].x, facet[next_i].y);
      edge.seed1 = -1;  // Subdiv2D does not directly provide seed indices
      edge.seed2 = -1;
      
      edges_.push_back(edge);
    }
  }

  // Generate vertices (endpoints of edges)
  const double threshold = 0.01;  // 1cm threshold
  for (const auto& edge : edges_) {
    // Add start point
    bool start_found = false;
    for (auto& v : vertices_) {
      if ((v.position - edge.start).norm() < threshold) {
        start_found = true;
        break;
      }
    }
    if (!start_found) {
      VoronoiVertex v;
      v.position = edge.start;
      vertices_.push_back(v);
    }

    // Add end point
    bool end_found = false;
    for (auto& v : vertices_) {
      if ((v.position - edge.end).norm() < threshold) {
        end_found = true;
        break;
      }
    }
    if (!end_found) {
      VoronoiVertex v;
      v.position = edge.end;
      vertices_.push_back(v);
    }
  }
}

std::vector<Eigen::Vector2d> VoronoiDiagram::extractBoundaryPoints() const {
  std::vector<Eigen::Vector2d> boundary_points;

  // Set for duplicate prevention (integer-based coordinate hash)
  std::set<std::pair<int, int>> unique_points;
  const double threshold = 0.05;  // Changed to 5cm threshold (more precise for edge connection)

  for (const auto& edge : edges_) {
    // Add start point
    int ix_start = static_cast<int>(edge.start.x() * 100);
    int iy_start = static_cast<int>(edge.start.y() * 100);
    std::pair<int, int> key_start = {ix_start, iy_start};

    if (unique_points.find(key_start) == unique_points.end()) {
      // Check for duplicates based on distance
      bool too_close = false;
      for (const auto& existing : boundary_points) {
        double dx = existing.x() - edge.start.x();
        double dy = existing.y() - edge.start.y();
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < threshold * threshold) {
          too_close = true;
          break;
        }
      }

      if (!too_close) {
        unique_points.insert(key_start);
        boundary_points.push_back(edge.start);
      }
    }

    // Add end point
    int ix_end = static_cast<int>(edge.end.x() * 100);
    int iy_end = static_cast<int>(edge.end.y() * 100);
    std::pair<int, int> key_end = {ix_end, iy_end};

    if (unique_points.find(key_end) == unique_points.end()) {
      // Check for duplicates based on distance
      bool too_close = false;
      for (const auto& existing : boundary_points) {
        double dx = existing.x() - edge.end.x();
        double dy = existing.y() - edge.end.y();
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < threshold * threshold) {
          too_close = true;
          break;
        }
      }

      if (!too_close) {
        unique_points.insert(key_end);
        boundary_points.push_back(edge.end);
      }
    }
  }

  return boundary_points;
}

std::vector<std::vector<Eigen::Vector2d>> VoronoiDiagram::extractCellBoundaries() const {
  std::vector<std::vector<Eigen::Vector2d>> cell_boundaries;
  
  if (seeds_.empty()) {
    return cell_boundaries;
  }

  // Recompute OpenCV Subdiv2D to extract cell boundaries
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& seed : seeds_) {
    if (std::isfinite(seed.x()) && std::isfinite(seed.y())) {
      min_x = std::min(min_x, seed.x());
      max_x = std::max(max_x, seed.x());
      min_y = std::min(min_y, seed.y());
      max_y = std::max(max_y, seed.y());
    }
  }

  // Case when no valid seeds
  if (min_x > max_x || min_y > max_y) {
    return cell_boundaries;
  }

  // Ensure minimum size
  const double min_size = 1.0;
  if (max_x - min_x < min_size) {
    double center_x = (min_x + max_x) / 2.0;
    min_x = center_x - min_size / 2.0;
    max_x = center_x + min_size / 2.0;
  }
  if (max_y - min_y < min_size) {
    double center_y = (min_y + max_y) / 2.0;
    min_y = center_y - min_size / 2.0;
    max_y = center_y + min_size / 2.0;
  }

  cv::Rect2f bounding_rect(
    static_cast<float>(min_x - 1.0),
    static_cast<float>(min_y - 1.0),
    static_cast<float>(std::abs(max_x - min_x) + 2.0),
    static_cast<float>(std::abs(max_y - min_y) + 2.0)
  );

  // Validate bounding rectangle
  if (bounding_rect.width <= 0 || bounding_rect.height <= 0) {
    return cell_boundaries;
  }

  cv::Subdiv2D subdiv(bounding_rect);

  for (const auto& seed : seeds_) {
    // Check for NaN/Inf values
    if (!std::isfinite(seed.x()) || !std::isfinite(seed.y())) {
      continue;
    }

    // Check if seed is inside bounding rectangle and clip
    float x = static_cast<float>(seed.x());
    float y = static_cast<float>(seed.y());
    
    // Clip to bounding rectangle boundary (with some margin)
    const float margin = 0.1f;
    x = std::max(bounding_rect.x + margin, std::min(bounding_rect.x + bounding_rect.width - margin, x));
    y = std::max(bounding_rect.y + margin, std::min(bounding_rect.y + bounding_rect.height - margin, y));

    cv::Point2f pt(x, y);
    
    try {
      subdiv.insert(pt);
    } catch (const cv::Exception& e) {
      // Ignore individual seed insertion failures and continue
      continue;
    }
  }

  std::vector<std::vector<cv::Point2f>> facets;
  std::vector<cv::Point2f> centers;
  subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);

  // Extract boundary points of cell for each seed
  for (size_t seed_idx = 0; seed_idx < seeds_.size() && seed_idx < facets.size(); ++seed_idx) {
    std::vector<Eigen::Vector2d> cell_points;
    
    const auto& facet = facets[seed_idx];
    for (const auto& pt : facet) {
      cell_points.push_back(Eigen::Vector2d(pt.x, pt.y));
    }

    if (cell_points.size() >= 3) {
      // Add first point at the end to close polygon
      if ((cell_points.front() - cell_points.back()).norm() > 0.01) {
        cell_points.push_back(cell_points.front());
      }
      cell_boundaries.push_back(cell_points);
    }
  }

  return cell_boundaries;
}

}  // namespace aos
