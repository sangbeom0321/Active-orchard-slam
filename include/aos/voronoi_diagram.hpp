#pragma once

#include <vector>
#include <Eigen/Dense>
#include <memory>

namespace aos {

struct VoronoiEdge {
  Eigen::Vector2d start;   // Edge start point
  Eigen::Vector2d end;     // Edge end point
  int seed1;                // First seed index
  int seed2;                // Second seed index (-1 for boundary edges)
};

struct VoronoiVertex {
  Eigen::Vector2d position; // Vertex position
  std::vector<int> edge_indices; // Connected edge indices
};

class VoronoiDiagram {
public:
  VoronoiDiagram();
  ~VoronoiDiagram();

  // Compute Voronoi diagram (recomputed for all seeds each time)
  void compute(const std::vector<Eigen::Vector2d>& seeds,
               double min_x, double max_x,
               double min_y, double max_y);

  // Access results
  const std::vector<VoronoiEdge>& getEdges() const { return edges_; }
  const std::vector<VoronoiVertex>& getVertices() const { return vertices_; }
  const std::vector<Eigen::Vector2d>& getSeeds() const { return seeds_; }

  // Extract Voronoi boundary points (points to be used as GVD nodes)
  std::vector<Eigen::Vector2d> extractBoundaryPoints() const;

  // Extract boundary points of each Voronoi cell (for visualization)
  std::vector<std::vector<Eigen::Vector2d>> extractCellBoundaries() const;

private:
  std::vector<Eigen::Vector2d> seeds_;
  std::vector<VoronoiEdge> edges_;
  std::vector<VoronoiVertex> vertices_;
};

}  // namespace aos

