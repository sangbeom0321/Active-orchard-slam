/**
 * @file tree_clusterer.hpp
 * @brief Tree clustering and row detection for orchard environments
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This class implements tree detection and clustering from LiDAR point clouds,
 * followed by row structure extraction for systematic orchard navigation.
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>
#include <memory>

namespace orbit_planner {

struct TreeCluster {
    geometry_msgs::msg::Point center;
    std::vector<geometry_msgs::msg::Point> points;
    double radius;
    double height;
    int point_count;
};

struct TreeRow {
    std::vector<TreeCluster> trees;
    geometry_msgs::msg::Point start_point;
    geometry_msgs::msg::Point end_point;
    double orientation; // in radians
    double length;
    double width;
    int tree_count;
};

class TreeClusterer {
public:
    TreeClusterer();
    ~TreeClusterer() = default;

    // Parameter setting
    void setParameters(double min_height, double max_height, 
                      double cluster_tolerance, int min_cluster_size,
                      double voxel_size, double row_tolerance,
                      double min_row_length, int min_trees_per_row,
                      double center_search_radius, double clustering_radius,
                      int min_neighbors_in_radius);

    // Main processing functions
    std::vector<TreeCluster> detectTrees(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
    
    std::vector<TreeRow> detectRows(
        const std::vector<TreeCluster>& tree_clusters);
    
    nav_msgs::msg::OccupancyGrid createOccupancyGrid(
        const std::vector<TreeCluster>& tree_clusters,
        const geometry_msgs::msg::Point& origin,
        double resolution = 0.1,
        int width = 1000,
        int height = 1000);

    // Getters
    const std::vector<TreeCluster>& getLastTreeClusters() const { return last_tree_clusters_; }
    const std::vector<TreeRow>& getLastTreeRows() const { return last_tree_rows_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredHeightBandCloud() const;

private:
    // Parameters
    double min_height_;
    double max_height_;
    double cluster_tolerance_;
    int min_cluster_size_;
    double voxel_size_;
    double row_tolerance_;
    double min_row_length_;
    int min_trees_per_row_;
    
    // Radius search parameters (from clustering_node.cpp)
    double center_search_radius_;
    double clustering_radius_;
    int min_neighbors_in_radius_;

    // PCL components
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor_;
    pcl::SACSegmentation<pcl::PointXYZ> line_segmenter_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;

    // Results
    std::vector<TreeCluster> last_tree_clusters_;
    std::vector<TreeRow> last_tree_rows_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_filtered_cloud_;

    // Internal processing functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterHeightBand(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
    
    std::vector<TreeCluster> clusterTrees(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud);
    
    // Radius search based clustering (from clustering_node.cpp)
    pcl::PointCloud<pcl::PointXYZ>::Ptr kdtreeSearchRadius(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr findPointCloudCenter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    TreeRow fitLineToClusters(const std::vector<TreeCluster>& clusters);
    
    bool isValidRow(const TreeRow& row);
    
    double calculateOrientation(const std::vector<TreeCluster>& clusters);
    
    geometry_msgs::msg::Point calculateCentroid(
        const std::vector<TreeCluster>& clusters);
    
    double calculateRowOrientation(const TreeRow& row) const;
    geometry_msgs::msg::Point calculateRowStart(const TreeRow& row) const;
    geometry_msgs::msg::Point calculateRowEnd(const TreeRow& row) const;
    
    // Utility functions
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
    
    double calculateAngle(const geometry_msgs::msg::Point& p1, 
                         const geometry_msgs::msg::Point& p2);
    
    bool isPointInHeightBand(const pcl::PointXYZ& point);
};

} // namespace orbit_planner
