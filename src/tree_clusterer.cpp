#include "aos/tree_clusterer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_set>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

namespace orbit_planner {

TreeClusterer::TreeClusterer() 
    : min_height_(1.0), max_height_(2.0), cluster_tolerance_(0.5),
      min_cluster_size_(10), voxel_size_(0.1), row_tolerance_(2.0),
      min_row_length_(5.0), min_trees_per_row_(3),
      center_search_radius_(0.15), clustering_radius_(0.8), min_neighbors_in_radius_(0) {
    kdtree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    last_filtered_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void TreeClusterer::setParameters(double min_height, double max_height, 
                                 double cluster_tolerance, int min_cluster_size,
                                 double voxel_size, double row_tolerance,
                                 double min_row_length, int min_trees_per_row,
                                 double center_search_radius, double clustering_radius,
                                 int min_neighbors_in_radius) {
    min_height_ = min_height;
    max_height_ = max_height;
    cluster_tolerance_ = cluster_tolerance;
    min_cluster_size_ = min_cluster_size;
    voxel_size_ = voxel_size;
    row_tolerance_ = row_tolerance;
    min_row_length_ = min_row_length;
    min_trees_per_row_ = min_trees_per_row;
    center_search_radius_ = center_search_radius;
    clustering_radius_ = clustering_radius;
    min_neighbors_in_radius_ = min_neighbors_in_radius;
}

std::vector<TreeCluster> TreeClusterer::detectTrees(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    
    if (!input_cloud || input_cloud->empty()) {
        return {};
    }
    
    // Filter height band
    auto filtered_cloud = filterHeightBand(input_cloud);
    
    // Cluster trees
    auto clusters = clusterTrees(filtered_cloud);
    
    last_tree_clusters_ = clusters;
    return clusters;
}

std::vector<TreeRow> TreeClusterer::detectRows(
    const std::vector<TreeCluster>& tree_clusters) {
    // 참고: row_detection_node.cpp의 방식과 유사하게
    // 1) 클러스터 중심점으로 임시 포인트클라우드 구성
    // 2) 중심점들에 대해 유클리드 클러스터링 수행
    // 3) 각 클러스터에 PCA를 적용하여 주성분 방향(eigenvector)과 대표점 선정
    // 4) 방향 유사도와 점-선 거리 기준으로 유니온파인드로 라인 그룹 병합
    // 5) 병합된 각 그룹에서 주성분축을 다시 계산하여 라인 양 끝점을 구해 TreeRow 생성

    std::vector<TreeRow> rows;
    if (tree_clusters.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("tree_clusterer"), "detectRows: no tree clusters provided");
        last_tree_rows_ = rows;
        return rows;
    }

    // 파라미터 매핑
    const double orientation_cos_threshold = 0.9; // row_detection_node.cpp의 기본값과 동일
    const double point_to_line_dist_threshold = std::max(0.5, row_tolerance_); // 거리 기준: row_tolerance_ 활용

    // 1) 중심점 포인트클라우드 구성
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "detectRows: input clusters=%zu, min_trees_per_row=%d, row_tol=%.3f",
                tree_clusters.size(), min_trees_per_row_, row_tolerance_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    centers_cloud->points.reserve(tree_clusters.size());
    for (const auto &tc : tree_clusters) {
        centers_cloud->points.emplace_back(tc.center.x, tc.center.y, tc.center.z);
    }
    centers_cloud->width = centers_cloud->points.size();
    centers_cloud->height = 1;
    centers_cloud->is_dense = true;

    // 2) 유클리드 클러스터링 (중심점들을 묶어 초기 라인 후보 생성)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(centers_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(row_tolerance_);
    ec.setMinClusterSize(std::max(2, min_trees_per_row_));
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(centers_cloud);
    ec.extract(cluster_indices);
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "detectRows: initial euclidean clusters=%zu", cluster_indices.size());

    if (cluster_indices.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("tree_clusterer"), "detectRows: no initial clusters from centers");
        last_tree_rows_ = rows;
        return rows;
    }

    struct LinearCurve {
        Eigen::Vector2f point; // 대표점
        Eigen::Vector2f dir;   // 주성분 방향(정규화)
        std::vector<int> member_indices; // centers_cloud 인덱스들
    };

    std::vector<LinearCurve> curves;
    curves.reserve(cluster_indices.size());

    auto compute_pca_dir = [&](const std::vector<int> &indices) -> Eigen::Vector2f {
        // 2D 공분산 기반 주성분축 계산
        if (indices.size() < 2) {
            return Eigen::Vector2f(1.f, 0.f);
        }
        Eigen::Vector2f mean(0.f, 0.f);
        for (int idx : indices) {
            const auto &p = centers_cloud->points[idx];
            mean += Eigen::Vector2f(p.x, p.y);
        }
        mean /= static_cast<float>(indices.size());
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (int idx : indices) {
            const auto &p = centers_cloud->points[idx];
            Eigen::Vector2f d(p.x - mean.x(), p.y - mean.y());
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(indices.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
        if (solver.info() != Eigen::Success) {
            return Eigen::Vector2f(1.f, 0.f);
        }
        // 최대 고유값에 해당하는 고유벡터가 마지막 열
        Eigen::Vector2f dir = solver.eigenvectors().col(1);
        if (dir.x() < 0.f) dir = -dir; // 방향 일관성
        return dir.normalized();
    };

    // 3) 각 초기 클러스터에 대해 LinearCurve 생성 (대표점은 두 번째 포인트가 있으면 선택)
    for (const auto &ci : cluster_indices) {
        if (ci.indices.empty()) continue;
        LinearCurve lc;
        lc.member_indices = ci.indices;
        Eigen::Vector2f dir = compute_pca_dir(ci.indices);
        lc.dir = dir;
        int rep_idx = (ci.indices.size() >= 2) ? ci.indices[1] : ci.indices[0];
        const auto &rep = centers_cloud->points[rep_idx];
        lc.point = Eigen::Vector2f(rep.x, rep.y);
        curves.push_back(std::move(lc));
    }

    if (curves.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("tree_clusterer"), "detectRows: no curves after PCA stage");
        last_tree_rows_ = rows;
        return rows;
    }
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "detectRows: curves built=%zu", curves.size());

    // 4) 유니온파인드로 라인 병합
    std::vector<int> parent(curves.size());
    for (size_t i = 0; i < curves.size(); ++i) parent[i] = static_cast<int>(i);
    std::function<int(int)> find_set = [&](int x) {
        return parent[x] == x ? x : parent[x] = find_set(parent[x]);
    };
    auto unite = [&](int a, int b) {
        a = find_set(a); b = find_set(b);
        if (a != b) parent[b] = a;
    };
    auto point_to_line_distance = [&](const Eigen::Vector2f &p, const Eigen::Vector2f &a, const Eigen::Vector2f &dir) -> float {
        // dist = || (a->p) x dir || / ||dir||, 여기서 2D는 수직벡터 이용
        Eigen::Vector2f ap = p - a;
        Eigen::Vector2f n(-dir.y(), dir.x());
        return std::abs(ap.dot(n));
    };

    size_t merge_count = 0;
    for (size_t i = 0; i < curves.size(); ++i) {
        for (size_t j = i + 1; j < curves.size(); ++j) {
            const auto &A = curves[i];
            const auto &B = curves[j];
            float cos_theta = std::abs(A.dir.normalized().dot(B.dir.normalized()));
            if (cos_theta > static_cast<float>(orientation_cos_threshold)) {
                float d = point_to_line_distance(B.point, A.point, A.dir);
                if (d < static_cast<float>(point_to_line_dist_threshold)) {
                    unite(static_cast<int>(i), static_cast<int>(j));
                    merge_count++;
                }
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "detectRows: merges=%zu (cos>%.2f, d<th=%.2f)",
                merge_count, orientation_cos_threshold, point_to_line_dist_threshold);

    // 그룹화: set_id -> 모든 멤버 인덱스 집계
    std::unordered_map<int, std::vector<int>> group_to_curve_indices;
    for (size_t i = 0; i < curves.size(); ++i) {
        int set_id = find_set(static_cast<int>(i));
        group_to_curve_indices[set_id].push_back(static_cast<int>(i));
    }
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "detectRows: groups=%zu", group_to_curve_indices.size());

    // 5) 각 그룹에서 TreeRow 생성
    for (const auto &kv : group_to_curve_indices) {
        std::vector<int> member_point_indices;
        for (int curve_idx : kv.second) {
            const auto &lc = curves[curve_idx];
            member_point_indices.insert(member_point_indices.end(), lc.member_indices.begin(), lc.member_indices.end());
        }
        if (member_point_indices.size() < static_cast<size_t>(min_trees_per_row_)) {
            RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"),
                        "detectRows: skip group set_id=%d due to small size (%zu < %d)",
                        kv.first, member_point_indices.size(), min_trees_per_row_);
            continue;
        }

        // 중복 제거
        std::sort(member_point_indices.begin(), member_point_indices.end());
        member_point_indices.erase(std::unique(member_point_indices.begin(), member_point_indices.end()), member_point_indices.end());

        // 주성분축과 끝점 계산 (3D로 z는 그대로 유지하되, 주성분은 xy에서 계산)
        Eigen::Vector3f centroid3(0.f, 0.f, 0.f);
        for (int idx : member_point_indices) {
            const auto &p = centers_cloud->points[idx];
            centroid3 += Eigen::Vector3f(p.x, p.y, p.z);
        }
        centroid3 /= static_cast<float>(member_point_indices.size());

        Eigen::Matrix3f cov3 = Eigen::Matrix3f::Zero();
        for (int idx : member_point_indices) {
            const auto &p = centers_cloud->points[idx];
            Eigen::Vector3f d(p.x - centroid3.x(), p.y - centroid3.y(), p.z - centroid3.z());
            cov3 += d * d.transpose();
        }
        cov3 /= static_cast<float>(member_point_indices.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver3(cov3);
        if (solver3.info() != Eigen::Success) {
            RCLCPP_WARN(rclcpp::get_logger("tree_clusterer"),
                        "detectRows: eigen decomposition failed for group set_id=%d", kv.first);
            continue;
        }
        Eigen::Vector3f principal_axis = solver3.eigenvectors().col(2);

        float min_proj = std::numeric_limits<float>::max();
        float max_proj = std::numeric_limits<float>::lowest();
        for (int idx : member_point_indices) {
            const auto &p = centers_cloud->points[idx];
            Eigen::Vector3f v(p.x, p.y, p.z);
            float proj = (v - centroid3).dot(principal_axis);
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
        }

        Eigen::Vector3f endpoint_min = centroid3 + principal_axis * min_proj;
        Eigen::Vector3f endpoint_max = centroid3 + principal_axis * max_proj;

        TreeRow row;
        row.tree_count = static_cast<int>(member_point_indices.size());
        row.length = std::sqrt(std::pow(endpoint_max.x() - endpoint_min.x(), 2) +
                               std::pow(endpoint_max.y() - endpoint_min.y(), 2) +
                               std::pow(endpoint_max.z() - endpoint_min.z(), 2));
        row.width = 2.0; // 기본값 유지
        row.orientation = std::atan2(principal_axis.y(), principal_axis.x());
        row.start_point.x = endpoint_min.x();
        row.start_point.y = endpoint_min.y();
        row.start_point.z = endpoint_min.z();
        row.end_point.x = endpoint_max.x();
        row.end_point.y = endpoint_max.y();
        row.end_point.z = endpoint_max.z();

        // TreeCluster 목록 구성
        row.trees.reserve(member_point_indices.size());
        for (int idx : member_point_indices) {
            row.trees.push_back(tree_clusters[static_cast<size_t>(idx)]);
        }

        if (isValidRow(row)) {
            rows.push_back(std::move(row));
        } else {
            RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"),
                        "detectRows: group set_id=%d failed validity (count=%d, length=%.2f, min_len=%.2f)",
                        kv.first, row.tree_count, row.length, min_row_length_);
        }
    }

    last_tree_rows_ = rows;
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "detectRows: result rows=%zu", rows.size());
    return rows;
}

nav_msgs::msg::OccupancyGrid TreeClusterer::createOccupancyGrid(
    const std::vector<TreeCluster>& tree_clusters,
    const geometry_msgs::msg::Point& origin,
    double resolution,
    int width,
    int height) {
    
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = rclcpp::Clock().now();
    
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position = origin;
    grid.info.origin.orientation.w = 1.0;
    
    grid.data.resize(width * height, 0);
    
    // Mark tree positions as occupied
    for (const auto& cluster : tree_clusters) {
        int x = static_cast<int>((cluster.center.x - origin.x) / resolution);
        int y = static_cast<int>((cluster.center.y - origin.y) / resolution);
        
        if (x >= 0 && x < width && y >= 0 && y < height) {
            // Mark tree and surrounding area as occupied
            for (int dx = -2; dx <= 2; ++dx) {
                for (int dy = -2; dy <= 2; ++dy) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        grid.data[ny * width + nx] = 100;
                    }
                }
            }
        }
    }
    
    return grid;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TreeClusterer::getFilteredHeightBandCloud() const {
    return last_filtered_cloud_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TreeClusterer::filterHeightBand(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "Filtering height band: input %zu points, height range [%.2f, %.2f]", 
                input_cloud->size(), min_height_, max_height_);
    
    for (const auto& point : input_cloud->points) {
        if (isPointInHeightBand(point)) {
            filtered_cloud->points.push_back(point);
        }
    }
    
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "Height filtering result: %zu points", filtered_cloud->size());
    
    // Store filtered cloud for visualization
    last_filtered_cloud_ = filtered_cloud;
    
    return filtered_cloud;
}

std::vector<TreeCluster> TreeClusterer::clusterTrees(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {
    
    std::vector<TreeCluster> clusters;
    
    if (filtered_cloud->empty()) {
        RCLCPP_WARN(rclcpp::get_logger("tree_clusterer"), "clusterTrees: empty input cloud");
        return clusters;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "clusterTrees: input points=%zu, cluster_tolerance=%.3f, min_size=%d", 
                filtered_cloud->size(), cluster_tolerance_, min_cluster_size_);
    
    // Step 1: Radius search filtering (like clustering_node.cpp)
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_filtered_cloud = kdtreeSearchRadius(filtered_cloud);
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "clusterTrees: after radius filtering=%zu points", radius_filtered_cloud->size());
    
    if (radius_filtered_cloud->empty()) {
        RCLCPP_WARN(rclcpp::get_logger("tree_clusterer"), "clusterTrees: no points after radius filtering");
        return clusters;
    }
    
    // Step 2: Find tree centers using radius search (like clustering_node.cpp)
    pcl::PointCloud<pcl::PointXYZ>::Ptr tree_centers = findPointCloudCenter(radius_filtered_cloud);
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "clusterTrees: found %zu tree centers", tree_centers->size());
    
    // Step 3: Create TreeCluster objects from centers
    for (const auto& center : tree_centers->points) {
        TreeCluster cluster;
        cluster.center.x = center.x;
        cluster.center.y = center.y;
        cluster.center.z = center.z;
        
        // Find points within cluster_tolerance_ of this center
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(radius_filtered_cloud);
        
        std::vector<int> point_indices;
        std::vector<float> point_squared_distance;
        
        if (kdtree.radiusSearch(center, cluster_tolerance_, point_indices, point_squared_distance) > 0) {
            cluster.point_count = point_indices.size();
            
            // Calculate radius and height from nearby points
            double max_radius = 0;
            double min_z = std::numeric_limits<double>::max();
            double max_z = std::numeric_limits<double>::lowest();
            
            for (int idx : point_indices) {
                const auto& point = radius_filtered_cloud->points[idx];
                double dist = std::sqrt(std::pow(point.x - cluster.center.x, 2) + 
                                      std::pow(point.y - cluster.center.y, 2));
                max_radius = std::max(max_radius, dist);
                min_z = std::min(min_z, static_cast<double>(point.z));
                max_z = std::max(max_z, static_cast<double>(point.z));
            }
            
            cluster.radius = max_radius;
            cluster.height = max_z - min_z;
            
            // Only add if meets minimum size requirement
            if (cluster.point_count >= min_cluster_size_) {
                clusters.push_back(cluster);
            }
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("tree_clusterer"), "clusterTrees: final clusters=%zu", clusters.size());
    return clusters;
}

bool TreeClusterer::isValidRow(const TreeRow& row) {
    return row.tree_count >= min_trees_per_row_ && row.length >= min_row_length_;
}

double TreeClusterer::calculateRowOrientation(const TreeRow& row) const {
    if (row.trees.size() < 2) return 0.0;
    
    // Calculate orientation using first and last trees
    const auto& first = row.trees.front().center;
    const auto& last = row.trees.back().center;
    
    return std::atan2(last.y - first.y, last.x - first.x);
}

geometry_msgs::msg::Point TreeClusterer::calculateRowStart(const TreeRow& row) const {
    if (row.trees.empty()) return {};
    
    geometry_msgs::msg::Point start = row.trees[0].center;
    for (const auto& tree : row.trees) {
        if (tree.center.x < start.x) {
            start = tree.center;
        }
    }
    return start;
}

geometry_msgs::msg::Point TreeClusterer::calculateRowEnd(const TreeRow& row) const {
    if (row.trees.empty()) return {};
    
    geometry_msgs::msg::Point end = row.trees[0].center;
    for (const auto& tree : row.trees) {
        if (tree.center.x > end.x) {
            end = tree.center;
        }
    }
    return end;
}

double TreeClusterer::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                       const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double TreeClusterer::calculateAngle(const geometry_msgs::msg::Point& p1, 
                                    const geometry_msgs::msg::Point& p2) {
    return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

bool TreeClusterer::isPointInHeightBand(const pcl::PointXYZ& point) {
    return point.z >= min_height_ && point.z <= max_height_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TreeClusterer::kdtreeSearchRadius(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    
    // k-d 트리 객체를 생성합니다.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 필터링된 인덱스를 저장할 객체입니다.
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // 모든 포인트에 대해 주변 이웃을 검사합니다.
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // 현재 포인트 주변의 이웃을 탐색합니다.
        if (kdtree.radiusSearch(cloud->points[i], clustering_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > min_neighbors_in_radius_) {
            // 최소 이웃 수 이상이면 결과에 포함시킵니다.
            inliers->indices.push_back(i);
        }
    }
    
    // 필터링된 포인트 클라우드를 추출합니다.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*filteredCloud);

    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TreeClusterer::findPointCloudCenter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    
    // k-d 트리 객체를 생성합니다.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 필터링된 인덱스를 저장할 객체입니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr central_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::unordered_set<int> processed_indices;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        // 이미 처리된 점은 건너뜁니다.
        if (processed_indices.find(i) != processed_indices.end()) {
            continue;
        }

        std::vector<int> point_indices;
        std::vector<float> point_squared_distance;

        // 반지름 내의 모든 점 검색
        if (kdtree.radiusSearch(cloud->points[i], center_search_radius_, point_indices, point_squared_distance) > 0) {
            float sum_x = 0, sum_y = 0, sum_z = 0;
            int count = 0;

            // 원점으로부터 가장 가까운 점 찾기
            for (int idx : point_indices) {
                if (processed_indices.find(idx) != processed_indices.end()) {
                    continue; // 이미 처리된 점은 건너뜁니다.
                }
                sum_x += cloud->points[idx].x;
                sum_y += cloud->points[idx].y;
                sum_z += cloud->points[idx].z;
                count++;
                processed_indices.insert(idx);
            }
            if (count > 0) {
                pcl::PointXYZ central_point(sum_x / count, sum_y / count, sum_z / count);
                central_points->points.push_back(central_point);
            }
        }
    }
    
    central_points->width = central_points->points.size();
    central_points->height = 1;
    central_points->is_dense = true;
    
    return central_points;
}

} // namespace orbit_planner
