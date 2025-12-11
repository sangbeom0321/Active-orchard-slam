# GVD Graph Message
# Represents the Generalized Voronoi Diagram graph for path planning

std_msgs/Header header

# Map information
float64 resolution
float64 origin_x
float64 origin_y

# Number of nodes in the graph
int32 num_nodes

# Number of edges in the graph
int32 num_edges

# Graph nodes (world coordinates in meters)
geometry_msgs/Point[] nodes

# Node labels: 0=normal, 1=TL (Top Left), 2=TR (Top Right), 3=BL (Bottom Left), 4=BR (Bottom Right)
# Aligned with nodes array (same length as num_nodes)
# 0: 일반 보로노이 경계점
# 1: endpoint1의 -90도 방향 보로노이 경계점 (TL)
# 2: endpoint1의 +90도 방향 보로노이 경계점 (TR)
# 3: endpoint2의 -90도 방향 보로노이 경계점 (BL)
# 4: endpoint2의 +90도 방향 보로노이 경계점 (BR)
# 비트마스크로 여러 라벨 조합 가능 (1=TL, 2=TR, 4=BL, 8=BR)
int32[] node_labels

# Cluster indices: -1 if not associated with any cluster, otherwise the cluster index (0-based)
# Aligned with nodes array (same length as num_nodes)
# 클러스터는 computeClusterInfo()에서 정렬된 순서로 저장됨
# (deprecated: 첫 번째 매칭된 클러스터만 저장, node_label_clusters 사용 권장)
int32[] node_cluster_indices

# Node label cluster information: 각 노드의 라벨이 어떤 클러스터에서 나왔는지 저장
# Aligned with nodes array (same length as num_nodes)
# 각 노드에 대해 클러스터 ID와 라벨 타입의 쌍을 저장
# label_type: 0=TL, 1=TR, 2=BL, 3=BR
# 구조: [node0_cluster0, node0_label0, node0_cluster1, node0_label1, ..., node1_cluster0, node1_label0, ...]
# 각 노드의 라벨 개수는 node_label_counts로 알 수 있음
int32[] node_label_clusters
int32[] node_label_types

# 각 노드의 라벨 개수 (node_label_clusters와 node_label_types의 인덱싱에 사용)
# Aligned with nodes array (same length as num_nodes)
# 예: node_label_counts[0] = 2이면, node0은 2개의 라벨을 가짐
#     node_label_clusters[0], node_label_types[0]이 첫 번째 라벨의 클러스터 ID와 타입
#     node_label_clusters[1], node_label_types[1]이 두 번째 라벨의 클러스터 ID와 타입
int32[] node_label_counts

# Graph edges (as pairs of indices: [from_idx, to_idx, from_idx2, to_idx2, ...])
# Each consecutive pair represents an edge
int32[] edges

# Edge metrics aligned with edges array (same length as num_edges)
float32[] edge_lengths
float32[] edge_clearances

