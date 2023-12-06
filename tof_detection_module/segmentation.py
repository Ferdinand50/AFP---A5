import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt  # for DBSCAN


def plane_segmentation_and_clustering(path):
    # Open a point cloud from a PLY file
    #pcd = o3d.io.read_point_cloud("/home/paul/Schreibtisch/AFP/Point Clouds/LUCID_HLS003S-001_Boxes-3.ply")
    pcd = o3d.io.read_point_cloud(path)

    # Segment the floor using RANSAC algorithm
    plane_model, inliers = pcd.segment_plane(distance_threshold=100, ransac_n=3, num_iterations=15000)

    # Create point clouds for inliers and outliers based on the segmented floor
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # Convert point cloud data to NumPy arrays
    inlier_points = np.asarray(inlier_cloud.points)
    outlier_points = np.asarray(outlier_cloud.points)

    # Create new Open3D point clouds for inliers and outliers
    new_inlier_cloud = o3d.geometry.PointCloud()
    new_inlier_cloud.points = o3d.utility.Vector3dVector(inlier_points[:, :3])
    new_outlier_cloud = o3d.geometry.PointCloud()
    new_outlier_cloud.points = o3d.utility.Vector3dVector(outlier_points[:, :3])

    # Color the inlier and outlier point clouds for visualization
    new_inlier_cloud.paint_uniform_color([1, 0, 0])  # Red color
    new_outlier_cloud.paint_uniform_color([0, 0, 1])  # Blue color

    # Visualize the point clouds
    o3d.visualization.draw_geometries([new_inlier_cloud, new_outlier_cloud])

    # DBSCAN clustering on the outlier point cloud
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            new_outlier_cloud.cluster_dbscan(eps=87, min_points=16, print_progress=True))

    # Get the maximum label (number of clusters)
    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    # Color the points based on the DBSCAN cluster labels
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    new_outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # Visualize the clustered point cloud
    o3d.visualization.draw_geometries([new_outlier_cloud])

def save_segments(path):
        # Open a point cloud from a PLY file
    #pcd = o3d.io.read_point_cloud("/home/paul/Schreibtisch/AFP/Point Clouds/LUCID_HLS003S-001_Boxes-3.ply")
    pcd = o3d.io.read_point_cloud(path)

    # Segment the floor using RANSAC algorithm
    plane_model, inliers = pcd.segment_plane(distance_threshold=100, ransac_n=3, num_iterations=15000)

    # Create point clouds for inliers and outliers based on the segmented floor
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # Convert point cloud data to NumPy arrays
    inlier_points = np.asarray(inlier_cloud.points)
    outlier_points = np.asarray(outlier_cloud.points)

    # Create new Open3D point clouds for inliers and outliers
    new_inlier_cloud = o3d.geometry.PointCloud()
    new_inlier_cloud.points = o3d.utility.Vector3dVector(inlier_points[:, :3])
    new_outlier_cloud = o3d.geometry.PointCloud()
    new_outlier_cloud.points = o3d.utility.Vector3dVector(outlier_points[:, :3])

    # Color the inlier and outlier point clouds for visualization
    new_inlier_cloud.paint_uniform_color([1, 0, 0])  # Red color
    new_outlier_cloud.paint_uniform_color([0, 0, 1])  # Blue color

    # Visualize the point clouds
    o3d.visualization.draw_geometries([new_inlier_cloud, new_outlier_cloud])

    # DBSCAN clustering on the outlier point cloud
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            new_outlier_cloud.cluster_dbscan(eps=87, min_points=16, print_progress=True))

    # Get the maximum label (number of clusters)
    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    # Color the points based on the DBSCAN cluster labels
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    new_outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # Visualize the clustered point cloud
    o3d.visualization.draw_geometries([new_outlier_cloud])
    
    # Iterate through clusters and save each as a separate point cloud
    for cluster_label in range(max_label + 1):
        # Extract points belonging to the current cluster
        cluster_indices = np.where(labels == cluster_label)[0]
        cluster_points = new_outlier_cloud.select_by_index(cluster_indices)

        # Save the cluster as a point cloud file
        cluster_filename = f"cluster_{cluster_label}.ply"
        o3d.io.write_point_cloud(cluster_filename, cluster_points)

def bounding_volumes_cluster(path):
        # Open a point cloud from a PLY file
    #pcd = o3d.io.read_point_cloud("/home/paul/Schreibtisch/AFP/Point Clouds/LUCID_HLS003S-001_Boxes-3.ply")
    pcd = o3d.io.read_point_cloud(path)

    # Segment the floor using RANSAC algorithm
    plane_model, inliers = pcd.segment_plane(distance_threshold=100, ransac_n=3, num_iterations=15000)

    # Create point clouds for inliers and outliers based on the segmented floor
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # Convert point cloud data to NumPy arrays
    inlier_points = np.asarray(inlier_cloud.points)
    outlier_points = np.asarray(outlier_cloud.points)

    # Create new Open3D point clouds for inliers and outliers
    new_inlier_cloud = o3d.geometry.PointCloud()
    new_inlier_cloud.points = o3d.utility.Vector3dVector(inlier_points[:, :3])
    new_outlier_cloud = o3d.geometry.PointCloud()
    new_outlier_cloud.points = o3d.utility.Vector3dVector(outlier_points[:, :3])

    # Color the inlier and outlier point clouds for visualization
    new_inlier_cloud.paint_uniform_color([1, 0, 0])  # Red color
    new_outlier_cloud.paint_uniform_color([0, 0, 1])  # Blue color

    # Visualize the point clouds
    o3d.visualization.draw_geometries([new_inlier_cloud, new_outlier_cloud])

    # DBSCAN clustering on the outlier point cloud
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            new_outlier_cloud.cluster_dbscan(eps=87, min_points=16, print_progress=True))

    # Get the maximum label (number of clusters)
    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    # Color the points based on the DBSCAN cluster labels
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    new_outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # Visualize the clustered point cloud
    o3d.visualization.draw_geometries([new_outlier_cloud])
    

    # List to store individual cluster point clouds
    cluster_geometries = []

    # Iterate through clusters and create bounding boxes
    for cluster_label in range(max_label + 1):
        # Extract points belonging to the current cluster
        cluster_indices = np.where(labels == cluster_label)[0]
        cluster_points = new_outlier_cloud.select_by_index(cluster_indices)

        # Create a bounding box for the current cluster
        #aabb = cluster_points.get_axis_aligned_bounding_box()
        #aabb.color = (0, 1, 0)
        obb = cluster_points.get_oriented_bounding_box()
        obb.color = (1, 0, 0)

        # Add the cluster points to the list
        cluster_geometries.append(cluster_points)

        # Add the bounding box to the list
        #cluster_geometries.append(aabb)

        cluster_geometries.append(obb)

    print(cluster_geometries)
    # Visualize the combined geometries with bounding boxes
    o3d.visualization.draw_geometries(cluster_geometries)

if __name__ == "__main__":
    dataset_path = "/home/paul/Schreibtisch/AFP/Point Clouds/LUCID Color-Assorted-Objects-on-Carpet.ply"

    bounding_volumes_cluster(dataset_path)