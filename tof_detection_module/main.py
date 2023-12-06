import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt  # for DBSCAN
from segmentation import plane_segmentation_and_clustering, save_segments, bounding_volumes_cluster

def open_pointcloud(path):
    pcd = o3d.io.read_point_cloud(path)
    o3d.visualization.draw_geometries([pcd])

def open_pointcloud_color(path):
    pcd = o3d.io.read_point_cloud(path)
    # Convert point cloud data to NumPy array
    points = np.asarray(pcd.points)

    # Create new Open3D point cloud 
    new_cloud = o3d.geometry.PointCloud()
    new_cloud.points = o3d.utility.Vector3dVector(points[:, :3])
    # Visualize the colored point cloud
    o3d.visualization.draw_geometries([new_cloud])

def bounding_volumes(path):
    pcd = o3d.io.read_point_cloud(path)

    aabb = pcd.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    obb = pcd.get_oriented_bounding_box()
    obb.color = (0, 1, 0)
    o3d.visualization.draw_geometries([pcd, aabb, obb])                          

if __name__ == "__main__":

    #dataset_path = "/home/paul/Schreibtisch/AFP/Point Clouds/box1.ply"
    dataset_path = "/home/paul/Schreibtisch/AFP/Point Clouds/LUCID Color-Assorted-Objects-on-Carpet.ply"
    #dataset_path = "/home/paul/Schreibtisch/AFP/Point Clouds/Tableau_Werkstueck_1.ply"

    #open_pointcloud(dataset_path)
    #open_pointcloud_color(dataset_path)
    #plane_segmentation_and_clustering(dataset_path)
    #bounding_volumes(dataset_path)
    bounding_volumes_cluster(dataset_path)
