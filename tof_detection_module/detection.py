import open3d as o3d
import numpy as np
import copy
import matplotlib.pyplot as plt  # for DBSCAN
from operator import itemgetter
import os

from tof_detection_module.CameraHandler import save_ply

#-------------------------------------------SETTINGS----------------------------------------------#
#gets the point cloud fromt the camera and saves it into the input dir
save_ply()
input = "src/tof_detection_module/point_cloud/scene.ply"  #finaler input ist kamerabild von ROS

# CAD Point cloud paths
target_clouds_directory = "src/tof_detection_module/cad_point_clouds/"

# Settings for the RANSAC algorithm
distance_threshold = 15         # maximum distance from a point to the fitted plane for the point to be considered as an inlier
ransac_n = 3                    # number of points randomly sampled to fit the plane model in each iteration
num_iterations = 15000

# Settings for the DBSCAN
eps = 60                        # maximum distance between two samples for one to be considered as part of the neighborhood of the other
min_points = 6000               # number of points required to form a cluster

# Set the threshold for correspondence distance in ICP (Iterative Closest Point) algorithm
icp_threshold = 15              # Adjust this threshold based on your specific use case

# Set number of rotations
num_rotations = 2
#-------------------------------------------------------------------------------------------------#


# Defines the rotation angle (90 degrees in radians)
rotation_angle = (2 * np.pi) / num_rotations

# Load target point clouds
target_clouds_paths = [os.path.join(target_clouds_directory, filename) for filename in os.listdir(target_clouds_directory) if filename.endswith(".ply")]
target_clouds = [o3d.io.read_point_cloud(path) for path in target_clouds_paths]

# List to store the source point clouds from segmentation
source_clouds = []

# List to store the fitness values for each pair of source and target
fitness_values = []

# Defines rotation and transformation matrix about z axis
rotation_matrix_z = np.array([
    [np.cos(rotation_angle), -np.sin(rotation_angle), 0],
    [np.sin(rotation_angle), np.cos(rotation_angle), 0],
    [0, 0, 1],
    ])
translation_vector = np.array([0, 0, 0])
transformation_matrix_z = np.eye(4)
transformation_matrix_z[:3, :3] = rotation_matrix_z
transformation_matrix_z[:3, 3] = translation_vector

# Function to visualize the registration result
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])
    # Visualize the point cloud and the coordinate frame
    o3d.visualization.draw_geometries([source_temp, target_temp, mesh_frame])

# Function to evaluate the fitness of a registration
def evaluate_registration(source, target, threshold, trans_init):
    # Evaluate the alignment
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    return evaluation.fitness

# Function to move a point cloud to its center of mass
def move_to_center_of_mass(point_cloud):
    center = point_cloud.get_center()
    translation = -center
    point_cloud.translate(translation)
    return translation

def rotate_about_z(point_cloud, rotation_angle):
    point_cloud.rotate(rotation_matrix_z)

def get_distance(pcd):
    #pcd = o3d.io.read_point_cloud(path)
    center_of_mass = np.asarray(pcd.get_center())
    # Calculate the length of the vector (magnitude)
    vector_length = np.linalg.norm(center_of_mass)
    # Print the coordinates of the center of mass
    print("Center of Mass Coordinates:")
    print(f"X: {center_of_mass[0]:.4f}")
    print(f"Y: {center_of_mass[1]:.4f}")
    print(f"Z: {center_of_mass[2]:.4f}")
    # Print the result
    print(f"Distance from TCP: {vector_length:.4f}")

def plane_segmentation_and_clustering(path): 
    # Open a point cloud from a PLY file
    pcd = o3d.io.read_point_cloud(path)

    # Segment the floor using RANSAC algorithm
    plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)

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
    #o3d.visualization.draw_geometries([new_inlier_cloud, new_outlier_cloud])

    # DBSCAN clustering on the outlier point cloud
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            new_outlier_cloud.cluster_dbscan(eps, min_points, print_progress=True))

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
        cluster_point_cloud = o3d.geometry.PointCloud()
        cluster_point_cloud.points = o3d.utility.Vector3dVector(np.asarray(cluster_points.points))
        # Add the cluster to list
        source_clouds.append(cluster_point_cloud)
 
def icp_registration():
    # Loop over each source point cloud
    for i, source in enumerate(source_clouds):
        source_temp = copy.deepcopy(source)
        
        # Apply the initial transformation to the source point cloud
        #source.transform(initial_transformation)
    
        # Move the source point cloud to its center of mass
        translation_source = move_to_center_of_mass(source_temp)
        
        # Loop over each target point cloud
        for j, target in enumerate(target_clouds):
            target_temp = copy.deepcopy(target)

            # Move the target point cloud to its center of mass
            translation_target = move_to_center_of_mass(target_temp)
            
            # Visualize the initial alignment
            #draw_registration_result(source_temp, target_temp, np.identity(4))  # No initial transformation

            for rotation in range(num_rotations):

                rotate_about_z(source_temp, rotation_angle)

                #draw_registration_result(source, target, np.identity(4))

                # Apply point-to-point ICP (Iterative Closest Point)
                reg_p2p = o3d.pipelines.registration.registration_icp(
                    source_temp, target_temp, icp_threshold, np.identity(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

                # Evaluate the final alignment
                final_fitness = evaluate_registration(source_temp, target_temp, icp_threshold, reg_p2p.transformation)
                print(f"fitness after {rotation} rotation of z axis initial rotation: {final_fitness}")
                #print(reg_p2p.transformation)

                # Visualize the result after ICP registration
                #draw_registration_result(source_temp, target_temp, reg_p2p.transformation)

                # Store the fitness value and transformation for this pair
                fitness_values.append({
                'transformation': reg_p2p.transformation,
                'fitness': final_fitness,
                'source': i,
                'target': j,
                'rotation': rotation
                })


def main():
    plane_segmentation_and_clustering(input)
    print(source_clouds)
    icp_registration()

    # Find the entry with the highest score
    best_entry = max(fitness_values, key=itemgetter('fitness'))
    print("Best fitness score: {} reached with {} rotations about the z axis".format(best_entry['fitness'],best_entry['rotation']))
    
    get_distance(source_clouds[best_entry['source']])

    # print matched Betriebsmittel
    print(f"Betriebsmittel: {os.path.splitext(os.path.basename(target_clouds_paths[best_entry['target']]))[0]}")

    # Calculate overall transformation matrix
    combined_transformation_matrix_z = np.linalg.matrix_power(transformation_matrix_z, best_entry['rotation']+1)
    overall_transformation_matrix = np.linalg.multi_dot([best_entry['transformation'],combined_transformation_matrix_z])

    print("Transformation Matrix:") 
    print(overall_transformation_matrix)

    # Visualize the initial alignment
    source_cloud = source_clouds[best_entry['source']]
    target_cloud = target_clouds[best_entry['target']]
    draw_registration_result(source_cloud, target_cloud, np.identity(4))  # No initial transformation

    # Visualizing the matching with best result:
    translation_source = move_to_center_of_mass(source_cloud)
    translation_target = move_to_center_of_mass(target_cloud)
    draw_registration_result(source_cloud, target_cloud, overall_transformation_matrix)
    
    
    
    
if __name__ == "__main__":
    main()

