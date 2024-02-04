import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp
import open3d as o3d

import numpy as np
import pcl

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'point_cloud_topic',  # Replace 'point_cloud_topic' with your actual topic name
            self.point_cloud_callback,
            10)

    def point_cloud_callback(self, msg):
        # Convert PointCloud2 message to numpy array
        points = rnp.point_cloud2.point_cloud2_to_array(msg)

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
    
        # Visualize the point cloud
        o3d.visualization.draw_geometries([pcd])
        


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
