import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import std_msgs.msg
import numpy as np

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud_topic', 10)
        self.timer_ = self.create_timer(1.0, self.publish_point_cloud)

    def publish_point_cloud(self):
        # Create a PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set the fields of the PointCloud2 message
        msg.height = 1
        msg.width = 4
        msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))

        # Generate some random point cloud data
        points = np.random.rand(4, 3).astype(np.float32)
        msg.data = points.tobytes()

        # Publish the PointCloud2 message
        self.publisher_.publish(msg)
        self.get_logger().info('Published point cloud')

def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher)
    point_cloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
