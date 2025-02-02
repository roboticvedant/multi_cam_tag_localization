import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose

class MapToOdomPublisher(Node):
    def __init__(self):
        super().__init__('map_to_odom_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Initialize listener
        self.subscription = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10)
        self.odom_to_base = None

    def pose_callback(self, msg):
        # Assume msg is in map frame. Get latest odom → base_link from TF.
        try:
            # Get transform from odom → base_link
            self.odom_to_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            
            # Compute map → odom = map → base_link * base_link → odom
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            
            # Compute transform (map → odom)
            # map → base_link (from camera) = transform * odom → base_link
            # Solve for transform (map → odom)
            # transform = map → base_link * (odom → base_link)^-1
            # This requires converting poses to transformations and composing them.
            # For simplicity, use TF2 to compute the inverse and compose.
            camera_pose = PoseStamped()
            camera_pose.header.frame_id = 'map'
            camera_pose.pose = msg.pose
            odom_to_base_transform = self.odom_to_base.transform
            map_to_odom_transform = do_transform_pose(camera_pose, odom_to_base_transform)
            
            transform.transform.translation.x = map_to_odom_transform.pose.position.x
            transform.transform.translation.y = map_to_odom_transform.pose.position.y
            transform.transform.translation.z = map_to_odom_transform.pose.position.z
            transform.transform.rotation = map_to_odom_transform.pose.orientation
            
            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            self.get_logger().error(f'TF error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()