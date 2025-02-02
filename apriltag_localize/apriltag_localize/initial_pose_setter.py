import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot/pose',
            self.pose_callback,
            1)
        self.initial_pose_set = False
        self.global_ekf_client = None
        self.local_ekf_client = None
        self.timer = self.create_timer(1.0, self.initialize_clients)
        self.futures = []  # Track pending service requests

    def initialize_clients(self):
        # Create service clients if they don't exist
        if not self.global_ekf_client:
            self.global_ekf_client = self.create_client(SetPose, '/global_ekf/set_pose')
        if not self.local_ekf_client:
            self.local_ekf_client = self.create_client(SetPose, '/local_ekf/set_pose')
        
        # Check if services are ready
        global_ready = self.global_ekf_client.wait_for_service(timeout_sec=0.5)
        local_ready = self.local_ekf_client.wait_for_service(timeout_sec=0.5)
        
        if global_ready and local_ready:
            self.timer.cancel()  # Stop checking once services are ready

    def pose_callback(self, msg):
        if self.initial_pose_set:
            return  # Ignore subsequent messages
        
        # Ensure services are fully ready
        if not (self.global_ekf_client and self.local_ekf_client):
            self.get_logger().warn('Service clients not initialized!')
            return
        
        if not (self.global_ekf_client.service_is_ready() and self.local_ekf_client.service_is_ready()):
            self.get_logger().warn('Services not ready yet. Skipping initialization.')
            return

        self.get_logger().info('Initializing EKFs...')
        
        # Prepare requests
        req_global = SetPose.Request()
        req_global.pose = msg
        req_global.pose.header.frame_id = "map"
        
        req_local = SetPose.Request()
        req_local.pose = msg
        req_local.pose.header.frame_id = "odom"

        # Send requests and track futures
        self.futures.append(self.global_ekf_client.call_async(req_global))
        self.futures.append(self.local_ekf_client.call_async(req_local))
        
        self.initial_pose_set = True  # Prevent re-initialization

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()