import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


class CalibCheckNode(Node):
    def __init__(self):
        super().__init__('calib_check')
        self.declare_parameter('frame_1', 'tag_0')
        self.declare_parameter('frame_2', 'tag_1')

        self.frame_1 = self.get_parameter('frame_1').get_parameter_value().string_value
        self.frame_2 = self.get_parameter('frame_2').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Live plotting setup
        self.distances = deque(maxlen=100)  # Store the latest 100 distance values in mm
        self.timestamps = deque(maxlen=100)  # Store timestamps for plotting

        # Start a timer for calculating the distance at 30 Hz
        self.timer = self.create_timer(0.033, self.check_distance)

        # Setup the live plot
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label=f"Distance: {self.frame_1} -> {self.frame_2}")
        self.ax.axhline(83, color='r', linestyle='--', label="83 mm Reference Line")  # Dotted reference line
        self.ax.set_xlim(0, 100)  # Adjust this if necessary
        self.ax.set_ylim(0, 200)  # Adjust this based on expected distances in mm
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Distance (mm)")
        self.ax.legend()
        self.start_time = self.get_clock().now().to_msg().sec

        # Animation for live plot
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        self.plotting = True
        plt.show(block=False)  # Non-blocking mode

    def check_distance(self):
        try:
            # Get the transform between the two frames
            transform = self.tf_buffer.lookup_transform(self.frame_1, self.frame_2, rclpy.time.Time())
            distance = self.compute_distance(transform.transform.translation) * 1000  # Convert to mm

            # Store distance and timestamp
            timestamp = self.get_clock().now().to_msg().sec - self.start_time
            self.distances.append(distance)
            self.timestamps.append(timestamp)

            self.get_logger().info(f"Distance between {self.frame_1} and {self.frame_2}: {distance:.2f} mm")
        except Exception as e:
            self.get_logger().warn(f"Failed to lookup transform: {str(e)}")

    def update_plot(self, frame):
        """Update the live plot with new data."""
        self.line.set_data(self.timestamps, self.distances)
        if self.timestamps:
            self.ax.set_xlim(max(0, self.timestamps[0] - 1), self.timestamps[-1] + 1)
        self.ax.set_ylim(0, max(max(self.distances, default=1) + 10, 100))  # Adjusted for better visibility
        self.ax.figure.canvas.draw()

    @staticmethod
    def compute_distance(translation):
        """Compute Euclidean distance from translation."""
        return math.sqrt(translation.x ** 2 + translation.y ** 2 + translation.z ** 2)

    def shutdown(self):
        """Graceful shutdown for ROS2 and plotting."""
        if self.plotting:
            plt.close('all')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    import matplotlib
    matplotlib.use('TkAgg')  # Ensure proper backend

    rclpy.init(args=args)
    node = CalibCheckNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            plt.pause(0.01)  # Keep the plot responsive
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
