# A node that just drives the car forward. Used for testing and visualization
import rclpy

from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class DrunkDriver(Node):
    def __init__(self):
        super().__init__("drunk_driver")
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.timer = self.create_timer(1 / 20, self.timer_cb)

    def timer_cb(self):
        msg = AckermannDriveStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 2.0
        msg.drive.steering_angle = 0.0

        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(DrunkDriver())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()