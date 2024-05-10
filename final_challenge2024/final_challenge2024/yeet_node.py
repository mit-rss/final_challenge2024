import numpy as np
import rclpy
import tf2_ros
import tf_transformations

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
# from racecar_follower.np_encrypt import encode, decode
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float32
from time import sleep
import matplotlib.pyplot as plt



class YeetNode(Node):
    def __init__(self):
        super().__init__("yeet_node")
        self.declare_parameter("drive_topic", "/drive") 
        self.declare_parameter("base_frame","base_link")

        self.BASE_FRAME = "base_link" #self.get_parameter('base_frame').get_parameter_value().string_value
        self.DRIVE_TOPIC = "/vesc/high_level/input/nav_2" #self.get_parameter('drive_topic').get_parameter_value().string_value

        self.command_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 1)

    def test_basic_forward(self,speed):


        msg = AckermannDriveStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.BASE_FRAME
        msg.drive.speed = float(speed/2)
        msg.drive.steering_angle = 0.0

        self.command_pub.publish(msg)

        #wait for error info
        rclpy.spin_once(self,timeout_sec=45)

def main():
    rclpy.init()
    test = YeetNode()
    test.test_basic_forward(8.0)

    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()