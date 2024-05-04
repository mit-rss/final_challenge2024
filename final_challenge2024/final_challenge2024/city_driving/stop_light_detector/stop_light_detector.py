#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from stop_msgs.msg import PixelLocation

# import your color segmentation algorithm; call this function in ros_image_callback!
from stop_light_detector import cd_color_segmentation


class StoplightDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.stoplight_pub = self.create_publisher(PixelLocation, "/relative_stoplight_px", 10)
        self.debug_pub = self.create_publisher(Image, "/stoplight_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Stoplight Detector Initialized")

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        

        bounding_box = cd_color_segmentation(image,None) #pass None because template does nothing
        cv2.rectangle(image, (bounding_box[0], bounding_box[1]), (bounding_box[2], bounding_box[3]), (0, 255, 0), 2)
        #If valid bounding box send message, invalid bounding box is ((0,0),(0,0))
        if (bounding_box[0] + bounding_box[1] + bounding_box[2] + bounding_box[3]) !=0:
            circle_height = bounding_box[3]-bounding_box[1]

            #base of stoplight is about 7.33 light circles (after measuring) from top of red light (i vibed it out from the picture but we can measure also)
            bot_y = bounding_box[3]+7.33*circle_height
            center_bot_x = int((bounding_box[0]+bounding_box[2])/2)

            pixel_msg = PixelLocation()
            pixel_msg.u = float(center_bot_x)
            pixel_msg.v = float(bot_y)

            self.stoplight_pub.publish(pixel_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    stoplight_detector = StoplightDetector()
    rclpy.spin(stoplight_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()