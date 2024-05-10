#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
# from stop_msgs.msg import PixelLocation
from std_msgs.msg import Bool

# import your color segmentation algorithm; call this function in ros_image_callback!
from color_segmentation import cd_color_segmentation


class StoplightDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("stop_light_detector")
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        # self.stoplight_pub = self.create_publisher(PixelLocation, "/relative_stoplight_px", 10)
        self.stoplight_pub = self.create_publisher(Bool, "/detects_red", 10)

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
        h,w = image.shape[:2] #mask top and bottom
        cv2.rectangle(image,(0,0),(w,int(h*.25)),(0,0,0),-1) #top rectangle
        cv2.rectangle(image, (0, h-int(h*.6)), (w, h), (0, 0, 0), -1)#bottom rectangle
        detects_red_msg = Bool()
        detects_red_msg.data = False

        bounding_box = cd_color_segmentation(image,None) #pass None because template does nothing
        cv2.rectangle(image, (bounding_box[0], bounding_box[1]), (bounding_box[2], bounding_box[3]), (0, 255, 0), 2)
        #If valid bounding box send message, invalid bounding box is ((0,0),(0,0))
        if (bounding_box[0] + bounding_box[1] + bounding_box[2] + bounding_box[3]) !=0:
            detects_red_msg.data = True
        
        self.stoplight_pub.publish(detects_red_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        # self.get_logger().info("Publishing debug")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    stoplight_detector = StoplightDetector()
    rclpy.spin(stoplight_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()